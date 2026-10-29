// ============================================================================
// Weapons_Interop.cpp — IRADS target hand-off to DCS weapons
// ============================================================================
// All entry points are resolved by export name. Vtable slots were read from
// DCS 2.9.29.27468 CockpitBase.dll and must be re-checked after DCS updates:
//
// avSimpleWeaponSystem::launch_station (CockpitBase RVA 0x27AAC0):
//   manager = contextMovingObject->vtbl[0x120]()          weapon manager
//   manager->vtbl[0x0A0](station)                          select station
//   manager->vtbl[0x190](station, substation, targetId,
//                        const Vec3f* point, bool armed)   launch -> int
// avSimpleWeaponSystem::Launch:        manager->vtbl[0x168]() current station
// avSimpleWeaponSystem::get_weapon_count: manager->vtbl[0x0D0](station) count
// Launch::vtbl[0x1B8]()                                    station count
//
// avRangefinder::update_spot (CockpitBase RVA 0x33A2A0):
//   spot = woSpot::create_laser_spot(ownerId, &mountLocalPos, &aimWorld, code)
//   woSpot::set_point(spot, &aimWorld)
//   wSimCalendar::registerToKill(spot + <Suicide base>)   remove
//   ownerId = *(uint32*)(contextLinkHost - 0xE4)  (MovingObject+0x1C)
// ============================================================================

#include "stdafx.h"
#include "Weapons_Interop.h"
#include "CockpitBase_Interop.h"
#include "CockpitBase_Layouts.h"
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cmath>

namespace WeaponsInterop
{
    namespace
    {
        struct Vec3d { double x, y, z; };
        struct Vec3f { float x, y, z; };

        constexpr size_t kSlotGetWeaponManager = 0x120 / 8;  // MovingObject
        constexpr size_t kSlotSelectStation    = 0x0A0 / 8;  // weapon manager
        constexpr size_t kSlotWeaponCount      = 0x0D0 / 8;
        constexpr size_t kSlotCurrentStation   = 0x168 / 8;
        constexpr size_t kSlotLaunch           = 0x190 / 8;
        constexpr size_t kSlotStationCount     = 0x1B8 / 8;
        constexpr ptrdiff_t kLinkHostToOwnerId = -0xE4;
        constexpr size_t kSuicideScanBytes     = 0x400;

        using PFN_GetManager    = void* (__fastcall*)(void* self);
        using PFN_SelectStation = void  (__fastcall*)(void* self, int station);
        using PFN_WeaponCount   = int   (__fastcall*)(void* self, int station);
        using PFN_CurrentStation= int   (__fastcall*)(void* self);
        using PFN_StationCount  = int   (__fastcall*)(void* self);
        using PFN_Launch        = int   (__fastcall*)(void* self, int station, int substation,
                                                      unsigned int targetId, const Vec3f* point, bool armed);
        using PFN_CreateLaserSpot = void* (__cdecl*)(unsigned int ownerId, const Vec3d* localPos,
                                                     const Vec3d* aimPoint, unsigned int code);
        using PFN_SetPoint      = void  (__fastcall*)(void* spot, const Vec3d* point);
        using PFN_RegisterToKill= void  (__cdecl*)(void* suicide);

        bool               s_resolved        = false;
        bool               s_laserDisabled = false;
        bool               s_disabled        = false;   // set after a fault; stops further native calls
        PFN_CreateLaserSpot s_createLaserSpot = nullptr;
        PFN_SetPoint       s_setPoint        = nullptr;
        PFN_RegisterToKill s_registerToKill  = nullptr;
        const void*        s_spotSuicideVtbl = nullptr;

        void*        s_spot          = nullptr;
        unsigned int s_spotCode      = 0;
        ptrdiff_t    s_suicideOffset = -1;

        void Log(const char* fmt, ...)
        {
            char buf[512];
            va_list va;
            va_start(va, fmt);
            vsnprintf(buf, sizeof(buf), fmt, va);
            va_end(va);
            CockpitInterop::Log_(buf);
        }

        void Log_(const char* msg) { CockpitInterop::Log_(msg); }

        void Resolve()
        {
            if (s_resolved)
                return;


            HMODULE hWG = GetModuleHandleA("WorldGeneral.dll");
            HMODULE hW  = GetModuleHandleA("World.dll");
            if (hWG)
            {
                s_createLaserSpot = reinterpret_cast<PFN_CreateLaserSpot>(
                    GetProcAddress(hWG, "?create_laser_spot@woSpot@@SAPEAV1@IAEBVVec3d@osg@@0I@Z"));
                s_setPoint = reinterpret_cast<PFN_SetPoint>(
                    GetProcAddress(hWG, "?set_point@woSpot@@QEAAXAEBVVec3d@osg@@@Z"));
                s_spotSuicideVtbl = reinterpret_cast<const void*>(
                    GetProcAddress(hWG, "??_7woSpot@@6BSuicide@@@"));
            }
            if (hW)
            {
                s_registerToKill = reinterpret_cast<PFN_RegisterToKill>(
                    GetProcAddress(hW, "?registerToKill@wSimCalendar@@SAXPEAVSuicide@@@Z"));
            }
            s_resolved = s_createLaserSpot && s_setPoint && s_registerToKill && s_spotSuicideVtbl;
            Log("WeaponsInterop: create_laser_spot=%p set_point=%p registerToKill=%p woSpot::Suicide vtbl=%p",
                (void*)s_createLaserSpot, (void*)s_setPoint, (void*)s_registerToKill, s_spotSuicideVtbl);
        }

        CockpitInterop::CcCockpitContextLayout* Context()
        {
            if (!CockpitInterop::g_contexts_ptr_addr)
                return nullptr;
            return reinterpret_cast<CockpitInterop::CcCockpitContextLayout*>(*CockpitInterop::g_contexts_ptr_addr);
        }

        template <typename Fn>
        Fn Slot(void* object, size_t slot)
        {
            return reinterpret_cast<Fn>((*reinterpret_cast<void***>(object))[slot]);
        }

        // SEH wrappers. Kept free of C++ objects so __try is allowed.
        void* GetWeaponManagerRaw(void* movingObject)
        {
            __try
            {
                return Slot<PFN_GetManager>(movingObject, kSlotGetWeaponManager)(movingObject);
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                s_disabled = true;
                return nullptr;
            }
        }

        bool LaunchStationRaw(void* manager, int station, const Vec3f* point, int* outResult)
        {
            __try
            {
                if (Slot<PFN_WeaponCount>(manager, kSlotWeaponCount)(manager, station) <= 0)
                {
                    *outResult = 0;
                    return true;
                }
                Slot<PFN_SelectStation>(manager, kSlotSelectStation)(manager, station);
                *outResult = Slot<PFN_Launch>(manager, kSlotLaunch)(manager, station, -1, 0u, point, true);
                return true;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                s_disabled = true;
                return false;
            }
        }

        bool StationInfoRaw(void* manager, int* outCurrent, int* outCount)
        {
            __try
            {
                *outCurrent = Slot<PFN_CurrentStation>(manager, kSlotCurrentStation)(manager);
                *outCount   = Slot<PFN_StationCount>(manager, kSlotStationCount)(manager);
                return true;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                s_disabled = true;
                return false;
            }
        }

        void* CreateSpotRaw(unsigned int ownerId, const Vec3d* local, const Vec3d* aim, unsigned int code)
        {
            __try
            {
                return s_createLaserSpot(ownerId, local, aim, code);
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                s_laserDisabled = true;
                return nullptr;
            }
        }

        bool SetPointRaw(void* spot, const Vec3d* aim)
        {
            __try
            {
                s_setPoint(spot, aim);
                return true;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                return false;
            }
        }

        bool KillRaw(void* suicide)
        {
            __try
            {
                s_registerToKill(suicide);
                return true;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                return false;
            }
        }

        void* WeaponManager()
        {
            if (s_disabled)
                return nullptr;
            CockpitInterop::CcCockpitContextLayout* ctx = Context();
            if (!ctx || !ctx->movingObjectRaw)
                return nullptr;
            return GetWeaponManagerRaw(ctx->movingObjectRaw);
        }

        // woSpot's Suicide base offset differs between DCS builds; find it by
        // locating the exported Suicide-subobject vtable inside a live spot.
        ptrdiff_t FindSuicideOffset(void* spot)
        {
            if (!s_spotSuicideVtbl)
                return -1;
            const uintptr_t target = reinterpret_cast<uintptr_t>(s_spotSuicideVtbl);
            const auto* words = reinterpret_cast<const uintptr_t*>(spot);
            for (size_t i = 1; i < kSuicideScanBytes / sizeof(uintptr_t); ++i)
            {
                if (words[i] == target)
                    return static_cast<ptrdiff_t>(i * sizeof(uintptr_t));
            }
            return -1;
        }

        void KillSpot()
        {
            if (!s_spot)
                return;
            if (s_registerToKill && s_suicideOffset > 0)
            {
                if (!KillRaw(reinterpret_cast<char*>(s_spot) + s_suicideOffset))
                    Log_("WeaponsInterop: registerToKill faulted; dropping laser spot pointer");
            }
            else
            {
                Log_("WeaponsInterop: cannot remove laser spot (kill path unresolved); dropping pointer");
            }
            s_spot = nullptr;
        }
    }

    bool IsLaunchAvailable()
    {
        return WeaponManager() != nullptr;
    }

    bool LaunchAtPoint(const double worldPoint[3], int station)
    {
        if (!worldPoint || station < 0 || station > 8) return false;
        for (int i=0; i<3; ++i) if (!std::isfinite(worldPoint[i]) || std::abs(worldPoint[i])>1.0e8) return false;
        void* manager = WeaponManager();
        if (!manager) return false;
        int current = -1, count = 0;
        if (!StationInfoRaw(manager, &current, &count) || station >= count) return false;
        const Vec3f point{static_cast<float>(worldPoint[0]), static_cast<float>(worldPoint[1]), static_cast<float>(worldPoint[2])};
        int result = 0;
        if (!LaunchStationRaw(manager, station, &point, &result))
        {
            Log("WeaponsInterop: launch fault station=%d; disabled", station);
            return false;
        }
        Log("WeaponsInterop: launch station=%d target=(%.2f,%.2f,%.2f) result=%d", station,point.x,point.y,point.z,result);
        return result != 0; // matches native avSimpleWeaponSystem acceptance test
    }

    void UpdateLaser(bool wantOn, const double worldPoint[3], unsigned int code)
    {
        Resolve();

        if (!wantOn || s_laserDisabled || !worldPoint)
        {
            KillSpot();
            return;
        }

        // Changing code on a live spot is build-specific; recreate instead.
        if (s_spot && code != s_spotCode)
            KillSpot();

        for (int i=0; i<3; ++i) if (!std::isfinite(worldPoint[i])) { KillSpot(); return; }
        const Vec3d aim{ worldPoint[0], worldPoint[1], worldPoint[2] };
        if (s_spot)
        {
            if (!SetPointRaw(s_spot, &aim))
            {
                Log_("WeaponsInterop: set_point faulted; laser spot dropped");
                KillSpot();
                s_laserDisabled = true;
            }
            return;
        }

        if (!s_createLaserSpot || !s_setPoint || !s_registerToKill || !s_spotSuicideVtbl)
            return;

        CockpitInterop::CcCockpitContextLayout* ctx = Context();
        if (!ctx || !ctx->carrierLinkHostRaw)
            return;

        const unsigned int ownerId = *reinterpret_cast<const std::uint32_t*>(
            reinterpret_cast<const char*>(ctx->carrierLinkHostRaw) + kLinkHostToOwnerId);
        const Vec3d mount{ CockpitInterop::IRADS_POS_X, CockpitInterop::IRADS_POS_Y, CockpitInterop::IRADS_POS_Z };

        s_spot = CreateSpotRaw(ownerId, &mount, &aim, code);
        s_spotCode = code;
        if (!s_spot)
        {
            Log_("WeaponsInterop: create_laser_spot failed; laser disabled");
            return;
        }
        if (s_suicideOffset < 0)
            s_suicideOffset = FindSuicideOffset(s_spot);
        Log("WeaponsInterop: laser spot %p created owner=%u code=%u suicideOffset=0x%llX",
            s_spot, ownerId, code, static_cast<long long>(s_suicideOffset));
    }

    bool IsLaserFiring()
    {
        return s_spot != nullptr;
    }

    void Reset()
    {
        KillSpot();
        s_disabled = false;
        s_laserDisabled = false;
        s_suicideOffset = -1;
    }
}
