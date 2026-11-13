// ============================================================================
// Weapons_Interop.cpp — IRADS target hand-off to DCS weapons
// ============================================================================
// Store launch and laser spot calls go through exported DCS functions and the
// weapon manager interface. Tested with DCS 2.9.29.27468; the interface
// positions are build-specific and must be re-checked after DCS updates.
// ============================================================================

#include "stdafx.h"
#include "Weapons_Interop.h"
#include "CockpitBase_Interop.h"
#include "CockpitBase_Layouts.h"
#include <cstdint>
#include <cstdarg>
#include <cstdarg>
#include <cmath>

namespace Weapons_interop
{
namespace
{
struct Vec3d
{
    double x;
    double y;
    double z;
};
struct Vec3f
{
    float x;
    float y;
    float z;
};

constexpr size_t slot_get_weapon_manager = 0x120 / 8;  // MovingObject
constexpr size_t slot_select_station = 0x0A0 / 8;      // weapon manager
constexpr size_t slot_weapon_count = 0x0D0 / 8;
constexpr size_t slot_current_station = 0x168 / 8;
constexpr size_t slot_launch = 0x190 / 8;
constexpr size_t slot_station_count = 0x1B8 / 8;
constexpr ptrdiff_t link_host_to_owner_id = -0xE4;
constexpr size_t suicide_scan_bytes = 0x400;

using Pfn_get_manager = void*(__fastcall*)(void* self);
using Pfn_select_station = void(__fastcall*)(void* self, int station);
using Pfn_weapon_count = int(__fastcall*)(void* self, int station);
using Pfn_current_station = int(__fastcall*)(void* self);
using Pfn_station_count = int(__fastcall*)(void* self);
using Pfn_launch = int(__fastcall*)(void* self, int station, int substation, unsigned int target_id, const Vec3f* point,
                                    bool armed);
using Pfn_create_laser_spot = void*(__cdecl*)(unsigned int owner_id, const Vec3d* local_pos, const Vec3d* aim_point,
                                              unsigned int code);
using Pfn_set_point = void(__fastcall*)(void* spot, const Vec3d* point);
using Pfn_register_to_kill = void(__cdecl*)(void* suicide);

bool s_resolved = false;
bool s_laser_disabled = false;
bool s_disabled = false;  // set after a fault; stops further native calls
Pfn_create_laser_spot s_create_laser_spot = nullptr;
Pfn_set_point s_set_point = nullptr;
Pfn_register_to_kill s_register_to_kill = nullptr;
const void* s_spot_suicide_vtbl = nullptr;

void* s_spot = nullptr;
unsigned int s_spot_code = 0;
ptrdiff_t s_suicide_offset = -1;

void log_format(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    Cockpit_interop::log_vformat(fmt, va);
    va_end(va);
}

void log_message(const char* msg)
{
    Cockpit_interop::log_message(msg);
}

void resolve()
{
    if (s_resolved)
        return;

    HMODULE h_wg = GetModuleHandleA("WorldGeneral.dll");
    HMODULE h_w = GetModuleHandleA("World.dll");
    if (h_wg)
    {
        s_create_laser_spot = reinterpret_cast<Pfn_create_laser_spot>(
            GetProcAddress(h_wg, "?create_laser_spot@woSpot@@SAPEAV1@IAEBVVec3d@osg@@0I@Z"));
        s_set_point =
            reinterpret_cast<Pfn_set_point>(GetProcAddress(h_wg, "?set_point@woSpot@@QEAAXAEBVVec3d@osg@@@Z"));
        s_spot_suicide_vtbl = reinterpret_cast<const void*>(GetProcAddress(h_wg, "??_7woSpot@@6BSuicide@@@"));
    }
    if (h_w)
    {
        s_register_to_kill = reinterpret_cast<Pfn_register_to_kill>(
            GetProcAddress(h_w, "?registerToKill@wSimCalendar@@SAXPEAVSuicide@@@Z"));
    }
    s_resolved = s_create_laser_spot && s_set_point && s_register_to_kill && s_spot_suicide_vtbl;
    log_format("WeaponsInterop: create_laser_spot=%p set_point=%p registerToKill=%p woSpot::Suicide vtbl=%p",
               reinterpret_cast<void*>(s_create_laser_spot), reinterpret_cast<void*>(s_set_point),
               reinterpret_cast<void*>(s_register_to_kill), s_spot_suicide_vtbl);
}

Cockpit_interop::Cc_cockpit_context_layout* context()
{
    if (!Cockpit_interop::g_contexts_ptr_addr)
        return nullptr;
    return reinterpret_cast<Cockpit_interop::Cc_cockpit_context_layout*>(*Cockpit_interop::g_contexts_ptr_addr);
}

template <typename Fn>
Fn slot(void* object, size_t slot)
{
    return reinterpret_cast<Fn>((*reinterpret_cast<void***>(object))[slot]);
}

// SEH wrappers. Kept free of C++ objects so __try is allowed.
void* get_weapon_manager_raw(void* moving_object)
{
    __try
    {
        return slot<Pfn_get_manager>(moving_object, slot_get_weapon_manager)(moving_object);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return nullptr;
    }
}

bool launch_station_raw(void* manager, int station, const Vec3f* point, int* out_result)
{
    __try
    {
        if (slot<Pfn_weapon_count>(manager, slot_weapon_count)(manager, station) <= 0)
        {
            *out_result = 0;
            return true;
        }
        slot<Pfn_select_station>(manager, slot_select_station)(manager, station);
        *out_result = slot<Pfn_launch>(manager, slot_launch)(manager, station, -1, 0u, point, true);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return false;
    }
}

bool station_info_raw(void* manager, int* out_current, int* out_count)
{
    __try
    {
        *out_current = slot<Pfn_current_station>(manager, slot_current_station)(manager);
        *out_count = slot<Pfn_station_count>(manager, slot_station_count)(manager);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return false;
    }
}

void* create_spot_raw(unsigned int owner_id, const Vec3d* local, const Vec3d* aim, unsigned int code)
{
    __try
    {
        return s_create_laser_spot(owner_id, local, aim, code);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_laser_disabled = true;
        return nullptr;
    }
}

bool set_point_raw(void* spot, const Vec3d* aim)
{
    __try
    {
        s_set_point(spot, aim);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool kill_raw(void* suicide)
{
    __try
    {
        s_register_to_kill(suicide);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Stock time-of-flight predictor, exported by the weapons library as a small handle object. Resolved lazily.
using Pfn_tof_ctor = void*(__fastcall*)(void* self);
using Pfn_gbu_tof = double(__fastcall*)(void* self, double a, double b, double c, double d, double e);
bool s_tof_resolved = false;
Pfn_tof_ctor s_tof_ctor = nullptr;
Pfn_gbu_tof s_gbu_tof = nullptr;

bool resolve_tof()
{
    if (s_tof_resolved)
        return s_tof_ctor && s_gbu_tof;
    s_tof_resolved = true;
    HMODULE h = GetModuleHandleA("WeaponsBase.dll");
    if (h)
    {
        s_tof_ctor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??0TOFEstimator@@QEAA@XZ"));
        s_gbu_tof =
            reinterpret_cast<Pfn_gbu_tof>(GetProcAddress(h, "?getGBU24TOF@TOFEstimator@@QEAANNNNNN@Z"));
    }
    log_format("WeaponsInterop: tof predictor ctor=%p call=%p", reinterpret_cast<void*>(s_tof_ctor),
               reinterpret_cast<void*>(s_gbu_tof));
    return s_tof_ctor && s_gbu_tof;
}

bool call_gbu_tof_raw(double a, double b, double c, double d, double e, double* out)
{
    alignas(16) unsigned char handle[64] = {};
    __try
    {
        s_tof_ctor(handle);
        *out = s_gbu_tof(handle, a, b, c, d, e);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_tof_ctor = nullptr;  // stop calling after a fault
        return false;
    }
}

// Stock GPS-bomb time-of-flight predictor, a larger state object with its own constructor. Resolved lazily.
using Pfn_jdam_tof = double(__fastcall*)(void* self, double alt, double speed, double pitch, double bank_a,
                                         double bank_b, double azimuth, double range, double target_alt,
                                         double override_a, double override_b, int type);
bool s_jdam_resolved = false;
Pfn_tof_ctor s_jdam_ctor = nullptr;
Pfn_tof_ctor s_jdam_dtor = nullptr;
Pfn_jdam_tof s_jdam_tof = nullptr;

bool resolve_jdam_tof()
{
    if (s_jdam_resolved)
        return s_jdam_ctor && s_jdam_tof;
    s_jdam_resolved = true;
    HMODULE h = GetModuleHandleA("WeaponsBase.dll");
    if (h)
    {
        s_jdam_ctor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??0LARBuilder@@QEAA@XZ"));
        s_jdam_dtor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??1LARBuilder@@QEAA@XZ"));
        s_jdam_tof = reinterpret_cast<Pfn_jdam_tof>(
            GetProcAddress(h, "?getJDAMTOF@LARBuilder@@QEAANNNNNNNNNNNW4JDAM_Type@1@@Z"));
    }
    log_format("WeaponsInterop: jdam tof predictor ctor=%p call=%p", reinterpret_cast<void*>(s_jdam_ctor),
               reinterpret_cast<void*>(s_jdam_tof));
    return s_jdam_ctor && s_jdam_tof;
}

bool call_jdam_tof_raw(int type, double alt, double speed, double pitch, double range, double target_alt, double* out)
{
    alignas(16) unsigned char state[1024] = {};
    __try
    {
        s_jdam_ctor(state);
        // The predictor refines its release solution over repeated calls; a fresh object answers 0 at first.
        // Repeat until two consecutive answers agree.
        double previous = 0.0;
        *out = 0.0;
        for (int pass = 0; pass < 60; ++pass)
        {
            *out = s_jdam_tof(state, alt, speed, pitch, 0.0, 0.0, 0.0, range, target_alt, -1.0, -1.0, type);
            if (*out > 0.0 && std::fabs(*out - previous) < 0.01)
                break;
            previous = *out;
        }
        if (s_jdam_dtor)
            s_jdam_dtor(state);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_jdam_ctor = nullptr;  // stop calling after a fault
        return false;
    }
}

void* weapon_manager()
{
    if (s_disabled)
        return nullptr;
    Cockpit_interop::Cc_cockpit_context_layout* ctx = context();
    if (!ctx || !ctx->moving_object_raw)
        return nullptr;
    return get_weapon_manager_raw(ctx->moving_object_raw);
}

// This position differs between DCS builds; find it by locating the
// exported table inside a live spot.
ptrdiff_t find_suicide_offset(void* spot)
{
    if (!s_spot_suicide_vtbl)
        return -1;
    const uintptr_t target = reinterpret_cast<uintptr_t>(s_spot_suicide_vtbl);
    const auto* words = reinterpret_cast<const uintptr_t*>(spot);
    for (size_t i = 1; i < suicide_scan_bytes / sizeof(uintptr_t); ++i)
    {
        if (words[i] == target)
            return static_cast<ptrdiff_t>(i * sizeof(uintptr_t));
    }
    return -1;
}

void kill_spot()
{
    if (!s_spot)
        return;
    if (s_register_to_kill && s_suicide_offset > 0)
    {
        if (!kill_raw(reinterpret_cast<char*>(s_spot) + s_suicide_offset))
            log_message("WeaponsInterop: registerToKill faulted; dropping laser spot pointer");
    }
    else
    {
        log_message("WeaponsInterop: cannot remove laser spot (kill path unresolved); dropping pointer");
    }
    s_spot = nullptr;
}
}  // namespace

bool is_launch_available()
{
    return weapon_manager() != nullptr;
}

bool launch_at_point(const double world_point[3], int station)
{
    if (!world_point || station < 0 || station > 8)
        return false;
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(world_point[i]) || std::abs(world_point[i]) > 1.0e8)
            return false;
    void* manager = weapon_manager();
    if (!manager)
        return false;
    int current = -1;
    int count = 0;
    if (!station_info_raw(manager, &current, &count) || station >= count)
        return false;
    const Vec3f point{static_cast<float>(world_point[0]), static_cast<float>(world_point[1]),
                      static_cast<float>(world_point[2])};
    int result = 0;
    if (!launch_station_raw(manager, station, &point, &result))
    {
        log_format("WeaponsInterop: launch fault station=%d; disabled", station);
        return false;
    }
    log_format("WeaponsInterop: launch station=%d target=(%.2f,%.2f,%.2f) result=%d", station, point.x, point.y,
               point.z, result);
    return result != 0;  // matches native avSimpleWeaponSystem acceptance test
}

bool stock_laser_bomb_tof(double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                          double ground_range_m, double* out)
{
    if (!out || !resolve_tof())
        return false;
    return call_gbu_tof_raw(release_alt_m, release_speed_mps, pitch_rad, target_alt_m, ground_range_m, out);
}

bool stock_jdam_tof(int type, double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                    double ground_range_m, double* out)
{
    if (!out || !resolve_jdam_tof())
        return false;
    return call_jdam_tof_raw(type, release_alt_m, release_speed_mps, pitch_rad, ground_range_m, target_alt_m, out);
}

void update_laser(bool want_on, const double world_point[3], unsigned int code)
{
    resolve();

    if (!want_on || s_laser_disabled || !world_point)
    {
        kill_spot();
        return;
    }

    // Changing code on a live spot is build-specific; recreate instead.
    if (s_spot && code != s_spot_code)
        kill_spot();

    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(world_point[i]))
        {
            kill_spot();
            return;
        }
    const Vec3d aim{world_point[0], world_point[1], world_point[2]};
    if (s_spot)
    {
        if (!set_point_raw(s_spot, &aim))
        {
            log_message("WeaponsInterop: set_point faulted; laser spot dropped");
            kill_spot();
            s_laser_disabled = true;
        }
        return;
    }

    if (!s_create_laser_spot || !s_set_point || !s_register_to_kill || !s_spot_suicide_vtbl)
        return;

    Cockpit_interop::Cc_cockpit_context_layout* ctx = context();
    if (!ctx || !ctx->carrier_link_host_raw)
        return;

    const unsigned int owner_id = *reinterpret_cast<const std::uint32_t*>(
        reinterpret_cast<const char*>(ctx->carrier_link_host_raw) + link_host_to_owner_id);
    const Vec3d mount{Cockpit_interop::irads_pos_x, Cockpit_interop::irads_pos_y, Cockpit_interop::irads_pos_z};

    s_spot = create_spot_raw(owner_id, &mount, &aim, code);
    s_spot_code = code;
    if (!s_spot)
    {
        log_message("WeaponsInterop: create_laser_spot failed; laser disabled");
        return;
    }
    if (s_suicide_offset < 0)
        s_suicide_offset = find_suicide_offset(s_spot);
    log_format("WeaponsInterop: laser spot %p created owner=%u code=%u suicideOffset=0x%llX", s_spot, owner_id, code,
               static_cast<long long>(s_suicide_offset));
}

bool is_laser_firing()
{
    return s_spot != nullptr;
}

void reset()
{
    kill_spot();
    s_disabled = false;
    s_laser_disabled = false;
    s_suicide_offset = -1;
}
}  // namespace Weapons_interop
