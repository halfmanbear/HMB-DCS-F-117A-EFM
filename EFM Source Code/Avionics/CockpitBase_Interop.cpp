// ============================================================================
// CockpitBase_Interop.cpp — two-phase avTVSensor injection
// ============================================================================
// Phase A (DllMain): construct avTVSensor + devices_keeper::add()
//   Runs as early as possible. If ccCockpitContext is not yet initialised
//   (contexts_ptr is NULL), the attempt is silently skipped; Phase B will
//   retry before doing anything else.
//   device_init.lua must have NO creators[devices.IRADS] entry.
//
// Phase B (ed_fm_cold/hot_start):
//   1. Retry Phase A if it was skipped (contexts_ptr was NULL at DllMain time).
//   2. Verify devices_keeper[9] still holds our sensor.
//   3. Ensure avTVSensor::initialize() has run — check byte+0x260 (active flag).
//      If not, call it explicitly.  initialize() creates the search timer,
//      calls initLimits (→ "TV_SENSOR_V_70X100"), and cages the gimbal.
//   4. Override the camera mount position via vtable[4] with the F-117A DLIR
//      belly coordinates. This overrides whatever avPlatform::initialize()
//      read from the (absent) model sensor station.
//   5. Call SET_FLIR_TECHINIQE_DEFAULT to activate the IR render pass.
//
// Crash fix A — avDevice+0x50 (lua_State*):
//   avTVSensor ctor writes double -1.0 (0xBFF0000000000000) at +0x50, which
//   overlays the avDevice base-class lua_State* slot.  avDevice::start() passes
//   that pointer to Lua::Config::Config → lua_gettop → ACCESS_VIOLATION.
//   Fix: zero +0x50 immediately after the ctor returns.
//
// Crash fix B — avDevice+0x58 (subsystem pointer vs. camera zoom double):
//   Ghidra decompilation of avDevice::start (RVA 0x236570) shows:
//
//     (**(code **)(*(longlong *)this + 0x38))();     // vtable[7](this)
//     ...
//     plVar1 = *(longlong **)(this + 0x58);
//     if (plVar1 != NULL) {
//         (**(code **)(*plVar1 + 0x20))(plVar1, 0); // ← CRASH
//     }
//
//   avDevice uses +0x58 as a subsystem object pointer (null = no subsystem).
//   avTVSensor ctor writes double 1.0 (0x3FF0000000000000) there as a camera
//   zoom/scale parameter. Non-null → avDevice::start dereferences 1.0 as an
//   address → ACCESS_VIOLATION.
//
//   vtable[7] is called first and reads +0x58 = 1.0 for camera setup, so we
//   cannot zero it before that virtual call.
//
//   Fix: install a per-instance vtable hook on slot 7. Our wrapper calls the
//   real virtual function with all camera params intact, then zeros +0x50 and
//   +0x58 before avDevice::start reads them as pointers.
//
// avTVSensor object layout (from CockpitBase.dll decompilation):
//   +0x000  primary vtable ptr (avPlatform face)
//   +0x050  double -1.0 written by ctor → avDevice lua_State* slot (zero it)
//   +0x058  double  1.0 written by ctor → avDevice subsystem* slot (zero after vtable[7])
//   +0x260  byte active flag — set to 1 only by initialize()
//   +0x268  avBasicSensor_SearchTimer*  — null until initialize() runs
//   +0x2C8  avMotor_Additive  (azimuth slew, offset 712)
//   +0x2F8  avMotor_Additive  (elevation slew, offset 760)
//   +0x328  Vec3d cached tracked/designated world point
//   +0x340  byte mode — 3 means actively tracking
//   +0x348  base FOV
//   +0x358  current zoom
//   +0x3B0  avTrackData       (tracking state, offset 944)
//   +0x4C0  camera aspect ratio
//   +0x4C8  sensor_frame      (current video frame, offset 1224)
//   +0x610  hat-lock flag
//   +0x648  avSensorLimits    ("EMPTY" after ctor, "TV_SENSOR_V_70X100" after initLimits)
//   total object size: 1776 bytes (alloc 0x800 = 2048 — sufficient)
// ============================================================================

#include "stdafx.h"
#include "CockpitBase_Interop.h"
#include "CockpitBase_Layouts.h"
#include "../Inputs.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <malloc.h>
#include <string>

namespace CockpitInterop
{
    static constexpr const char* kIRADSBuildMarker = "2026-04-23-1638-diag-groundstab-v1";

    // ── Globals ───────────────────────────────────────────────────────────────
    HMODULE                   g_hDLL                   = nullptr;
    PFN_avTVSensor_ctor       g_avTVSensor_ctor        = nullptr;
    PFN_avTVSensor_initialize g_avTVSensor_initialize  = nullptr;
    PFN_avTVSensor_initZoom   g_avTVSensor_initZoom    = nullptr;
    PFN_devices_keeper_add    g_devices_keeper_add     = nullptr;
    PFN_devices_keeper_get    g_devices_keeper_get     = nullptr;
    PFN_SetFlirDefault        g_setFlirDefault         = nullptr; // export does not exist; kept for ABI compat
    PFN_avDevice_start        g_avDevice_start         = nullptr;
    PFN_avTVSensor_update      g_avTVSensor_update       = nullptr;
    PFN_avTVSensor_update_frame g_avTVSensor_update_frame = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_left    = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_right   = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_up      = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_down    = nullptr;
    PFN_avTVSensor_abs_axis    g_avTVSensor_move_horizontal_abs = nullptr;
    PFN_avTVSensor_abs_axis    g_avTVSensor_move_vertical_abs   = nullptr;
    PFN_avTVSensor_set_external_designation g_avTVSensor_set_external_designation = nullptr;
    PFN_avTVSensor_get_tracked_point g_avTVSensor_get_tracked_point = nullptr;
    PFN_avTVSensor_get_polar_position g_avTVSensor_get_polar_position = nullptr;
    PFN_avTVSensor_stabilize_on_ground g_avTVSensor_stabilize_on_ground = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_search = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_uncage = nullptr;
    void (__fastcall*          g_avTVSensor_slew_stop)(void* self) = nullptr;
    void (__fastcall*          g_avTVSensor_zoom_in)(void* self) = nullptr;
    void (__fastcall*          g_avTVSensor_zoom_out)(void* self) = nullptr;
    void**                    g_contexts_ptr_addr      = nullptr;
    void*                     g_avTVSensor_instance    = nullptr;
    PFN_avPlatform_ctor       g_avPlatform_ctor        = nullptr;
    PFN_avPlatform_set_carrier g_avPlatform_set_carrier= nullptr;
    PFN_avPlatform_update_pos g_avPlatform_update_pos  = nullptr;
    PFN_binding_context       g_binding_context        = nullptr;
    void*                     g_stub_platform          = nullptr;

    static bool  s_initialized  = false;
    static bool  s_added        = false;
    static bool  s_started      = false;
    static bool  s_carrier_set  = false;
    static bool  s_carrier_link_disabled_logged = false;
    static bool  s_link_host_seed_logged = false;
    static int   s_link_host_seed_fail_logs = 0;
    static int   s_designation_diag_logs = 0;
    static bool  s_pose_seed_logged = false;
    static FILE* s_log          = nullptr;
    static float s_irads_horizontal_axis = 0.0f;
    static float s_irads_vertical_axis   = 0.0f;
    static bool  s_irads_lock_valid      = false;
    static bool  s_irads_designating     = false;
    static double s_last_aircraft_pos[3]  = {0.0, 0.0, 0.0};
    static double s_last_aircraft_quat[4] = {0.0, 0.0, 0.0, 1.0};
    static double s_irads_lock_world[3]   = {0.0, 0.0, 0.0};
    static double s_irads_lock_polar[3]   = {0.0, 0.0, 0.0};

    // ── Sensor state accessors ────────────────────────────────────────────────
    // Vtable slot indices (multiply by 8 for byte offset into vtable).
    static constexpr size_t VTABLE_SLOT_SET_POS  = 4;     // avPlatform::setPlatformLocalPosition
    static constexpr size_t VTABLE_SLOT_LOAD_CFG = 7;     // avDevice::start calls this first

    static AvTVSensorLayout* AsSensor(void* sensor)
    {
        return static_cast<AvTVSensorLayout*>(sensor);
    }

    static CcCockpitContextLayout* AsContext(void* context)
    {
        return static_cast<CcCockpitContextLayout*>(context);
    }

    static CcIndicatorLayout* AsIndicator(void* indicator)
    {
        return static_cast<CcIndicatorLayout*>(indicator);
    }

    static AvPlatformLayout* AsPlatform(void* platform)
    {
        return static_cast<AvPlatformLayout*>(platform);
    }

    static void Log(const char* fmt, ...);
    static void SeedInjectedSensorPose(double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w);
    static bool SeedInjectedSensorLinkHost();

    static AvTrackDataLayout* AsTrackData(void* track)
    {
        return static_cast<AvTrackDataLayout*>(track);
    }

    static CcCockpitContextLayout* GetGlobalContext()
    {
        return (g_contexts_ptr_addr && *g_contexts_ptr_addr) ? AsContext(*g_contexts_ptr_addr) : nullptr;
    }

    static CcCockpitContextLayout* GetBindingContext()
    {
        return g_binding_context ? AsContext(g_binding_context()) : nullptr;
    }

    static bool FindIndicatorInContextTree(
        CcCockpitContextLayout* ctx,
        void** out_indicator,
        int* out_depth,
        int depth,
        bool allow_force_link)
    {
        if (!ctx || depth > 8)
            return false;

        void** arr_start = ctx->indicators.begin;
        void** arr_end   = ctx->indicators.end;
        if (arr_start && arr_end && arr_end > arr_start)
        {
            for (void** p = arr_start; p < arr_end; ++p)
            {
                void* ind = *p;
                if (!ind)
                    continue;

                CcIndicatorLayout* v = AsIndicator(ind);
                if (v->controller == g_avTVSensor_instance)
                {
                    *out_indicator = ind;
                    if (out_depth) *out_depth = depth;
                    return true;
                }
            }

            if (allow_force_link)
            {
                for (void** p = arr_start; p < arr_end; ++p)
                {
                    void* ind = *p;
                    if (!ind)
                        continue;

                    CcIndicatorLayout* v = AsIndicator(ind);
                    if (v->indicatorType == 0 &&
                        static_cast<int32_t>(v->renderTargetId) == 0 &&
                        (v->controller == nullptr || v->controller == g_avTVSensor_instance))
                    {
                        v->controller = g_avTVSensor_instance;
                        *out_indicator = ind;
                        if (out_depth) *out_depth = depth;
                        return true;
                    }
                }
            }
        }

        void** child_start = ctx->childContexts.begin;
        void** child_end   = ctx->childContexts.end;
        if (child_start && child_end && child_end > child_start)
        {
            for (void** p = child_start; p < child_end; ++p)
            {
                CcCockpitContextLayout* child = AsContext(*p);
                if (FindIndicatorInContextTree(child, out_indicator, out_depth, depth + 1, allow_force_link))
                    return true;
            }
        }

        return false;
    }

    static bool GetSensorTrackPoint(double out_point[3])
    {
        if (!g_avTVSensor_instance)
            return false;

        AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);

        // Mode 3 = active object-tracking; any other active mode returns the cached
        // designated/stabilised ground point at +0x328. Both are valid for CCRP.
        // We use the native function for mode 3, and read +0x328 directly for other modes.
        if (sensor->trackMode == 3)
        {
            if (g_avTVSensor_get_tracked_point)
            {
                double nativePoint[3] = {};
                g_avTVSensor_get_tracked_point(g_avTVSensor_instance, nativePoint);
                out_point[0] = nativePoint[0];
                out_point[1] = nativePoint[1];
                out_point[2] = nativePoint[2];
                return (out_point[0] != 0.0 || out_point[1] != 0.0 || out_point[2] != 0.0);
            }

            // Fallback: matrix multiply from avTrackData (confirmed correct against binary)
            auto* track = AsTrackData(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x3B0);
            const double* m = track->worldTransform;
            const double local_x = track->localPointOffset[0];
            const double local_y = track->localPointOffset[1];
            const double local_z = track->localPointOffset[2];
            out_point[0] = local_x * m[0]  + local_y * m[4]  + local_z * m[8]  + m[12];
            out_point[1] = local_x * m[1]  + local_y * m[5]  + local_z * m[9]  + m[13];
            out_point[2] = local_x * m[2]  + local_y * m[6]  + local_z * m[10] + m[14];
            return (out_point[0] != 0.0 || out_point[1] != 0.0 || out_point[2] != 0.0);
        }

        // Non-tracking modes: read cached designated/stabilised point at +0x328.
        // This is populated by set_external_designation → mode 4/5 stabilisation loop.
        out_point[0] = sensor->cachedTrackedPoint[0];
        out_point[1] = sensor->cachedTrackedPoint[1];
        out_point[2] = sensor->cachedTrackedPoint[2];
        return (out_point[0] != 0.0 || out_point[1] != 0.0 || out_point[2] != 0.0);
    }

    static void GetSensorPolar(double out_polar[3])
    {
        out_polar[0] = 0.0;
        out_polar[1] = 0.0;
        out_polar[2] = 0.0;

        if (g_avTVSensor_instance && g_avTVSensor_get_polar_position)
        {
            g_avTVSensor_get_polar_position(g_avTVSensor_instance, out_polar);
            return;
        }

        auto* track = AsTrackData(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x3B0);
        out_polar[0] = track->relativeAzimuth;
        out_polar[1] = track->relativeElevation;
        out_polar[2] = track->relativeRange;
    }

    static bool TryCaptureIRADSLock()
    {
        double world[3];
        if (!GetSensorTrackPoint(world))
            return false;

        s_irads_lock_world[0] = world[0];
        s_irads_lock_world[1] = world[1];
        s_irads_lock_world[2] = world[2];
        GetSensorPolar(s_irads_lock_polar);
        s_irads_lock_valid = true;
        if (s_designation_diag_logs < 8)
        {
            AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);
            void* sensorLink = *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20);
            Log("TryCaptureIRADSLock: mode=%u cached=(%.2f, %.2f, %.2f) polar=(%.2f, %.2f, %.2f) frame=(%.4f, %.4f) pose=(%.2f, %.2f, %.2f) link=%p",
                (unsigned)sensor->trackMode,
                sensor->cachedTrackedPoint[0], sensor->cachedTrackedPoint[1], sensor->cachedTrackedPoint[2],
                s_irads_lock_polar[0], s_irads_lock_polar[1], s_irads_lock_polar[2],
                sensor->currentFrame.halfAzimuth, sensor->currentFrame.halfElevation,
                sensor->platformWorldPose.m[12], sensor->platformWorldPose.m[13], sensor->platformWorldPose.m[14],
                sensorLink);
            ++s_designation_diag_logs;
        }
        return true;
    }

    static bool SeedInjectedSensorLinkHost()
    {
        if (!g_avTVSensor_instance)
            return false;

        CcCockpitContextLayout* ctx = GetGlobalContext();
        CcCockpitContextLayout* bindingCtx = GetBindingContext();
        if (!ctx)
            ctx = bindingCtx;

        void* linkHost = ctx ? ctx->carrierLinkHostRaw : nullptr;
        if (!linkHost && bindingCtx && bindingCtx != ctx)
            linkHost = bindingCtx->carrierLinkHostRaw;
        if (!linkHost)
        {
            if (s_link_host_seed_fail_logs < 6)
            {
                Log("SeedInjectedSensorLinkHost: no host globalCtx=%p moving=%p host=%p bindingCtx=%p bindingMoving=%p bindingHost=%p sensorLink=%p",
                    (void*)ctx,
                    ctx ? ctx->movingObjectRaw : nullptr,
                    ctx ? ctx->carrierLinkHostRaw : nullptr,
                    (void*)bindingCtx,
                    bindingCtx ? bindingCtx->movingObjectRaw : nullptr,
                    bindingCtx ? bindingCtx->carrierLinkHostRaw : nullptr,
                    *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20));
                ++s_link_host_seed_fail_logs;
            }
            return false;
        }

        // avPlatform::get_carrier_position() reads the LinkBase host pointer from self+0x20.
        // set_carrier() seeds that through LinkBase::Set(MovingObject+0x100). For the injected
        // sensor we bypass set_carrier and copy the already-live cockpit host pointer directly.
        *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20) = linkHost;

        if (!s_link_host_seed_logged)
        {
            Log("SeedInjectedSensorLinkHost: context=%p binding=%p linkHost=%p",
                (void*)ctx, (void*)bindingCtx, linkHost);
            s_link_host_seed_logged = true;
        }
        return true;
    }

    static bool SeedInjectedSensorCarrierLinkBase()
    {
        if (!g_avTVSensor_instance)
        {
            if (s_link_host_seed_fail_logs < 6)
            {
                Log("SeedInjectedSensorCarrierLinkBase: no sensor instance");
                ++s_link_host_seed_fail_logs;
            }
            return false;
        }

        CcCockpitContextLayout* ctx = GetGlobalContext();
        CcCockpitContextLayout* bindingCtx = GetBindingContext();
        if (!ctx)
            ctx = bindingCtx;

        CcCockpitContextLayout* sourceCtx = nullptr;
        if (ctx && ctx->carrierLinkHostRaw)
            sourceCtx = ctx;
        else if (bindingCtx && bindingCtx->carrierLinkHostRaw)
            sourceCtx = bindingCtx;

        if (!sourceCtx)
        {
            if (s_link_host_seed_fail_logs < 6)
            {
                Log("SeedInjectedSensorCarrierLinkBase: no source ctx global=%p globalMoving=%p globalHost=%p binding=%p bindingMoving=%p bindingHost=%p sensorLink=%p",
                    (void*)ctx,
                    ctx ? ctx->movingObjectRaw : nullptr,
                    ctx ? ctx->carrierLinkHostRaw : nullptr,
                    (void*)bindingCtx,
                    bindingCtx ? bindingCtx->movingObjectRaw : nullptr,
                    bindingCtx ? bindingCtx->carrierLinkHostRaw : nullptr,
                    *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20));
                ++s_link_host_seed_fail_logs;
            }
            return false;
        }

        // IDA shows ccCockpitContext::init_unit() copies a LinkBase object from
        // a temporary into context+0x18, and avPlatform::set_carrier() copies an
        // equivalent LinkBase into sensor+0x10. Recreate that object directly.
        std::memcpy(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x10,
                    sourceCtx->carrierLinkBaseStorage,
                    sizeof(sourceCtx->carrierLinkBaseStorage));

        if (!s_link_host_seed_logged)
        {
            Log("SeedInjectedSensorCarrierLinkBase: sourceCtx=%p host=%p sensorLink=%p",
                (void*)sourceCtx,
                sourceCtx->carrierLinkHostRaw,
                *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20));
            s_link_host_seed_logged = true;
        }
        return true;
    }

    static bool GroundStabilizeIRADS()
    {
        if (!g_avTVSensor_instance)
            return false;

        SeedInjectedSensorPose(
            s_last_aircraft_pos[0], s_last_aircraft_pos[1], s_last_aircraft_pos[2],
            s_last_aircraft_quat[0], s_last_aircraft_quat[1],
            s_last_aircraft_quat[2], s_last_aircraft_quat[3]);

        if (g_avTVSensor_stabilize_on_ground)
        {
            const bool hasCarrierLinkBase = SeedInjectedSensorCarrierLinkBase();
            if (s_designation_diag_logs < 8)
            {
                Log("GroundStabilizeIRADS: stabilizeOnGround(useCarrier=%d) sensorLink=%p",
                    hasCarrierLinkBase ? 1 : 0,
                    *reinterpret_cast<void**>(reinterpret_cast<char*>(g_avTVSensor_instance) + 0x20));
            }
            g_avTVSensor_stabilize_on_ground(g_avTVSensor_instance, hasCarrierLinkBase ? 1 : 0);
            return true;
        }

        if (g_avTVSensor_search)
        {
            if (s_designation_diag_logs < 8)
                Log("GroundStabilizeIRADS: search fallback");
            g_avTVSensor_search(g_avTVSensor_instance);
            return true;
        }

        if (s_designation_diag_logs < 8)
            Log("GroundStabilizeIRADS: no native designation path available");
        return false;
    }

    static void ClearIRADSLock()
    {
        s_irads_lock_valid = false;
        s_irads_designating = false;
        s_irads_lock_world[0] = s_irads_lock_world[1] = s_irads_lock_world[2] = 0.0;
        s_irads_lock_polar[0] = s_irads_lock_polar[1] = s_irads_lock_polar[2] = 0.0;
    }

    static void SetExternalDesignation(bool enabled)
    {
        if (g_avTVSensor_instance && g_avTVSensor_set_external_designation)
        {
            g_avTVSensor_set_external_designation(g_avTVSensor_instance, enabled ? 1 : 0, 0);
        }
    }

    // ── Logging ───────────────────────────────────────────────────────────────

    static void OpenLog()
    {
        if (s_log || !g_hDLL) return;
        char dllPath[MAX_PATH] = {};
        if (GetModuleFileNameA(g_hDLL, dllPath, MAX_PATH))
        {
            char* lastSlash = strrchr(dllPath, '\\');
            if (lastSlash) *(lastSlash + 1) = '\0';
            std::string path = std::string(dllPath) + "F117_IRADS.log";
            s_log = fopen(path.c_str(), "a");
        }
        if (s_log)
        {
            fprintf(s_log, "\n--- F117_IRADS log opened ---\n");
            fprintf(s_log, "[F117-IRADS] BUILD %s\n", kIRADSBuildMarker);
            fflush(s_log);
        }
    }

    void Log_(const char* msg)
    {
        OpenLog();
        if (s_log) { fprintf(s_log, "[F117-IRADS] %s\n", msg); fflush(s_log); }
        OutputDebugStringA("[F117-IRADS] ");
        OutputDebugStringA(msg);
        OutputDebugStringA("\n");
    }

    static void Log(const char* fmt, ...)
    {
        char buf[512];
        va_list va;
        va_start(va, fmt);
        vsnprintf(buf, sizeof(buf), fmt, va);
        va_end(va);
        Log_(buf);
    }

    // ── Vtable[7] hook ────────────────────────────────────────────────────────
    // avDevice::start calls vtable[7](this) then reads this+0x58 as a pointer.
    // avTVSensor ctor writes double 1.0 to +0x58; vtable[7] needs that value
    // for camera/render-target setup, so we cannot zero it before the call.
    // We wrap vtable[7]: call the real function, then zero +0x50 and +0x58.

    static uintptr_t* s_private_vtable  = nullptr;
    static uintptr_t  s_orig_vtable7_fn = 0;

    static void __fastcall Vtable7Wrapper(void* self)
    {
        // Call the real avTVSensor virtual with all ctor camera params intact.
        reinterpret_cast<void(__fastcall*)(void*)>(s_orig_vtable7_fn)(self);

        // After it returns, zero both "double-as-pointer" fields so that
        // avDevice::start's subsequent checks see NULL and skip crash paths.
        AvTVSensorLayout* sensor = AsSensor(self);
        sensor->luaStateAlias = 0;
        sensor->subsystemAlias = 0;
        Log_("Vtable7Wrapper: real vtable[7] done, +0x50 and +0x58 zeroed");
    }

    static bool InstallVtable7Hook(void* sensor)
    {
        static constexpr size_t VTABLE_ENTRIES = 128;  // avTVSensor has many virtuals

        uintptr_t* origVtable = *reinterpret_cast<uintptr_t**>(sensor);
        Log("InstallVtable7Hook: origVtable=%p  slot7=%p",
            (void*)origVtable, (void*)origVtable[VTABLE_SLOT_LOAD_CFG]);

        s_private_vtable = static_cast<uintptr_t*>(
            malloc(VTABLE_ENTRIES * sizeof(uintptr_t)));
        if (!s_private_vtable) { Log_("InstallVtable7Hook: malloc failed"); return false; }

        memcpy(s_private_vtable, origVtable, VTABLE_ENTRIES * sizeof(uintptr_t));
        s_orig_vtable7_fn                       = origVtable[VTABLE_SLOT_LOAD_CFG];
        s_private_vtable[VTABLE_SLOT_LOAD_CFG]  = reinterpret_cast<uintptr_t>(&Vtable7Wrapper);

        *reinterpret_cast<uintptr_t**>(sensor) = s_private_vtable;
        Log("InstallVtable7Hook: slot7 → Vtable7Wrapper, orig=%p", (void*)s_orig_vtable7_fn);
        return true;
    }

    // ── Fix 3: Set IRADS camera position ─────────────────────────────────────
    // avTVSensor::initialize() calls avPlatform::setPlatformInitialLocalPosition via
    // vtable[4] with coordinates read from the model file. Our injected sensor
    // has no model entry, so initialize() reads zeros and places the camera at
    // (4.0, 0, 0) in model-local space. We override this with the real F-117A
    // DLIR belly mount position after initialize() has run.
    //
    // The position matrix is a row-major 4×4 double (wPosition3<N>):
    //   [0..3]  = right-axis row    (body X)
    //   [4..7]  = up-axis row       (body Y)
    //   [8..11] = forward-axis row  (body Z)
    //   [12..15]= translation row   (position, last element = 1.0)
    //
    // Identity orientation (sensor points forward along body X):
    //   right   = (1, 0, 0)
    //   up      = (0, 1, 0)
    //   forward = (0, 0, 1)
    //   pos     = (IRADS_POS_X, IRADS_POS_Y, IRADS_POS_Z, 1.0)

    static void SetCameraPosition(void* sensor)
    {
        double mat[16] = {
            1.0, 0.0, 0.0, 0.0,   // right axis (body X)
            0.0, 1.0, 0.0, 0.0,   // up axis    (body Y)
            0.0, 0.0, 1.0, 0.0,   // fwd axis   (body Z)
            IRADS_POS_X, IRADS_POS_Y, IRADS_POS_Z, 1.0
        };

        using SetPosFn = void(__fastcall*)(void*, double*);
        uintptr_t* vtbl = *reinterpret_cast<uintptr_t**>(sensor);
        auto fn = reinterpret_cast<SetPosFn>(vtbl[VTABLE_SLOT_SET_POS]);
        fn(sensor, mat);

        Log("SetCameraPosition: vtable[4] called, pos=(%.2f, %.2f, %.2f)",
            IRADS_POS_X, IRADS_POS_Y, IRADS_POS_Z);
    }

    static void BuildWorldPoseFromAircraft(double out_mat[16],
        double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w)
    {
        const double norm = std::sqrt(
            quat_x * quat_x + quat_y * quat_y + quat_z * quat_z + quat_w * quat_w);
        const double qx = (norm > 1.0e-9) ? (quat_x / norm) : 0.0;
        const double qy = (norm > 1.0e-9) ? (quat_y / norm) : 0.0;
        const double qz = (norm > 1.0e-9) ? (quat_z / norm) : 0.0;
        const double qw = (norm > 1.0e-9) ? (quat_w / norm) : 1.0;
        const double xx = qx * qx;
        const double yy = qy * qy;
        const double zz = qz * qz;
        const double xy = qx * qy;
        const double xz = qx * qz;
        const double yz = qy * qz;
        const double wx = qw * qx;
        const double wy = qw * qy;
        const double wz = qw * qz;

        out_mat[0] = 1.0 - 2.0 * (yy + zz);
        out_mat[1] = 2.0 * (xy + wz);
        out_mat[2] = 2.0 * (xz - wy);
        out_mat[3] = 0.0;

        out_mat[4] = 2.0 * (xy - wz);
        out_mat[5] = 1.0 - 2.0 * (xx + zz);
        out_mat[6] = 2.0 * (yz + wx);
        out_mat[7] = 0.0;

        out_mat[8] = 2.0 * (xz + wy);
        out_mat[9] = 2.0 * (yz - wx);
        out_mat[10] = 1.0 - 2.0 * (xx + yy);
        out_mat[11] = 0.0;

        out_mat[12] = aircraft_x
                    + IRADS_POS_X * out_mat[0]
                    + IRADS_POS_Y * out_mat[4]
                    + IRADS_POS_Z * out_mat[8];
        out_mat[13] = aircraft_y
                    + IRADS_POS_X * out_mat[1]
                    + IRADS_POS_Y * out_mat[5]
                    + IRADS_POS_Z * out_mat[9];
        out_mat[14] = aircraft_z
                    + IRADS_POS_X * out_mat[2]
                    + IRADS_POS_Y * out_mat[6]
                    + IRADS_POS_Z * out_mat[10];
        out_mat[15] = 1.0;
    }

    static void SeedInjectedSensorPose(double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w)
    {
        if (!g_avTVSensor_instance)
            return;

        AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);
        AvPlatformLayout* root = AsPlatform(sensor->platformRoot ? sensor->platformRoot : g_avTVSensor_instance);
        double world_pose[16] = {};

        BuildWorldPoseFromAircraft(world_pose,
            aircraft_x, aircraft_y, aircraft_z,
            quat_x, quat_y, quat_z, quat_w);

        std::memcpy(sensor->platformWorldPose.m, world_pose, sizeof(world_pose));
        std::memcpy(sensor->platformInitialWorldPose.m, world_pose, sizeof(world_pose));
        if (root)
        {
            std::memcpy(root->platformWorldPose.m, world_pose, sizeof(world_pose));
            std::memcpy(root->platformInitialWorldPose.m, world_pose, sizeof(world_pose));
        }

        s_last_aircraft_pos[0] = aircraft_x;
        s_last_aircraft_pos[1] = aircraft_y;
        s_last_aircraft_pos[2] = aircraft_z;
        s_last_aircraft_quat[0] = quat_x;
        s_last_aircraft_quat[1] = quat_y;
        s_last_aircraft_quat[2] = quat_z;
        s_last_aircraft_quat[3] = quat_w;

        if (!s_pose_seed_logged)
        {
            Log("SeedInjectedSensorPose: aircraft=(%.2f, %.2f, %.2f) sensor=(%.2f, %.2f, %.2f)",
                aircraft_x, aircraft_y, aircraft_z,
                world_pose[12], world_pose[13], world_pose[14]);
            s_pose_seed_logged = true;
        }
    }

    // ── Initialize ────────────────────────────────────────────────────────────

    bool Initialize()
    {
        if (s_initialized)
            return g_avTVSensor_ctor && g_devices_keeper_add && g_contexts_ptr_addr &&
                   g_avDevice_start && g_avTVSensor_update && g_avTVSensor_update_frame &&
                   g_avTVSensor_zoom_in &&
                   g_avTVSensor_zoom_out;

        HMODULE hCB = GetModuleHandleA("CockpitBase.dll");
        Log("Initialize: build marker %s", kIRADSBuildMarker);
        if (!hCB) { Log_("Initialize: CockpitBase.dll not yet loaded — will retry"); return false; }

#define LOAD(var, name)         var = reinterpret_cast<decltype(var)>(GetProcAddress(hCB, name));         Log("Initialize: %-60s → %p", name, (void*)var);

        LOAD(g_avTVSensor_ctor,         "??0avTVSensor@cockpit@@QEAA@XZ");
        LOAD(g_avTVSensor_initialize,   "?initialize@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_initZoom,     "?initZoom@avTVSensor@cockpit@@MEAAXXZ");
        LOAD(g_devices_keeper_add,      "?add@devices_keeper@cockpit@@QEAAXPEAVavDevice@2@@Z");
        LOAD(g_devices_keeper_get,      "?get@devices_keeper@cockpit@@QEAAPEAVavDevice@2@E@Z");
        // SET_FLIR_TECHINIQE_DEFAULT is not exported — we write preffered_IR_effect directly via RVA.
        // g_setFlirDefault remains null.
        LOAD(g_avTVSensor_update,       "?update@avTVSensor@cockpit@@UEAAXN@Z");
        LOAD(g_avTVSensor_update_frame, "?update_frame@avTVSensor@cockpit@@MEAAXXZ");
        LOAD(g_avTVSensor_slew_left,    "?slew_left@avTVSensor@cockpit@@UEAAXN@Z");
        LOAD(g_avTVSensor_slew_right,   "?slew_right@avTVSensor@cockpit@@UEAAXN@Z");
        LOAD(g_avTVSensor_slew_up,      "?slew_up@avTVSensor@cockpit@@UEAAXN@Z");
        LOAD(g_avTVSensor_slew_down,    "?slew_down@avTVSensor@cockpit@@UEAAXN@Z");
        LOAD(g_avTVSensor_move_horizontal_abs, "?on_TV_SENSOR_move_horizontal_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
        LOAD(g_avTVSensor_move_vertical_abs,   "?on_TV_SENSOR_move_vertical_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
        LOAD(g_avTVSensor_slew_stop,    "?slew_stop@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_search,       "?search@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_uncage,       "?uncage@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_stabilize_on_ground, "?stabilizeOnGround@avTVSensor@cockpit@@UEAAX_N@Z");
        LOAD(g_avTVSensor_zoom_in,      "?zoom_in@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_zoom_out,     "?zoom_out@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_get_tracked_point, "?get_tracked_point@avTVSensor@cockpit@@MEBA?AVVec3d@osg@@XZ");
        LOAD(g_avTVSensor_get_polar_position, "?getPolarPosition@avTVSensor@cockpit@@UEBA?AVPolar@Math@@XZ");
        LOAD(g_avDevice_start,          "?start@avDevice@cockpit@@QEAAXI@Z");
        LOAD(g_avPlatform_ctor,         "??0avPlatform@cockpit@@QEAA@XZ");
        LOAD(g_avPlatform_set_carrier,  "?set_carrier@avPlatform@cockpit@@UEAAXPEAVMovingObject@@@Z");
        LOAD(g_avPlatform_update_pos,   "?update_platform_position@avPlatform@cockpit@@QEAAXXZ");
        LOAD(g_binding_context,         "?binding_context@cockpit@@YAPEAVccCockpitContext@1@XZ");
        g_contexts_ptr_addr = reinterpret_cast<void**>(
            GetProcAddress(hCB, "?contexts_ptr@ccCockpitContext@cockpit@@0PAPEAV12@A"));
        LOAD(g_avTVSensor_set_external_designation,
             "?set_external_designation@avTVSensor@cockpit@@UEAAX_N0@Z");
        Log("Initialize: contexts_ptr addr=%p  *value=%p  binding_context=%p",
            (void*)g_contexts_ptr_addr,
            g_contexts_ptr_addr ? *g_contexts_ptr_addr : nullptr,
            g_binding_context ? g_binding_context() : nullptr);
#undef LOAD

        s_initialized = true;

        bool ok = g_avTVSensor_ctor && g_devices_keeper_add && g_contexts_ptr_addr &&
                  g_avDevice_start && g_avTVSensor_update && g_avTVSensor_update_frame &&
                  g_avTVSensor_zoom_in &&
                  g_avTVSensor_zoom_out;
        Log(ok ? "Initialize: OK" : "Initialize: PARTIAL — some exports missing");

        if (!g_avTVSensor_initialize)
            Log_("Initialize: WARNING — avTVSensor::initialize not exported; "
                 "will attempt to rely on DCS ccCockpitContext::start() only");
        if (!g_avTVSensor_initZoom)
            Log_("Initialize: WARNING — avTVSensor::initZoom not exported; "
                 "will fall back to direct zoom reset");
        if (!g_avTVSensor_move_horizontal_abs || !g_avTVSensor_move_vertical_abs)
            Log_("Initialize: WARNING — avTVSensor absolute-axis exports missing; "
                 "will fall back to directional slew handlers");
        if (!g_avTVSensor_search)
            Log_("Initialize: WARNING — avTVSensor::search not exported; designation can only capture existing tracks");
        if (!g_avTVSensor_stabilize_on_ground)
            Log_("Initialize: WARNING — avTVSensor::stabilizeOnGround not exported; ground designation will fall back to search");

        return ok;
    }

    // ── Phase A ───────────────────────────────────────────────────────────────

    bool InjectIRADSSensor_Early()
    {
        Log_("InjectIRADSSensor_Early: called");
        if (s_added) { Log_("InjectIRADSSensor_Early: already done."); return true; }
        if (!Initialize()) return false;

        CcCockpitContextLayout* ctx = AsContext(*g_contexts_ptr_addr);
        Log("InjectIRADSSensor_Early: contexts_ptr value = %p", (void*)ctx);
        if (!ctx) { Log_("InjectIRADSSensor_Early: context NULL — too early, will retry in Late."); return false; }

        // ── Construct avTVSensor ──────────────────────────────────────────────
        if (!g_avTVSensor_instance)
        {
            void* mem = _aligned_malloc(AVTVSENSOR_ALLOC_SIZE, 16);
            if (!mem) { Log_("InjectIRADSSensor_Early: alloc failed."); return false; }
            memset(mem, 0, AVTVSENSOR_ALLOC_SIZE);

            void* result = g_avTVSensor_ctor(mem);
            if (!result) { Log_("InjectIRADSSensor_Early: ctor returned null."); _aligned_free(mem); return false; }

            // Crash fix A: zero lua_State* at +0x50.
            // ctor writes double -1.0 there; avDevice::start treats it as
            // lua_State* and crashes. Zero it before avDevice::start runs.
            // vtable[7] does NOT read +0x50, so this is safe to zero now.
            AsSensor(mem)->luaStateAlias = 0;

            // Crash fix B: install vtable[7] hook.
            // avDevice::start calls vtable[7](this) (needs +0x58 = 1.0 for
            // camera setup), then reads +0x58 as subsystem*. The hook lets
            // vtable[7] run intact, then zeros both +0x50 and +0x58 afterward.
            InstallVtable7Hook(mem);

            g_avTVSensor_instance = mem;
            Log("InjectIRADSSensor_Early: avTVSensor constructed @ %p", (void*)mem);
        }

        // Stamp the device slot byte used by devices_keeper::add().
        AsSensor(g_avTVSensor_instance)->deviceSlotIndex = IRADS_DEVICE_INDEX;

        // Add to devices_keeper at ccCockpitContext+0x30.
        // The keeper is what DCS's indicator system queries when looking up
        // the TV sensor for an MFD. This must happen before ccCockpitContext::start().
        void* keeper = ctx->devicesKeeperStorage;
        g_devices_keeper_add(keeper, g_avTVSensor_instance);

        if (g_devices_keeper_get)
        {
            void* stored = g_devices_keeper_get(keeper, IRADS_DEVICE_INDEX);
            Log("InjectIRADSSensor_Early: devices_keeper[%d] = %p  (expected %p)",
                (int)IRADS_DEVICE_INDEX, (void*)stored, g_avTVSensor_instance);
            if (stored != g_avTVSensor_instance)
                Log_("InjectIRADSSensor_Early: WARNING — keeper slot does not hold our sensor!");
        }

        s_added = true;
        Log_("InjectIRADSSensor_Early: done.");
        return true;
    }

    // ── Phase B ───────────────────────────────────────────────────────────────

    bool InjectIRADSSensor_Late()
    {
        Log_("InjectIRADSSensor_Late: called");
        if (!Initialize()) return false;

        // Fix 1: retry Early if DllMain ran before the cockpit context existed.
        if (!s_added)
        {
            Log_("InjectIRADSSensor_Late: Early not done — retrying now.");
            if (!InjectIRADSSensor_Early())
            {
                Log_("InjectIRADSSensor_Late: Early retry failed — cannot proceed.");
                return false;
            }
        }

        CcCockpitContextLayout* ctx = GetGlobalContext();
        CcCockpitContextLayout* bindingCtx = GetBindingContext();
        Log("InjectIRADSSensor_Late: global ctx=%p  binding ctx=%p",
            (void*)ctx, (void*)bindingCtx);
        if (!ctx && !bindingCtx) { Log_("InjectIRADSSensor_Late: no cockpit context."); return false; }
        if (!ctx)
            ctx = bindingCtx;

        // ── Verify keeper slot ────────────────────────────────────────────────
        if (g_devices_keeper_get && g_avTVSensor_instance)
        {
            void* keeper = ctx->devicesKeeperStorage;
            void* stored = g_devices_keeper_get(keeper, IRADS_DEVICE_INDEX);
            Log("InjectIRADSSensor_Late: devices_keeper[%d]=%p  our sensor=%p",
                (int)IRADS_DEVICE_INDEX, (void*)stored, g_avTVSensor_instance);
            if (stored != g_avTVSensor_instance)
                Log_("InjectIRADSSensor_Late: WARNING — slot was overwritten by DCS!");
        }

        if (!g_avTVSensor_instance) { Log_("InjectIRADSSensor_Late: no sensor instance!"); return false; }
        AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);

        // ── Fix 2: ensure initialize() has run ───────────────────────────────
        // byte at +608 is set to 1 exclusively by avTVSensor::initialize().
        // If DCS called ccCockpitContext::start() after we added the device,
        // initialize() ran automatically (via avDevice::start → vtable chain).
        // If not (Early was too late, or start() was already done), we call it.
        uint8_t activeFlag = sensor->activeFlag;
        Log("InjectIRADSSensor_Late: sensor+%zu (active flag) = %d",
            offsetof(AvTVSensorLayout, activeFlag), (int)activeFlag);

        if (!s_started && g_avDevice_start)
        {
            Log_("InjectIRADSSensor_Late: calling avDevice::start(0) for injected sensor.");
            g_avDevice_start(g_avTVSensor_instance, 0);
            s_started = true;

            // Carrier will be set lazily in UpdateIRADSSensorFrame once init_unit
            // has bound the aircraft MovingObject to the context (context+0x10).
            // avDevice::start leaks RDX=0 into set_carrier, and DCS fires vtable[7]
            // again after this function returns — both would clear any carrier we
            // set here. Defer to per-frame path instead.

            activeFlag = sensor->activeFlag;
            Log("InjectIRADSSensor_Late: after avDevice::start(0), active flag = %d",
                (int)activeFlag);
        }

        if (activeFlag == 0)
        {
            if (g_avTVSensor_initialize)
            {
                Log_("InjectIRADSSensor_Late: initialize() not yet run after start() — calling explicitly.");
                g_avTVSensor_initialize(g_avTVSensor_instance);
                activeFlag = sensor->activeFlag;
                Log("InjectIRADSSensor_Late: after explicit initialize(), active flag = %d",
                    (int)activeFlag);
                if (activeFlag == 0)
                    Log_("InjectIRADSSensor_Late: WARNING — initialize() ran but flag still 0 "
                         "(SearchTimer alloc may have failed inside initialize)");
            }
            else
            {
                Log_("InjectIRADSSensor_Late: initialize() export not found and start() did not activate the sensor — "
                     "sensor may not render. Check that devices_keeper::add ran before "
                     "ccCockpitContext::start().");
            }
        }
        else
        {
            Log_("InjectIRADSSensor_Late: sensor active after DCS/start path.");
        }

        // Un-cage the sensor.
        // initialize() always calls cage() which sets byte+0x3AA=1. Every slew function
        // (slew_left/right/up/down) checks this byte and returns immediately if set.
        // We clear it here so manual gimbal control is available from mission start.
        *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938) = 0;
        if (g_avTVSensor_uncage)
            g_avTVSensor_uncage(g_avTVSensor_instance);
        Log_("InjectIRADSSensor_Late: sensor uncaged (byte+0x3AA cleared)");

        // ── Fix 3: override camera mount position ────────────────────────────
        // initialize() reads getPlatformInitialLocalPosition() which returns zeros
        // for an unmodeled sensor station, then hardcodes X=4.0 and leaves Y/Z=0.
        // We override with the correct F-117A DLIR belly position. This is safe
        // to call even if initialize() already ran — it just overwrites vtable[4]'s
        // previous result.
        Log_("InjectIRADSSensor_Late: setting IRADS camera mount position.");
        SetCameraPosition(g_avTVSensor_instance);

        // ── Activate IR render pass ───────────────────────────────────────────
        // SET_FLIR_TECHINIQE_DEFAULT is NOT exported from CockpitBase.dll.
        // The underlying static cockpit::avTVSensor::preffered_IR_effect lives at
        // RVA 0x533CF0 (confirmed via GET_FLIR_TECHNIQUE getter decompilation).
        // Write to it directly. ForceIRADSIndicatorActive also forces screenCondition=11
        // per-frame, but setting the global static ensures the render pass is selected
        // before the first indicator draw call.
        {
            HMODULE hCB = GetModuleHandleA("CockpitBase.dll");
            if (hCB)
            {
                static constexpr uintptr_t kPrefferedIREffectRVA = 0x533CF0;
                int* p = reinterpret_cast<int*>(reinterpret_cast<uintptr_t>(hCB) + kPrefferedIREffectRVA);
                *p = IR_SCREEN_CONDITION_FLIR;
                Log("InjectIRADSSensor_Late: preffered_IR_effect@RVA=0x%X set to %d",
                    (unsigned)kPrefferedIREffectRVA, IR_SCREEN_CONDITION_FLIR);
            }
            else
            {
                Log_("InjectIRADSSensor_Late: WARNING — CockpitBase.dll not found for FLIR static write");
            }
        }

        // ── Fix 4: initialize zoom through avTVSensor's real setup path ──────
        // The recovered binary shows current zoom at +0x358 is motor-driven and
        // the preset list lives in a vector at +0x390.  Writing +0x358 directly
        // bypasses that state machine, so prefer initZoom() when available.
        {
            Log("InjectIRADSSensor_Late: camera params before fix: "
                "fov_base=%.4f zoom=%.4f aspect=%.4f  (displayed_fov=%.1f deg)",
                sensor->baseFov, sensor->currentZoom, sensor->aspectRatio,
                (sensor->currentZoom > 0.001 ? sensor->baseFov / sensor->currentZoom : 0.0) * 57.296);
            if (g_avTVSensor_initZoom)
            {
                g_avTVSensor_initZoom(g_avTVSensor_instance);
                Log("InjectIRADSSensor_Late: initZoom() ran → displayed FOV = %.1f°",
                    (sensor->currentZoom > 0.001 ? sensor->baseFov / sensor->currentZoom : 0.0) * 57.296);
            }
            else
            {
                sensor->currentZoom = 1.0;
                Log("InjectIRADSSensor_Late: initZoom missing, fallback zoom reset to 1.0 → displayed FOV = %.1f°",
                    sensor->baseFov * 57.296);
            }
        }

        // ── Diagnostic: dump indicator array ─────────────────────────────────
        // context+0xB0 = void** array start, context+0xB8 = void** array end.
        // Each element is a pointer to a ccIndicator object. We log the key
        // offsets that determine whether the DLIR camera feed will render and
        // keep the live indicator state pinned to the IRADS values:
        //   indicator+0x38  (+56)   = avDevice* controller
        //   indicator+0x3A0 (+928)  = screen_condition
        //   indicator+0x46C (+1132) = indicator_type byte
        //   indicator+0x470 (+1136) = render_target_id
        //   indicator+0x48C (+1164) = camera_condition gate
        {
            auto logContextIndicators = [&](const char* label, CcCockpitContextLayout* whichCtx)
            {
                if (!whichCtx)
                {
                    Log("InjectIRADSSensor_Late: %s ctx is NULL", label);
                    return;
                }

                void** arr_start = whichCtx->indicators.begin;
                void** arr_end   = whichCtx->indicators.end;
                void** child_start = whichCtx->childContexts.begin;
                void** child_end   = whichCtx->childContexts.end;
                Log("InjectIRADSSensor_Late: %s ctx=%p indicators [%p .. %p] count=%zu children=%zu",
                    label,
                    (void*)whichCtx,
                    (void*)arr_start,
                    (void*)arr_end,
                    (arr_start && arr_end && arr_end > arr_start) ? (size_t)(arr_end - arr_start) : 0u,
                    (child_start && child_end && child_end > child_start) ? (size_t)(child_end - child_start) : 0u);
            };

            logContextIndicators("global", ctx);
            if (bindingCtx != ctx)
                logContextIndicators("binding", bindingCtx);
        }

        Log_("InjectIRADSSensor_Late: done.");
        return true;
    }
    bool HandleIRADSCommand(int command, float value)
    {
        if (!g_avTVSensor_instance) return false;

        const float axisDeadzone = 0.5f;
        const bool active = value > axisDeadzone || value < -axisDeadzone;
        const float axisMagnitude = (value < 0.0f) ? -value : value;

        auto dispatchHorizontal = [&]() -> bool
        {
            if (s_irads_horizontal_axis < -axisDeadzone && g_avTVSensor_slew_left)
            {
                g_avTVSensor_slew_left(g_avTVSensor_instance, -static_cast<double>(s_irads_horizontal_axis));
                return true;
            }
            if (s_irads_horizontal_axis > axisDeadzone && g_avTVSensor_slew_right)
            {
                g_avTVSensor_slew_right(g_avTVSensor_instance, static_cast<double>(s_irads_horizontal_axis));
                return true;
            }
            if (g_avTVSensor_slew_stop)
            {
                g_avTVSensor_slew_stop(g_avTVSensor_instance);
                return true;
            }
            if (g_avTVSensor_move_horizontal_abs)
                return g_avTVSensor_move_horizontal_abs(
                    g_avTVSensor_instance,
                    s_irads_horizontal_axis,
                    axisDeadzone,
                    static_cast<double>(s_irads_horizontal_axis));
            return false;
        };

        auto dispatchVertical = [&]() -> bool
        {
            if (s_irads_vertical_axis > axisDeadzone && g_avTVSensor_slew_up)
            {
                g_avTVSensor_slew_up(g_avTVSensor_instance, static_cast<double>(s_irads_vertical_axis));
                return true;
            }
            if (s_irads_vertical_axis < -axisDeadzone && g_avTVSensor_slew_down)
            {
                g_avTVSensor_slew_down(g_avTVSensor_instance, -static_cast<double>(s_irads_vertical_axis));
                return true;
            }
            if (g_avTVSensor_slew_stop)
            {
                g_avTVSensor_slew_stop(g_avTVSensor_instance);
                return true;
            }
            if (g_avTVSensor_move_vertical_abs)
                return g_avTVSensor_move_vertical_abs(
                    g_avTVSensor_instance,
                    s_irads_vertical_axis,
                    axisDeadzone,
                    static_cast<double>(s_irads_vertical_axis));
            return false;
        };

        switch (command)
        {
        case IRADSSlewLeft:
            if (active) s_irads_horizontal_axis = -axisMagnitude;
            else if (s_irads_horizontal_axis < 0.0f) s_irads_horizontal_axis = 0.0f;
            Log("HandleIRADSCommand: SlewLeft value=%.2f axis=%.2f caged=%d", value, s_irads_horizontal_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchHorizontal();
            return true;
        case IRADSSlewRight:
            if (active) s_irads_horizontal_axis = axisMagnitude;
            else if (s_irads_horizontal_axis > 0.0f) s_irads_horizontal_axis = 0.0f;
            Log("HandleIRADSCommand: SlewRight value=%.2f axis=%.2f caged=%d", value, s_irads_horizontal_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchHorizontal();
            return true;
        case IRADSSlewUp:
            if (active) s_irads_vertical_axis = axisMagnitude;
            else if (s_irads_vertical_axis > 0.0f) s_irads_vertical_axis = 0.0f;
            Log("HandleIRADSCommand: SlewUp value=%.2f axis=%.2f caged=%d", value, s_irads_vertical_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchVertical();
            return true;
        case IRADSSlewDown:
            if (active) s_irads_vertical_axis = -axisMagnitude;
            else if (s_irads_vertical_axis < 0.0f) s_irads_vertical_axis = 0.0f;
            Log("HandleIRADSCommand: SlewDown value=%.2f axis=%.2f caged=%d", value, s_irads_vertical_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchVertical();
            return true;
        case IRADSSlewStop:
            s_irads_horizontal_axis = 0.0f;
            s_irads_vertical_axis = 0.0f;
            if (g_avTVSensor_slew_stop)
                g_avTVSensor_slew_stop(g_avTVSensor_instance);
            if (active)
            {
                s_irads_designating = true;
                // Native set_external_designation(true) enters object search.
                // Ground designation needs stabilizeOnGround(), which writes the
                // current LOS/terrain intersection to cachedTrackedPoint at +0x328.
                GroundStabilizeIRADS();
                if (TryCaptureIRADSLock())
                {
                    s_irads_designating = false;
                    Log("HandleIRADSCommand: designation captured immediately world=(%.2f, %.2f, %.2f) polar=(%.2f, %.2f, %.2f)",
                        s_irads_lock_world[0], s_irads_lock_world[1], s_irads_lock_world[2],
                        s_irads_lock_polar[0], s_irads_lock_polar[1], s_irads_lock_polar[2]);
                }
                else
                {
                    Log_("HandleIRADSCommand: designation armed — lock pending stabilisation");
                }
            }
            return true;
        case TVSensorZoomIn:
        case IRADSZoomIn:
            // value_down is not set in the keyboard binding so value=0 → active is always false.
            // Zoom is a momentary press, not an axis, so just call unconditionally.
            Log_("HandleIRADSCommand: ZoomIn");
            if (g_avTVSensor_zoom_in) g_avTVSensor_zoom_in(g_avTVSensor_instance);
            return true;
        case TVSensorZoomOut:
        case IRADSZoomOut:
            Log_("HandleIRADSCommand: ZoomOut");
            if (g_avTVSensor_zoom_out) g_avTVSensor_zoom_out(g_avTVSensor_instance);
            return true;
        case IRADSDesignationClear:
            ClearIRADSLock();
            SetExternalDesignation(false);
            return true;
        default:
            return false;
        }
    }

    void UpdateIRADSSensorFrame(double dt,
        double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w)
    {
        if (!g_avTVSensor_instance || !g_avTVSensor_update) return;
        if (AsSensor(g_avTVSensor_instance)->activeFlag == 0) return;

        SeedInjectedSensorPose(
            aircraft_x, aircraft_y, aircraft_z,
            quat_x, quat_y, quat_z, quat_w);
        // Do not call avPlatform::set_carrier for the injected sensor. DCS crash
        // logs showed avTVSensor::update -> update_platform_position ->
        // get_carrier_position dereferencing an unsafe carrier chain after this
        // link was installed. Keeping platformRoot self-referential preserves the
        // native update path without entering that crash-prone MovingObject chain.
        if (!s_carrier_link_disabled_logged)
        {
            Log_("UpdateIRADSSensorFrame: native carrier link disabled for injected sensor");
            s_carrier_link_disabled_logged = true;
        }

        if (false && !s_carrier_set && g_avPlatform_set_carrier && g_contexts_ptr_addr && *g_contexts_ptr_addr)
        {
            CcCockpitContextLayout* ctx = AsContext(*g_contexts_ptr_addr);
            void* movingObject = ctx->movingObjectRaw;
            if (movingObject)
            {
                g_avPlatform_set_carrier(g_avTVSensor_instance, movingObject);
                s_carrier_set = true;
                Log("UpdateIRADSSensorFrame: carrier set → MovingObject=%p", movingObject);
            }
        }

        // Keep the sensor uncaged every frame.
        // initialize() calls cage() which sets byte+0x3AA=1, blocking all slew.
        // Some DCS mode transitions may also re-cage — clearing it each frame
        // ensures slew is always available during IRADS operation.
        *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938) = 0;
        if (g_avTVSensor_uncage)
            g_avTVSensor_uncage(g_avTVSensor_instance);

        // Tick the sensor: avTVSensor::update drives the AZ/EL additive motors
        // (from slew commands), then avPlatform::update_platform_position reads
        // those motor angles via update_angles()/elevation() and applies them to
        // the world pose at sensor+0xA8. No direct pose write needed.
        g_avTVSensor_update(g_avTVSensor_instance, dt);
        if (g_avTVSensor_update_frame)
            g_avTVSensor_update_frame(g_avTVSensor_instance);

        if (s_irads_designating)
        {
            if (TryCaptureIRADSLock())
                s_irads_designating = false;
        }
    }

    bool CaptureIRADSLock()
    {
        s_irads_designating = true;
        if (s_designation_diag_logs < 8)
            Log("CaptureIRADSLock: begin build=%s", kIRADSBuildMarker);
        GroundStabilizeIRADS();
        if (TryCaptureIRADSLock())
        {
            s_irads_designating = false;
            return true;
        }
        if (s_designation_diag_logs < 8)
            Log("CaptureIRADSLock: no tracked point after designation");
        return false;
    }

    bool HasIRADSLock()
    {
        return s_irads_lock_valid;
    }

    bool GetIRADSLockWorldPoint(double out_point[3])
    {
        if (!s_irads_lock_valid || !out_point)
            return false;
        out_point[0] = s_irads_lock_world[0];
        out_point[1] = s_irads_lock_world[1];
        out_point[2] = s_irads_lock_world[2];
        return true;
    }

    bool GetIRADSLockPolar(double out_polar[3])
    {
        if (!s_irads_lock_valid || !out_polar)
            return false;
        out_polar[0] = s_irads_lock_polar[0];
        out_polar[1] = s_irads_lock_polar[1];
        out_polar[2] = s_irads_lock_polar[2];
        return true;
    }


    // ── Phase C: per-frame IRADS indicator keep-alive ─────────────────────────
    // Called every frame from ed_fm_simulate. On the first call after indicators
    // are populated (post device_init.lua), it locates the ccIndicator whose
    // controller pointer matches our injected avTVSensor. Every subsequent frame
    // it forces screenCondition = FLIR (11) and cameraCondition = ON (1) to
    // prevent DCS from reverting the indicator to a non-camera state.
    //
    // Offsets used (from ccDrawable/ccIndicator decompilation):
    //   indicator + 0x38  = avDevice* controller
    //   indicator + 0x3A0 = screenCondition (TVScreenCondition)
    //   indicator + 0x48C = cameraCondition (must be non-zero for camera feed)

    void ForceIRADSIndicatorActive()
    {
        static void* s_our_ind = nullptr;
        static bool s_indicator_search_logged = false;

        if (!g_avTVSensor_instance)
            return;

        if (!s_our_ind)
        {
            int foundDepth = -1;
            CcCockpitContextLayout* globalCtx = GetGlobalContext();
            CcCockpitContextLayout* bindingCtx = GetBindingContext();

            if (FindIndicatorInContextTree(globalCtx, &s_our_ind, &foundDepth, 0, false))
            {
                Log("ForceIRADSIndicatorActive: found indicator by controller @ %p depth=%d via global ctx",
                    s_our_ind, foundDepth);
            }
            else if (bindingCtx != globalCtx &&
                     FindIndicatorInContextTree(bindingCtx, &s_our_ind, &foundDepth, 0, false))
            {
                Log("ForceIRADSIndicatorActive: found indicator by controller @ %p depth=%d via binding ctx",
                    s_our_ind, foundDepth);
            }
            else if (FindIndicatorInContextTree(globalCtx, &s_our_ind, &foundDepth, 0, true))
            {
                Log("ForceIRADSIndicatorActive: force-linked DLIR camera indicator @ %p depth=%d via global ctx",
                    s_our_ind, foundDepth);
            }
            else if (bindingCtx != globalCtx &&
                     FindIndicatorInContextTree(bindingCtx, &s_our_ind, &foundDepth, 0, true))
            {
                Log("ForceIRADSIndicatorActive: force-linked DLIR camera indicator @ %p depth=%d via binding ctx",
                    s_our_ind, foundDepth);
            }
            else if (!s_indicator_search_logged)
            {
                Log("ForceIRADSIndicatorActive: no indicator found yet (global=%p binding=%p)",
                    (void*)globalCtx, (void*)bindingCtx);
                s_indicator_search_logged = true;
            }

        }

        if (s_our_ind)
        {
            CcIndicatorLayout* indicatorView = AsIndicator(s_our_ind);
            indicatorView->screenCondition = static_cast<uint32_t>(IR_SCREEN_CONDITION_FLIR);
            indicatorView->cameraCondition = static_cast<uint32_t>(IR_CAMERA_CONDITION_ON);
        }
    }

} // namespace CockpitInterop
