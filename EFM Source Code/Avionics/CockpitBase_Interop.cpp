// Standalone avTVSensor and explicit ccIndicator camera bridge.
// avTVSensor is NOT an avDevice: never register/start it through devices_keeper.
#include "stdafx.h"
#include "CockpitBase_Interop.h"
#include "CockpitBase_Layouts.h"
#include "../Inputs.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <limits>
#include <malloc.h>
#include <string>

namespace CockpitInterop
{
    static constexpr const char* kIRADSBuildMarker = "2026-11-02-thrust-table-v25";

    // â”€â”€ Globals â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
    // Keep centered analog input from cancelling a held keyboard direction.
    static float s_irads_analog_horizontal = 0.0f, s_irads_analog_vertical = 0.0f;
    static float s_irads_keyboard_horizontal = 0.0f, s_irads_keyboard_vertical = 0.0f;
    static int s_irads_zoom_level = 1;
    static bool s_irads_power = true;
    static bool  s_irads_lock_valid      = false;
    static bool s_irads_ray_valid=false, s_irads_terrain_pending=false;
    static double s_irads_render_ray[6]={}, s_irads_request_ray[6]={};
    // Seconds since the IRADS camera last rendered; its ray is stale when the
    // display is not drawing the sensor image.
    static double s_irads_ray_age=1e9;
    constexpr double kIRADSRayMaxAge=0.5;
    static unsigned s_irads_terrain_sequence=0;
    static double s_irads_terrain_wait=0;

    static bool  s_irads_designating     = false;
    static double s_last_aircraft_pos[3]  = {0.0, 0.0, 0.0};
    static double s_last_aircraft_quat[4] = {0.0, 0.0, 0.0, 1.0};
    static double s_irads_lock_world[3]   = {0.0, 0.0, 0.0};
    static double s_irads_lock_polar[3]   = {0.0, 0.0, 0.0};
    // When designation could not run ground stabilisation (which updates the cached
    // point synchronously), the cached point may still hold the previous designation.
    // Reject a capture until the cached point differs from this snapshot.
    static bool   s_require_fresh_point   = false;
    static double s_stale_point[3]        = {0.0, 0.0, 0.0};
    static void*  s_our_ind               = nullptr;   // cached DLIR indicator (owned by DCS)

    // â”€â”€ Sensor state accessors â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Native call positions used by the sensor bridge.
    static constexpr size_t VTABLE_SLOT_SET_POS  = 4;     // sets the sensor's local mount position

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
                if (v->indicatorType == 0 && static_cast<int32_t>(v->renderTargetId) == 0)
                {
                    *out_indicator = ind;
                    if (out_depth) *out_depth = depth;
                    return true;
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
        // designated/stabilised ground point. Both are valid for CCRP.
        // We use the native function for mode 3, and read the cached point for other modes.
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

            // Fallback: compute the point from the track data transform
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

        // Non-tracking modes: read the cached designated/stabilised point.
        // This is populated by the external designation stabilisation loop.
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

        // The platform's carrier position comes from its link host pointer. For the injected
        // sensor we skip set_carrier and copy the live cockpit host pointer directly.
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

        // The sensor needs a carrier link equivalent to the cockpit context's.
        // Recreate that object directly.
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
        if (!g_avTVSensor_instance || !s_irads_power || !s_irads_ray_valid) return false;
        double norm=0;
        for(int i=0;i<6;++i) if(!std::isfinite(s_irads_render_ray[i])) return false;
        for(int i=3;i<6;++i) norm+=s_irads_render_ray[i]*s_irads_render_ray[i];
        if(!std::isfinite(norm) || norm<0.5) return false;
        norm=std::sqrt(norm);
        for(int i=0;i<3;++i) {
            s_irads_request_ray[i]=s_irads_render_ray[i];
            s_irads_request_ray[i+3]=s_irads_render_ray[i+3]/norm;
        }
        s_irads_lock_valid=false;
        s_irads_terrain_pending=true;
        s_irads_terrain_wait=0;
        if(++s_irads_terrain_sequence>10000000) s_irads_terrain_sequence=1;
        Log("IRADS terrain request: id=%u eye=(%.3f,%.3f,%.3f) direction=(%.8f,%.8f,%.8f)",
            s_irads_terrain_sequence,s_irads_request_ray[0],s_irads_request_ray[1],s_irads_request_ray[2],
            s_irads_request_ray[3],s_irads_request_ray[4],s_irads_request_ray[5]);
        return true;
    }

    static void ClearIRADSLock()
    {
        s_irads_lock_valid = false;
        s_irads_terrain_pending = false;
        s_irads_designating = false;
        s_irads_lock_world[0] = s_irads_lock_world[1] = s_irads_lock_world[2] = 0.0;
        s_irads_lock_polar[0] = s_irads_lock_polar[1] = s_irads_lock_polar[2] = 0.0;
        s_require_fresh_point = false;
    }

    static void SetExternalDesignation(bool enabled)
    {
        if (g_avTVSensor_instance && g_avTVSensor_set_external_designation)
        {
            g_avTVSensor_set_external_designation(g_avTVSensor_instance, enabled ? 1 : 0, 0);
        }
    }

    // â”€â”€ Logging â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    // Keeps the log bounded on disk. When the DLL loads, a log over kLogRotateBytes becomes
    // F117_IRADS.old.log, replacing the previous one. One load stops writing to the file after
    // kLogLoadBytes, so a runaway message can't fill the disk or push out the older log.
    static constexpr unsigned long long kLogRotateBytes = 10ULL * 1024 * 1024;
    static constexpr unsigned long long kLogLoadBytes = 20ULL * 1024 * 1024;
    static unsigned long long s_log_load_bytes = 0;

    static void RotateLogIfLarge(const std::string& directory, const std::string& path)
    {
        WIN32_FILE_ATTRIBUTE_DATA info = {};
        if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &info)) return;
        const unsigned long long size =
            (static_cast<unsigned long long>(info.nFileSizeHigh) << 32) | info.nFileSizeLow;
        if (size <= kLogRotateBytes) return;
        // On failure the log keeps appending to the same file, and the next load tries again.
        MoveFileExA(path.c_str(), (directory + "F117_IRADS.old.log").c_str(), MOVEFILE_REPLACE_EXISTING);
    }

    static void OpenLog()
    {
        if (s_log || !g_hDLL) return;
        char dllPath[MAX_PATH] = {};
        if (GetModuleFileNameA(g_hDLL, dllPath, MAX_PATH))
        {
            char* lastSlash = strrchr(dllPath, '\\');
            if (lastSlash) *(lastSlash + 1) = '\0';
            std::string path = std::string(dllPath) + "F117_IRADS.log";
            RotateLogIfLarge(dllPath, path);
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
        if (s_log && s_log_load_bytes < kLogLoadBytes)
        {
            const int written = fprintf(s_log, "[F117-IRADS] %s\n", msg);
            if (written > 0) s_log_load_bytes += static_cast<unsigned long long>(written);
            if (s_log_load_bytes >= kLogLoadBytes)
                fprintf(s_log, "[F117-IRADS] Log limit reached; file output stops until the DLL next loads\n");
            fflush(s_log);
        }
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

    // Set the standalone sensor mount through avPlatform.
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

    // â”€â”€ Initialize â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    // g_hDLL is our EFM module (used for the log path), not CockpitBase.
    static FARPROC ResolveCockpitBaseExport(const char* name)
    {
        const HMODULE module = GetModuleHandleA("CockpitBase.dll");
        return module ? GetProcAddress(module, name) : nullptr;
    }

    bool Initialize()
    {
        if (s_initialized)
            return g_avTVSensor_ctor && g_avTVSensor_initialize && g_contexts_ptr_addr &&
                   g_avTVSensor_update && g_avTVSensor_update_frame &&
                   g_avTVSensor_zoom_in &&
                   g_avTVSensor_zoom_out;

        HMODULE hCB = GetModuleHandleA("CockpitBase.dll");
        Log("Initialize: build marker %s", kIRADSBuildMarker);
        if (!hCB) { Log_("Initialize: CockpitBase.dll not yet loaded â€” will retry"); return false; }

#define LOAD(var, name)         var = reinterpret_cast<decltype(var)>(GetProcAddress(hCB, name));         Log("Initialize: %-60s â†’ %p", name, (void*)var);

        LOAD(g_avTVSensor_ctor,         "??0avTVSensor@cockpit@@QEAA@XZ");
        LOAD(g_avTVSensor_initialize,   "?initialize@avTVSensor@cockpit@@UEAAXXZ");
        LOAD(g_avTVSensor_initZoom,     "?initZoom@avTVSensor@cockpit@@MEAAXXZ");
        // preffered_IR_effect is resolved by export name in InjectIRADSSensor_Late.
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

        bool ok = g_avTVSensor_ctor && g_avTVSensor_initialize && g_contexts_ptr_addr &&
                  g_avTVSensor_update && g_avTVSensor_update_frame &&
                  g_avTVSensor_zoom_in &&
                  g_avTVSensor_zoom_out;
        Log(ok ? "Initialize: OK" : "Initialize: PARTIAL â€” some exports missing");

        if (!g_avTVSensor_initialize)
            Log_("Initialize: WARNING â€” avTVSensor::initialize not exported; "
                 "standalone sensor cannot initialize");
        if (!g_avTVSensor_initZoom)
            Log_("Initialize: WARNING â€” avTVSensor::initZoom not exported; "
                 "will fall back to direct zoom reset");
        if (!g_avTVSensor_move_horizontal_abs || !g_avTVSensor_move_vertical_abs)
            Log_("Initialize: WARNING â€” avTVSensor absolute-axis exports missing; "
                 "will fall back to directional slew handlers");
        if (!g_avTVSensor_search)
            Log_("Initialize: WARNING â€” avTVSensor::search not exported; designation can only capture existing tracks");
        if (!g_avTVSensor_stabilize_on_ground)
            Log_("Initialize: WARNING â€” avTVSensor::stabilizeOnGround not exported; ground designation will fall back to search");

        return ok;
    }

    // â”€â”€ Phase A â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    bool InjectIRADSSensor_Early()
    {
        Log_("InjectIRADSSensor_Early: called");
        if (s_added) { Log_("InjectIRADSSensor_Early: already done."); return true; }
        if (!Initialize()) return false;

        CcCockpitContextLayout* ctx = AsContext(*g_contexts_ptr_addr);
        Log("InjectIRADSSensor_Early: contexts_ptr value = %p", (void*)ctx);
        if (!ctx) { Log_("InjectIRADSSensor_Early: context NULL â€” too early, will retry in Late."); return false; }

        // â”€â”€ Construct avTVSensor â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        if (!g_avTVSensor_instance)
        {
            void* mem = _aligned_malloc(AVTVSENSOR_ALLOC_SIZE, 16);
            if (!mem) { Log_("InjectIRADSSensor_Early: alloc failed."); return false; }
            memset(mem, 0, AVTVSENSOR_ALLOC_SIZE);

            void* result = g_avTVSensor_ctor(mem);
            if (!result) { Log_("InjectIRADSSensor_Early: ctor returned null."); _aligned_free(mem); return false; }

            using SetPlatform = void (__fastcall*)(void*, void*);
            const auto setPlatform = reinterpret_cast<SetPlatform>(ResolveCockpitBaseExport(
                "?set_platform@avPlatform@cockpit@@UEAAXPEAV12@@Z"));
            if (!setPlatform) { Log_("IRADS: set_platform missing; initialization stopped"); return false; }
            setPlatform(mem, nullptr); // native null-parent path establishes root=self
            g_avTVSensor_instance = mem;
            Log("InjectIRADSSensor_Early: avTVSensor constructed @ %p", (void*)mem);
        }

        s_added = true;
        Log_("InjectIRADSSensor_Early: done.");
        return true;
    }

    // â”€â”€ Phase B â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    bool InjectIRADSSensor_Late()
    {
        Log_("InjectIRADSSensor_Late: called");
        if (!Initialize()) return false;

        // Fix 1: retry Early if DllMain ran before the cockpit context existed.
        if (!s_added)
        {
            Log_("InjectIRADSSensor_Late: Early not done â€” retrying now.");
            if (!InjectIRADSSensor_Early())
            {
                Log_("InjectIRADSSensor_Late: Early retry failed â€” cannot proceed.");
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

        if (!g_avTVSensor_instance) { Log_("InjectIRADSSensor_Late: no sensor instance!"); return false; }
        AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);

        // This is a standalone avBasicSensor; initialize it directly, never
        // through avDevice::start or the cockpit device keeper.
        uint8_t activeFlag = sensor->activeFlag;
        Log("InjectIRADSSensor_Late: sensor+%zu (active flag) = %d",
            offsetof(AvTVSensorLayout, activeFlag), (int)activeFlag);

        if (activeFlag == 0)
        {
            if (g_avTVSensor_initialize)
            {
                Log_("InjectIRADSSensor_Late: initializing standalone sensor.");
                g_avTVSensor_initialize(g_avTVSensor_instance);
                activeFlag = sensor->activeFlag;
                Log("InjectIRADSSensor_Late: after explicit initialize(), active flag = %d",
                    (int)activeFlag);
                if (activeFlag == 0)
                    Log_("InjectIRADSSensor_Late: WARNING â€” initialize() ran but flag still 0 "
                         "(SearchTimer alloc may have failed inside initialize)");
            }
            else
            {
                Log_("InjectIRADSSensor_Late: native initialize export missing; sensor unavailable");
            }
        }
        else
        {
            Log_("InjectIRADSSensor_Late: standalone sensor already initialized.");
        }

        // Un-cage the sensor.
        // initialize() leaves the sensor caged, and the slew functions do nothing
        // while it is caged.
        // We clear it here so manual gimbal control is available from mission start.
        *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938) = 0;
        if (g_avTVSensor_uncage)
            g_avTVSensor_uncage(g_avTVSensor_instance);
        Log_("InjectIRADSSensor_Late: sensor uncaged (byte+0x3AA cleared)");

        // â”€â”€ Fix 3: override camera mount position â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        // initialize() has no modelled position for this sensor station, so it uses
        // a default.
        // We override with the correct F-117A DLIR belly position. This is safe
        // to call even if initialize() already ran; it overwrites the previous
        // result.
        Log_("InjectIRADSSensor_Late: setting IRADS camera mount position.");
        SetCameraPosition(g_avTVSensor_instance);

        // â”€â”€ Activate IR render pass â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        // cockpit::avTVSensor::preffered_IR_effect is an exported static, so
        // resolve it by name so it keeps working across DCS updates.
        // ForceIRADSIndicatorActive also forces screenCondition=11 per-frame, but
        // setting the global static ensures the render pass is selected before
        // the first indicator draw call.
        {
            HMODULE hCB = GetModuleHandleA("CockpitBase.dll");
            int* p = hCB ? reinterpret_cast<int*>(
                GetProcAddress(hCB, "?preffered_IR_effect@avTVSensor@cockpit@@2HA")) : nullptr;
            if (p)
            {
                *p = IR_SCREEN_CONDITION_FLIR;
                Log("InjectIRADSSensor_Late: preffered_IR_effect@%p set to %d",
                    (void*)p, IR_SCREEN_CONDITION_FLIR);
            }
            else
            {
                Log_("InjectIRADSSensor_Late: WARNING â€” preffered_IR_effect export not found; skipping FLIR static write");
            }
        }

        // â”€â”€ Fix 4: initialize zoom through avTVSensor's real setup path â”€â”€â”€â”€â”€â”€
        // Zoom is motor-driven and has its own preset list. Initialize the native
        // zoom motor, then set both current and target for our 1.4/10/80 presets.
        {
            Log("InjectIRADSSensor_Late: camera params before fix: "
                "fov_base=%.4f zoom=%.4f aspect=%.4f  (displayed_fov=%.1f deg)",
                sensor->baseFov, sensor->currentZoom, sensor->aspectRatio,
                (sensor->currentZoom > 0.001 ? sensor->baseFov / sensor->currentZoom : 0.0) * 57.296);
            if (g_avTVSensor_initZoom)
            {
                g_avTVSensor_initZoom(g_avTVSensor_instance);
                Log("InjectIRADSSensor_Late: initZoom() ran â†’ displayed FOV = %.1fÂ°",
                    (sensor->currentZoom > 0.001 ? sensor->baseFov / sensor->currentZoom : 0.0) * 57.296);
            }
            else
            {
                sensor->currentZoom = 1.0;
                Log("InjectIRADSSensor_Late: initZoom missing, fallback zoom reset to 1.0 â†’ displayed FOV = %.1fÂ°",
                    sensor->baseFov * 57.296);
            }
        }

        // â”€â”€ Diagnostic: dump indicator array â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        // Walk the cockpit context's indicators. We log the indicator state that
        // decides whether the DLIR camera feed renders, and keep it pinned to the
        // IRADS values: controller, screen condition, indicator type, render target
        // and camera condition.
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

        sensor->currentZoom = 1.4;
        sensor->targetZoom = 1.4;
        Log_("InjectIRADSSensor_Late: done.");
        return true;
    }
    bool HandleIRADSCommand(int command, float value)
    {
        if ((command >= 10008 && command <= 10121) ||
            (command >= 2110 && command <= 2112) || (command >= 20110 && command <= 20112) || command == 87)
        {
            static unsigned int loggedInputs = 0;
            if (loggedInputs++ < 128)
                Log("IRADS input: command=%d value=%.4f sensor=%p", command, value, g_avTVSensor_instance);
        }
        if (command >= 10119 && command <= 10121 && std::isfinite(value) && std::abs(value) > 0.05f)
        {
            static unsigned int axisLogs = 0;
            if (axisLogs++ < 48) Log("IRADS routed axis: command=%d value=%.4f", command, value);
        }
        // Slew commands can arrive continuously, so log only the first ones of each load.
        static unsigned int slewLogs = 0;
        if (!g_avTVSensor_instance) return false;

        if (!std::isfinite(value)) return false;
        value = (value < -1.0f) ? -1.0f : ((value > 1.0f) ? 1.0f : value);
        // Match angular rate to displayed FOV: wide can traverse quickly,
        // narrow retains precision. Native velocity uses its zoom-table index,
        // which does not track our explicit 1.4/10/80 optical zoom values.
        auto slewRateScale = [&]() -> double
        {
            double zoom = AsSensor(g_avTVSensor_instance)->currentZoom;
            if (!std::isfinite(zoom)) zoom = 1.4;
            zoom = (zoom < 1.4) ? 1.4 : ((zoom > 80.0) ? 80.0 : zoom);
            return 60.0 * (1.4 / zoom);
        };
        const float axisDeadzone = 0.05f;
        const bool active = value > axisDeadzone || value < -axisDeadzone;
        const float axisMagnitude = (value < 0.0f) ? -value : value;

        if (!s_irads_power && command != 87) return false;

        const bool isSlew = command == IRADSSlewLeft || command == IRADSSlewRight ||
            command == IRADSSlewUp || command == IRADSSlewDown ||
            command == IRADSSlewHorizontalAxis || command == IRADSSlewVerticalAxis ||
            command == 2110 || command == 2111 || command == 20110 || command == 20111;
        if (isSlew && active && (s_irads_lock_valid || s_irads_terrain_pending)) ClearIRADSLock();

        auto applyHorizontal = [&]() -> bool
        {
            if (s_irads_horizontal_axis < -axisDeadzone && g_avTVSensor_slew_left)
            {
                g_avTVSensor_slew_left(g_avTVSensor_instance, -static_cast<double>(s_irads_horizontal_axis) * slewRateScale());
                return true;
            }
            if (s_irads_horizontal_axis > axisDeadzone && g_avTVSensor_slew_right)
            {
                g_avTVSensor_slew_right(g_avTVSensor_instance, static_cast<double>(s_irads_horizontal_axis) * slewRateScale());
                return true;
            }
            return false;
        };

        auto applyVertical = [&]() -> bool
        {
            if (s_irads_vertical_axis > axisDeadzone && g_avTVSensor_slew_up)
            {
                g_avTVSensor_slew_up(g_avTVSensor_instance, static_cast<double>(s_irads_vertical_axis) * slewRateScale());
                return true;
            }
            if (s_irads_vertical_axis < -axisDeadzone && g_avTVSensor_slew_down)
            {
                g_avTVSensor_slew_down(g_avTVSensor_instance, -static_cast<double>(s_irads_vertical_axis) * slewRateScale());
                return true;
            }
            return false;
        };

        // Stopping slew stops BOTH gimbal motors and
        // rebases slew state, so after stopping one axis re-apply the other.
        auto stopAndResume = [&](bool resumeVertical) -> bool
        {
            if (!g_avTVSensor_slew_stop)
                return false;
            g_avTVSensor_slew_stop(g_avTVSensor_instance);
            if (resumeVertical) applyVertical();
            else                applyHorizontal();
            return true;
        };

        auto dispatchHorizontal = [&]() -> bool
        {
            if (applyHorizontal())
                return true;
            if (stopAndResume(true))
                return true;
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
            if (applyVertical())
                return true;
            if (stopAndResume(false))
                return true;
            if (g_avTVSensor_move_vertical_abs)
                return g_avTVSensor_move_vertical_abs(
                    g_avTVSensor_instance,
                    s_irads_vertical_axis,
                    axisDeadzone,
                    static_cast<double>(s_irads_vertical_axis));
            return false;
        };

        auto setZoomLevel = [&](int level)
        {
            s_irads_zoom_level = level;
            auto* sensor = AsSensor(g_avTVSensor_instance);
            const double zoom[] = {1.4, 10.0, 80.0};
            sensor->currentZoom = zoom[level - 1];
            sensor->targetZoom = sensor->currentZoom;
            // Reapply a held direction immediately at the new optical rate.
            applyHorizontal();
            applyVertical();
            Log("IRADS optical zoom: level=%d zoom=%.2f fullFovDeg=%.3f", level,
                sensor->currentZoom, sensor->baseFov / sensor->currentZoom * 57.295779513);

        };

        switch (command)
        {
        case 87: // iCommandPlaneEOSOnOff; iCommandTVSensor is undefined in DCS input profiles
            s_irads_power = !s_irads_power;
            s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
            s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
            s_irads_horizontal_axis = s_irads_vertical_axis = 0;
            ClearIRADSLock();
            if (g_avTVSensor_slew_stop) g_avTVSensor_slew_stop(g_avTVSensor_instance);
            Log("IRADS power: %s", s_irads_power ? "ON" : "OFF");
            return true;
        case 20110: // legacy direct-axis IDs
        case 2110:
        case IRADSSlewHorizontalAxis:
            s_irads_analog_horizontal = active ? value : 0.0f;
            s_irads_horizontal_axis = s_irads_keyboard_horizontal != 0.0f
                ? s_irads_keyboard_horizontal : s_irads_analog_horizontal;
            dispatchHorizontal();
            return true;
        case 20111:
        case 2111:
        case IRADSSlewVerticalAxis:
            s_irads_analog_vertical = active ? value : 0.0f;
            s_irads_vertical_axis = s_irads_keyboard_vertical != 0.0f
                ? s_irads_keyboard_vertical : s_irads_analog_vertical;
            dispatchVertical();
            return true;
        case 20112:
        case 2112:
        case IRADSZoomAxis:
        {
            // Hysteresis avoids repeated zoom steps at the detent boundaries.
            int level = s_irads_zoom_level;
            if (value < -0.4f) level = 1;
            else if (value > 0.4f) level = 3;
            else if (value > -0.25f && value < 0.25f) level = 2;
            setZoomLevel(level);
            return true;
        }
        case IRADSSlewLeft:
            if (active) s_irads_keyboard_horizontal = -axisMagnitude;
            else if (s_irads_keyboard_horizontal < 0.0f) s_irads_keyboard_horizontal = 0.0f;
            s_irads_horizontal_axis = s_irads_keyboard_horizontal != 0.0f
                ? s_irads_keyboard_horizontal : s_irads_analog_horizontal;
            if (slewLogs++ < 48) Log("HandleIRADSCommand: SlewLeft value=%.2f axis=%.2f caged=%d", value, s_irads_horizontal_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchHorizontal();
            return true;
        case IRADSSlewRight:
            if (active) s_irads_keyboard_horizontal = axisMagnitude;
            else if (s_irads_keyboard_horizontal > 0.0f) s_irads_keyboard_horizontal = 0.0f;
            s_irads_horizontal_axis = s_irads_keyboard_horizontal != 0.0f
                ? s_irads_keyboard_horizontal : s_irads_analog_horizontal;
            if (slewLogs++ < 48) Log("HandleIRADSCommand: SlewRight value=%.2f axis=%.2f caged=%d", value, s_irads_horizontal_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchHorizontal();
            return true;
        case IRADSSlewUp:
            if (active) s_irads_keyboard_vertical = axisMagnitude;
            else if (s_irads_keyboard_vertical > 0.0f) s_irads_keyboard_vertical = 0.0f;
            s_irads_vertical_axis = s_irads_keyboard_vertical != 0.0f
                ? s_irads_keyboard_vertical : s_irads_analog_vertical;
            if (slewLogs++ < 48) Log("HandleIRADSCommand: SlewUp value=%.2f axis=%.2f caged=%d", value, s_irads_vertical_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchVertical();
            return true;
        case IRADSSlewDown:
            if (active) s_irads_keyboard_vertical = -axisMagnitude;
            else if (s_irads_keyboard_vertical < 0.0f) s_irads_keyboard_vertical = 0.0f;
            s_irads_vertical_axis = s_irads_keyboard_vertical != 0.0f
                ? s_irads_keyboard_vertical : s_irads_analog_vertical;
            if (slewLogs++ < 48) Log("HandleIRADSCommand: SlewDown value=%.2f axis=%.2f caged=%d", value, s_irads_vertical_axis, (int)(*(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938)));
            dispatchVertical();
            return true;
        case IRADSSlewStop:
            s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
            s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
            s_irads_horizontal_axis = 0.0f;
            s_irads_vertical_axis = 0.0f;
            if (g_avTVSensor_slew_stop)
                g_avTVSensor_slew_stop(g_avTVSensor_instance);
            if (active)
            {
                ClearIRADSLock();
                const bool queued=GroundStabilizeIRADS();
                Log_(queued ? "IRADS terrain designation queued" : "IRADS designation unavailable: no rendered ray");
            }
            return true;
        case TVSensorZoomIn:
        case IRADSZoomIn:
            // value_down is not set in the keyboard binding so value=0 â†’ active is always false.
            // Zoom is a momentary press, not an axis, so just call unconditionally.
            Log_("HandleIRADSCommand: ZoomIn");
            setZoomLevel((s_irads_zoom_level < 3) ? s_irads_zoom_level + 1 : 3);
            return true;
        case TVSensorZoomOut:
        case IRADSZoomOut:
            Log_("HandleIRADSCommand: ZoomOut");
            setZoomLevel((s_irads_zoom_level > 1) ? s_irads_zoom_level - 1 : 1);
            return true;
        case IRADSDesignationClear:
            ClearIRADSLock();
            SetExternalDesignation(false);
            return true;
        default:
            return false;
        }
    }

    bool IsIRADSPowered() { return s_irads_power; }

    int GetIRADSZoomLevel()
    {
        return s_irads_zoom_level;
    }

    // DCS pose axes are forward X, up Y, right Z. Preserve the native camera's
    // roll while making its final rendered LOS pass through the saved world point.
    // Native slave_to_point is rate limited and otherwise leaves a visible lag.
    static bool AimPoseAtWorldPoint(double pose[16], const double target[3])
    {
        double forward[3], up[3], right[3];
        double lengthSq = 0.0;
        for (int i=0;i<3;++i)
        {
            if (!std::isfinite(target[i]) || !std::isfinite(pose[12+i]) ||
                !std::isfinite(pose[4+i])) return false;
            forward[i] = target[i] - pose[12+i];
            lengthSq += forward[i]*forward[i];
        }
        if (!std::isfinite(lengthSq) || lengthSq < 1.0) return false;
        const double invLength = 1.0 / std::sqrt(lengthSq);
        for (int i=0;i<3;++i) forward[i] *= invLength;
        double dot = 0.0;
        for (int i=0;i<3;++i) dot += pose[4+i]*forward[i];
        lengthSq = 0.0;
        for (int i=0;i<3;++i) { up[i]=pose[4+i]-dot*forward[i]; lengthSq+=up[i]*up[i]; }
        if (lengthSq < 1.0e-12)
        {
            // Near vertical LOS: choose a nonparallel reference without a singularity.
            const int axis = std::fabs(forward[1]) < 0.9 ? 1 : 2;
            lengthSq = 0.0;
            for (int i=0;i<3;++i) { up[i]=(i==axis?1.0:0.0)-forward[axis]*forward[i]; lengthSq+=up[i]*up[i]; }
        }
        const double invUp = 1.0/std::sqrt(lengthSq);
        for (int i=0;i<3;++i) up[i] *= invUp;
        right[0]=forward[1]*up[2]-forward[2]*up[1];
        right[1]=forward[2]*up[0]-forward[0]*up[2];
        right[2]=forward[0]*up[1]-forward[1]*up[0];
        for (int i=0;i<3;++i) { pose[i]=forward[i]; pose[4+i]=up[i]; pose[8+i]=right[i]; }
        return true;
    }

    void UpdateIRADSSensorFrame(double dt,
        double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w)
    {
        if (std::isfinite(dt) && dt>0) s_irads_ray_age+=dt;
        if (!g_avTVSensor_instance || !g_avTVSensor_update) return;
        if (AsSensor(g_avTVSensor_instance)->activeFlag == 0 || !s_irads_power) return;

        if(s_irads_terrain_pending) {
            s_irads_terrain_wait+=dt;
            if(s_irads_terrain_wait>1.0) { s_irads_terrain_pending=false;Log_("IRADS terrain request timed out; no lock"); }
        }
        SeedInjectedSensorPose(
            aircraft_x, aircraft_y, aircraft_z,
            quat_x, quat_y, quat_z, quat_w);
        // Do not call set_carrier for the injected sensor: it leads to an unsafe
        // carrier chain during the sensor update and crashed DCS. Keeping
        // platformRoot self-referential preserves the native update path without
        // entering that chain.
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
                Log("UpdateIRADSSensorFrame: carrier set â†’ MovingObject=%p", movingObject);
            }
        }

        // Keep the sensor uncaged every frame.
        // initialize() cages the sensor, which blocks all slew.
        // Some DCS mode transitions may also re-cage â€” clearing it each frame
        // ensures slew is always available during IRADS operation.
        *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + 938) = 0;
        if (g_avTVSensor_uncage)
            g_avTVSensor_uncage(g_avTVSensor_instance);

        // Tick the sensor: the update drives the gimbal motors from slew commands
        // and applies their angles to the world pose. No direct pose write needed.
        if (s_irads_lock_valid)
        {
            using SlaveToPoint = void (__fastcall*)(void*, const double*);
            static auto slave = reinterpret_cast<SlaveToPoint>(GetProcAddress(GetModuleHandleA("CockpitBase.dll"),
                "?slave_to_point@avTVSensor@cockpit@@UEAAXAEBVVec3d@osg@@@Z"));
            if (slave) slave(g_avTVSensor_instance, s_irads_lock_world);
        }
        g_avTVSensor_update(g_avTVSensor_instance, dt);
        if (g_avTVSensor_update_frame)
            g_avTVSensor_update_frame(g_avTVSensor_instance);
        // Apply after all native motor/frame work. Do not change the initial mount,
        // target coordinate, optical zoom or free-slew state.
        if (s_irads_lock_valid)
            AimPoseAtWorldPoint(AsSensor(g_avTVSensor_instance)->platformWorldPose.m, s_irads_lock_world);

        static double diagnosticTime = 0;
        static unsigned int diagnosticCount = 0;
        diagnosticTime += dt;
        if (diagnosticTime >= 2.0 && diagnosticCount < 60)
        {
            diagnosticTime = 0;
            ++diagnosticCount;
            const auto* sensor = AsSensor(g_avTVSensor_instance);
            Log("IRADS camera state: bridged=%d zoom=%.3f targetZoom=%.3f LOS=(%.4f,%.4f,%.4f) axes=(%.3f,%.3f)",
                s_our_ind != nullptr, sensor->currentZoom, sensor->targetZoom,
                sensor->platformWorldPose.m[0], sensor->platformWorldPose.m[1], sensor->platformWorldPose.m[2],
                s_irads_horizontal_axis, s_irads_vertical_axis);
        }
    }

    bool CaptureIRADSLock()
    {
        ClearIRADSLock();
        return GroundStabilizeIRADS(); // queued, not a valid lock until terrain reply
    }

    bool GetIRADSTerrainRequest(unsigned* sequence, double ray[6])
    {
        if(!s_irads_terrain_pending || !sequence || !ray) return false;
        *sequence=s_irads_terrain_sequence;
        std::memcpy(ray,s_irads_request_ray,sizeof(s_irads_request_ray));
        return true;
    }

    bool CompleteIRADSTerrainRequest(unsigned sequence, bool hit, const double point[3])
    {
        if(!s_irads_terrain_pending || sequence!=s_irads_terrain_sequence) return false;
        s_irads_terrain_pending=false;
        if(!hit || !point || !s_irads_power || !g_avTVSensor_instance) return false;
        double distanceSq=0,dot=0;
        for(int i=0;i<3;++i) {
            if(!std::isfinite(point[i])) return false;
            double delta=point[i]-s_irads_request_ray[i];
            distanceSq+=delta*delta;dot+=delta*s_irads_request_ray[i+3];
        }
        const double range=std::sqrt(distanceSq);
        if(!std::isfinite(range) || range<1 || range>=80000 || dot/range<0.99999999) return false;
        for(int i=0;i<3;++i) {
            s_irads_lock_world[i]=point[i];
            AsSensor(g_avTVSensor_instance)->cachedTrackedPoint[i]=point[i];
        }
        s_irads_lock_polar[0]=std::atan2(point[2]-s_irads_request_ray[2],point[0]-s_irads_request_ray[0]);
        s_irads_lock_polar[1]=std::asin((point[1]-s_irads_request_ray[1])/range);
        s_irads_lock_polar[2]=range;
        s_irads_lock_valid=true;
        Log("IRADS terrain lock accepted: id=%u world=(%.3f,%.3f,%.3f) range=%.2f",sequence,point[0],point[1],point[2],range);
        return true;
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

    bool GetIRADSLineOfSight(double out_direction[3])
    {
        if (!out_direction || !s_irads_power || !s_irads_ray_valid || s_irads_ray_age>kIRADSRayMaxAge)
            return false;
        double norm=0;
        for (int i=0;i<3;++i) norm+=s_irads_render_ray[i+3]*s_irads_render_ray[i+3];
        if (!std::isfinite(norm) || norm<0.5) return false;
        norm=std::sqrt(norm);
        for (int i=0;i<3;++i) out_direction[i]=s_irads_render_ray[i+3]/norm;
        return true;
    }

    bool GetIRADSLockPolar(double out_polar[3])
    {
        if (!s_irads_lock_valid || !out_polar)
            return false;
        const double dx=s_irads_lock_world[0]-s_last_aircraft_pos[0];
        const double dy=s_irads_lock_world[1]-s_last_aircraft_pos[1];
        const double dz=s_irads_lock_world[2]-s_last_aircraft_pos[2];
        const double range=std::sqrt(dx*dx+dy*dy+dz*dz);
        out_polar[0]=std::atan2(dz,dx);
        out_polar[1]=range>0 ? std::asin(dy/range) : 0.0;
        out_polar[2]=range;
        return true;
    }


    // â”€â”€ Phase C: per-frame IRADS indicator keep-alive â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Called every frame from ed_fm_simulate. On the first call after indicators
    // are populated (post device_init.lua), it locates the ccIndicator whose
    // controller pointer matches our injected avTVSensor. Every subsequent frame
    // it forces screenCondition = FLIR (11) and cameraCondition = ON (1) to
    // prevent DCS from reverting the indicator to a non-camera state.

    using IndicatorGetCamera = void* (__fastcall*)(void*);
    static IndicatorGetCamera s_original_get_camera = nullptr;
    using SensorFillCamera = void (__fastcall*)(void*, void*);
    static SensorFillCamera s_sensor_fill_camera = nullptr;
    static void* s_camera_bridge_indicator = nullptr;

    static void* __fastcall IRADSGetCamera(void* indicator)
    {
        void* camera = s_original_get_camera(indicator);
        if (!camera || !g_avTVSensor_instance) return camera;
        auto* sensor = AsSensor(g_avTVSensor_instance);
        const bool locked = s_irads_lock_valid &&
            AimPoseAtWorldPoint(sensor->platformWorldPose.m, s_irads_lock_world);
        // The native camera normally extrapolates the pose from mount and motor
        // history between updates.
        // That overwrites our corrected pose. Suppress ONLY this render prediction
        // while building a locked camera, then restore the native update timestamp.
        const double savedUpdateTime = sensor->platformLastUpdateTime;
        if (locked) sensor->platformLastUpdateTime = (std::numeric_limits<double>::max)();
        if (s_sensor_fill_camera) s_sensor_fill_camera(g_avTVSensor_instance, camera);
        if (locked) sensor->platformLastUpdateTime = savedUpdateTime;
        // Snapshot the visible ray. Terrain tracing must use this optical eye,
        // not a native cached target or a newly recomputed aircraft-forward ray.
        if(s_irads_power) {
            float eye[3],forward[3];
            std::memcpy(eye,static_cast<char*>(camera)+0x260,sizeof(eye));
            std::memcpy(forward,static_cast<char*>(camera)+0x230,sizeof(forward));
            s_irads_ray_valid=true;
            for(int i=0;i<3;++i) {
                s_irads_render_ray[i]=eye[i];s_irads_render_ray[i+3]=forward[i];
                if(!std::isfinite(eye[i]) || !std::isfinite(forward[i])) s_irads_ray_valid=false;
            }
            if(s_irads_ray_valid) s_irads_ray_age=0;
        }
        static unsigned renderSamples=0, renderFrames=0;
        if (locked && renderSamples<120 && (++renderFrames==1 || renderFrames%120==0))
        {
            // Observe the FINAL rendered ray, not just the cached pose.
            float forward[3],eye[3];
            std::memcpy(forward,static_cast<char*>(camera)+0x230,sizeof(forward));
            std::memcpy(eye,static_cast<char*>(camera)+0x260,sizeof(eye));
            double dot=0,rangeSq=0,axisSq=0;
            for(int i=0;i<3;++i) {const double d=s_irads_lock_world[i]-eye[i];dot+=d*forward[i];rangeSq+=d*d;axisSq+=forward[i]*forward[i];}
            if (rangeSq>1 && axisSq>0.5)
            {
                double cosine=dot/std::sqrt(rangeSq*axisSq);cosine=cosine>1?1:(cosine< -1?-1:cosine);
                Log("IRADS render aim: errorDeg=%.6f range=%.1f eye=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f)",
                    std::acos(cosine)*57.295779513,std::sqrt(rangeSq),eye[0],eye[1],eye[2],
                    s_irads_lock_world[0],s_irads_lock_world[1],s_irads_lock_world[2]);
                ++renderSamples;
            }
        }
        return camera;
    }

    static bool InstallIRADSCameraBridge(void* indicator)
    {
        if (s_camera_bridge_indicator == indicator) return true;
        auto nativeGetCamera = reinterpret_cast<IndicatorGetCamera>(
            ResolveCockpitBaseExport( "?get_camera@ccIndicator@cockpit@@UEAAAEBVCamera@Graphics@@XZ"));
        auto** vtable = *reinterpret_cast<void***>(indicator);
        if (!nativeGetCamera || !vtable) return false;
        s_sensor_fill_camera = reinterpret_cast<SensorFillCamera>(ResolveCockpitBaseExport(
            "?fill_camera@avTVSensor@cockpit@@UEBAXAEAVCamera@Graphics@@@Z"));
        if (!s_sensor_fill_camera) { Log_("IRADS camera bridge: native fill_camera unavailable"); return false; }
        constexpr size_t slots = 128;
        size_t slot = slots;
        for (size_t i = 0; i < slots; ++i)
            if (vtable[i] == reinterpret_cast<void*>(nativeGetCamera)) { slot = i; break; }
        if (slot == slots) { Log_("IRADS camera bridge: get_camera slot not found; disabled"); return false; }
        // Keep the type information entry ahead of the copied table for native RTTI.
        auto** copy = new void*[slots + 1];
        std::memcpy(copy, vtable - 1, sizeof(void*) * (slots + 1));
        s_original_get_camera = nativeGetCamera;
        copy[slot + 1] = reinterpret_cast<void*>(&IRADSGetCamera);
        *reinterpret_cast<void***>(indicator) = copy + 1;
        s_camera_bridge_indicator = indicator;
        Log("IRADS camera bridge installed: indicator=%p slot=%zu sensor=%p", indicator, slot, g_avTVSensor_instance);
        return true;
    }

    void ForceIRADSIndicatorActive()
    {
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
                Log("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via global ctx",
                    s_our_ind, foundDepth);
            }
            else if (bindingCtx != globalCtx &&
                     FindIndicatorInContextTree(bindingCtx, &s_our_ind, &foundDepth, 0, false))
            {
                Log("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via binding ctx",
                    s_our_ind, foundDepth);
            }
            else if (FindIndicatorInContextTree(globalCtx, &s_our_ind, &foundDepth, 0, true))
            {
                Log("ForceIRADSIndicatorActive: found DLIR camera render target @ %p depth=%d via global ctx",
                    s_our_ind, foundDepth);
            }
            else if (bindingCtx != globalCtx &&
                     FindIndicatorInContextTree(bindingCtx, &s_our_ind, &foundDepth, 0, true))
            {
                Log("ForceIRADSIndicatorActive: found DLIR camera render target @ %p depth=%d via binding ctx",
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
            if (!InstallIRADSCameraBridge(s_our_ind)) return;
            CcIndicatorLayout* indicatorView = AsIndicator(s_our_ind);
            indicatorView->screenCondition = static_cast<uint32_t>(IR_SCREEN_CONDITION_FLIR);
            indicatorView->cameraCondition = s_irads_power ? static_cast<uint32_t>(IR_CAMERA_CONDITION_ON) : 0;
        }
    }

    // Called from ed_fm_release. Cockpit teardown destroys the
    // context's indicators, so drop every pointer
    // into that context and re-inject on the next start. The old sensor block
    // is intentionally not freed: ownership after cockpit teardown is
    // unconfirmed, and a small leak is safer than a double free.
    void ResetIRADSSession()
    {
        ClearIRADSLock();
        s_irads_ray_valid=false;
        s_irads_ray_age=1e9;
        s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
        s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
        s_irads_horizontal_axis = 0.0f;
        s_irads_vertical_axis   = 0.0f;
        s_irads_zoom_level      = 1;
        s_irads_power           = true;
        s_camera_bridge_indicator = nullptr;
        s_our_ind               = nullptr;
        g_avTVSensor_instance   = nullptr;
        s_added                 = false;
        s_started               = false;
        s_carrier_set           = false;
        Log_("ResetIRADSSession: IRADS session state cleared");
    }

} // namespace CockpitInterop
