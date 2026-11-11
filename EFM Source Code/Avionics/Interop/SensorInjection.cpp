// Construction and start-up of the standalone avTVSensor.
// Early runs from DllMain (construct); Late runs from ed_fm_start (initialize, mount, zoom, IR mode).
#include "stdafx.h"
#include "IradsInternal.h"
#include <cstring>
#include <malloc.h>

namespace CockpitInterop
{
    // Native call position used by the sensor bridge.
    static constexpr size_t kVtableSlotSetPos = 4;     // sets the sensor's local mount position

    // Zoom presets are 1.4 / 10 / 80; the sensor starts at the widest.
    static constexpr double kInitialZoom = 1.4;

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
        auto fn = reinterpret_cast<SetPosFn>(vtbl[kVtableSlotSetPos]);
        fn(sensor, mat);

        Log("SetCameraPosition: vtable[4] called, pos=(%.2f, %.2f, %.2f)",
            IRADS_POS_X, IRADS_POS_Y, IRADS_POS_Z);
    }

    // -- Phase A ---------------------------------------------------------------

    bool InjectIRADSSensor_Early()
    {
        Log_("InjectIRADSSensor_Early: called");
        if (s_added) { Log_("InjectIRADSSensor_Early: already done."); return true; }
        if (!Initialize()) return false;

        CcCockpitContextLayout* ctx = AsContext(*g_contexts_ptr_addr);
        Log("InjectIRADSSensor_Early: contexts_ptr value = %p", static_cast<void*>(ctx));
        if (!ctx) { Log_("InjectIRADSSensor_Early: context NULL - too early, will retry in Late."); return false; }

        // -- Construct avTVSensor ----------------------------------------------
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
            Log("InjectIRADSSensor_Early: avTVSensor constructed @ %p", mem);
        }

        s_added = true;
        Log_("InjectIRADSSensor_Early: done.");
        return true;
    }

    // -- Phase B ---------------------------------------------------------------

    static double displayed_fov_deg(const AvTVSensorLayout* sensor)
    {
        return (sensor->currentZoom > 0.001 ? sensor->baseFov / sensor->currentZoom : 0.0) * kRadToDeg;
    }

    // This is a standalone avBasicSensor; initialize it directly, never through avDevice::start
    // or the cockpit device keeper.
    static void initialize_standalone_sensor(AvTVSensorLayout* sensor)
    {
        uint8_t activeFlag = sensor->activeFlag;
        Log("InjectIRADSSensor_Late: sensor+%zu (active flag) = %d",
            offsetof(AvTVSensorLayout, activeFlag), static_cast<int>(activeFlag));

        if (activeFlag != 0)
        {
            Log_("InjectIRADSSensor_Late: standalone sensor already initialized.");
            return;
        }
        if (!g_avTVSensor_initialize)
        {
            Log_("InjectIRADSSensor_Late: native initialize export missing; sensor unavailable");
            return;
        }
        Log_("InjectIRADSSensor_Late: initializing standalone sensor.");
        g_avTVSensor_initialize(g_avTVSensor_instance);
        activeFlag = sensor->activeFlag;
        Log("InjectIRADSSensor_Late: after explicit initialize(), active flag = %d", static_cast<int>(activeFlag));
        if (activeFlag == 0)
            Log_("InjectIRADSSensor_Late: WARNING - initialize() ran but flag still 0 "
                 "(SearchTimer alloc may have failed inside initialize)");
    }

    // cockpit::avTVSensor::preffered_IR_effect is an exported static, so resolve it by name so it
    // keeps working across DCS updates. ForceIRADSIndicatorActive also forces screenCondition=11
    // per-frame, but setting the global static ensures the render pass is selected before the first
    // indicator draw call.
    static void select_flir_render_pass()
    {
        int* effect = reinterpret_cast<int*>(
            ResolveCockpitBaseExport("?preffered_IR_effect@avTVSensor@cockpit@@2HA"));
        if (!effect)
        {
            Log_("InjectIRADSSensor_Late: WARNING - preffered_IR_effect export not found; skipping FLIR static write");
            return;
        }
        *effect = IR_SCREEN_CONDITION_FLIR;
        Log("InjectIRADSSensor_Late: preffered_IR_effect@%p set to %d",
            static_cast<void*>(effect), IR_SCREEN_CONDITION_FLIR);
    }

    // Zoom is motor-driven and has its own preset list. Initialize the native zoom motor, then
    // the caller sets both current and target for the 1.4/10/80 presets.
    static void initialize_zoom(AvTVSensorLayout* sensor)
    {
        Log("InjectIRADSSensor_Late: camera params before fix: "
            "fov_base=%.4f zoom=%.4f aspect=%.4f  (displayed_fov=%.1f deg)",
            sensor->baseFov, sensor->currentZoom, sensor->aspectRatio, displayed_fov_deg(sensor));
        if (g_avTVSensor_initZoom)
        {
            g_avTVSensor_initZoom(g_avTVSensor_instance);
            Log("InjectIRADSSensor_Late: initZoom() ran -> displayed FOV = %.1f deg", displayed_fov_deg(sensor));
        }
        else
        {
            sensor->currentZoom = 1.0;
            Log("InjectIRADSSensor_Late: initZoom missing, fallback zoom reset to 1.0 -> displayed FOV = %.1f deg",
                sensor->baseFov * kRadToDeg);
        }
    }

    static size_t pointer_span(void** begin, void** end)
    {
        return (begin && end && end > begin) ? static_cast<size_t>(end - begin) : 0u;
    }

    // Logs the indicator state that decides whether the DLIR camera feed renders.
    static void log_context_indicators(const char* label, CcCockpitContextLayout* context)
    {
        if (!context)
        {
            Log("InjectIRADSSensor_Late: %s ctx is NULL", label);
            return;
        }
        Log("InjectIRADSSensor_Late: %s ctx=%p indicators [%p .. %p] count=%zu children=%zu",
            label,
            static_cast<void*>(context),
            static_cast<void*>(context->indicators.begin),
            static_cast<void*>(context->indicators.end),
            pointer_span(context->indicators.begin, context->indicators.end),
            pointer_span(context->childContexts.begin, context->childContexts.end));
    }

    bool InjectIRADSSensor_Late()
    {
        Log_("InjectIRADSSensor_Late: called");
        if (!Initialize()) return false;

        // Retry Early if DllMain ran before the cockpit context existed.
        if (!s_added)
        {
            Log_("InjectIRADSSensor_Late: Early not done - retrying now.");
            if (!InjectIRADSSensor_Early())
            {
                Log_("InjectIRADSSensor_Late: Early retry failed - cannot proceed.");
                return false;
            }
        }

        CcCockpitContextLayout* ctx = GetGlobalContext();
        CcCockpitContextLayout* bindingCtx = GetBindingContext();
        Log("InjectIRADSSensor_Late: global ctx=%p  binding ctx=%p",
            static_cast<void*>(ctx), static_cast<void*>(bindingCtx));
        if (!ctx && !bindingCtx) { Log_("InjectIRADSSensor_Late: no cockpit context."); return false; }
        if (!ctx)
            ctx = bindingCtx;

        if (!g_avTVSensor_instance) { Log_("InjectIRADSSensor_Late: no sensor instance!"); return false; }
        AvTVSensorLayout* sensor = AsSensor(g_avTVSensor_instance);

        initialize_standalone_sensor(sensor);

        // initialize() leaves the sensor caged; clear it so manual gimbal control is available
        // from mission start.
        uncage_sensor();
        Log_("InjectIRADSSensor_Late: sensor uncaged (cage flag cleared)");

        // initialize() has no modelled position for this sensor station, so it uses a default.
        // Override with the F-117A DLIR belly position. Safe to repeat: it overwrites the result.
        Log_("InjectIRADSSensor_Late: setting IRADS camera mount position.");
        SetCameraPosition(g_avTVSensor_instance);

        select_flir_render_pass();
        initialize_zoom(sensor);

        log_context_indicators("global", ctx);
        if (bindingCtx != ctx)
            log_context_indicators("binding", bindingCtx);

        sensor->currentZoom = kInitialZoom;
        sensor->targetZoom = kInitialZoom;
        Log_("InjectIRADSSensor_Late: done.");
        return true;
    }
} // namespace CockpitInterop
