// Construction and start-up of the standalone avTVSensor.
// Early runs from DllMain (construct); Late runs from ed_fm_start (initialize, mount, zoom, IR mode).
#include "stdafx.h"
#include "IradsInternal.h"
#include <cstring>
#include <malloc.h>

namespace Cockpit_interop
{
// Native call position used by the sensor bridge.
static constexpr size_t vtable_slot_set_pos = 4;  // sets the sensor's local mount position

// Zoom presets are 1.4 / 10 / 80; the sensor starts at the widest.
static constexpr double initial_zoom = 1.4;

// Set the standalone sensor mount through avPlatform.
static void set_camera_position(void* sensor)
{
    double mat[16] = {1.0,         0.0,         0.0,         0.0,  // right axis (body X)
                      0.0,         1.0,         0.0,         0.0,  // up axis    (body Y)
                      0.0,         0.0,         1.0,         0.0,  // fwd axis   (body Z)
                      irads_pos_x, irads_pos_y, irads_pos_z, 1.0};

    using Set_pos_fn = void(__fastcall*)(void*, double*);
    uintptr_t* vtbl = *reinterpret_cast<uintptr_t**>(sensor);
    auto fn = reinterpret_cast<Set_pos_fn>(vtbl[vtable_slot_set_pos]);
    fn(sensor, mat);

    log_format("SetCameraPosition: vtable[4] called, pos=(%.2f, %.2f, %.2f)", irads_pos_x, irads_pos_y, irads_pos_z);
}

// -- Phase A ---------------------------------------------------------------

bool inject_irads_sensor_early()
{
    log_message("InjectIRADSSensor_Early: called");
    if (s_added)
    {
        log_message("InjectIRADSSensor_Early: already done.");
        return true;
    }
    if (!initialize())
        return false;

    Cc_cockpit_context_layout* ctx = as_context(*g_contexts_ptr_addr);
    log_format("InjectIRADSSensor_Early: contexts_ptr value = %p", static_cast<void*>(ctx));
    if (!ctx)
    {
        log_message("InjectIRADSSensor_Early: context NULL - too early, will retry in Late.");
        return false;
    }

    // -- Construct avTVSensor ----------------------------------------------
    if (!g_av_tv_sensor_instance)
    {
        void* mem = _aligned_malloc(avtvsensor_alloc_size, 16);
        if (!mem)
        {
            log_message("InjectIRADSSensor_Early: alloc failed.");
            return false;
        }
        memset(mem, 0, avtvsensor_alloc_size);

        void* result = g_av_tv_sensor_ctor(mem);
        if (!result)
        {
            log_message("InjectIRADSSensor_Early: ctor returned null.");
            _aligned_free(mem);
            return false;
        }

        using Set_platform = void(__fastcall*)(void*, void*);
        const auto set_platform = reinterpret_cast<Set_platform>(
            resolve_cockpit_base_export("?set_platform@avPlatform@cockpit@@UEAAXPEAV12@@Z"));
        if (!set_platform)
        {
            log_message("IRADS: set_platform missing; initialization stopped");
            return false;
        }
        set_platform(mem, nullptr);  // native null-parent path establishes root=self
        g_av_tv_sensor_instance = mem;
        log_format("InjectIRADSSensor_Early: avTVSensor constructed @ %p", mem);
    }

    s_added = true;
    log_message("InjectIRADSSensor_Early: done.");
    return true;
}

// -- Phase B ---------------------------------------------------------------

static double displayed_fov_deg(const Av_tv_sensor_layout* sensor)
{
    return (sensor->current_zoom > 0.001 ? sensor->base_fov / sensor->current_zoom : 0.0) * rad_to_deg;
}

// This is a standalone avBasicSensor; initialize it directly, never through avDevice::start
// or the cockpit device keeper.
static void initialize_standalone_sensor(Av_tv_sensor_layout* sensor)
{
    uint8_t active_flag = sensor->active_flag;
    log_format("InjectIRADSSensor_Late: sensor+%zu (active flag) = %d", offsetof(Av_tv_sensor_layout, active_flag),
               static_cast<int>(active_flag));

    if (active_flag != 0)
    {
        log_message("InjectIRADSSensor_Late: standalone sensor already initialized.");
        return;
    }
    if (!g_av_tv_sensor_initialize)
    {
        log_message("InjectIRADSSensor_Late: native initialize export missing; sensor unavailable");
        return;
    }
    log_message("InjectIRADSSensor_Late: initializing standalone sensor.");
    g_av_tv_sensor_initialize(g_av_tv_sensor_instance);
    active_flag = sensor->active_flag;
    log_format("InjectIRADSSensor_Late: after explicit initialize(), active flag = %d", static_cast<int>(active_flag));
    if (active_flag == 0)
        log_message("InjectIRADSSensor_Late: WARNING - initialize() ran but flag still 0 "
                    "(SearchTimer alloc may have failed inside initialize)");
}

// cockpit::avTVSensor::preffered_IR_effect is an exported static, so resolve it by name so it
// keeps working across DCS updates. ForceIRADSIndicatorActive also forces screenCondition=11
// per-frame, but setting the global static ensures the render pass is selected before the first
// indicator draw call.
static void select_flir_render_pass()
{
    int* effect = reinterpret_cast<int*>(resolve_cockpit_base_export("?preffered_IR_effect@avTVSensor@cockpit@@2HA"));
    if (!effect)
    {
        log_message(
            "InjectIRADSSensor_Late: WARNING - preffered_IR_effect export not found; skipping FLIR static write");
        return;
    }
    *effect = ir_screen_condition_flir;
    log_format("InjectIRADSSensor_Late: preffered_IR_effect@%p set to %d", static_cast<void*>(effect),
               ir_screen_condition_flir);
}

// Zoom is motor-driven and has its own preset list. Initialize the native zoom motor, then
// the caller sets both current and target for the 1.4/10/80 presets.
static void initialize_zoom(Av_tv_sensor_layout* sensor)
{
    log_format("InjectIRADSSensor_Late: camera params before fix: "
               "fov_base=%.4f zoom=%.4f aspect=%.4f  (displayed_fov=%.1f deg)",
               sensor->base_fov, sensor->current_zoom, sensor->aspect_ratio, displayed_fov_deg(sensor));
    if (g_av_tv_sensor_init_zoom)
    {
        g_av_tv_sensor_init_zoom(g_av_tv_sensor_instance);
        log_format("InjectIRADSSensor_Late: initZoom() ran -> displayed FOV = %.1f deg", displayed_fov_deg(sensor));
    }
    else
    {
        sensor->current_zoom = 1.0;
        log_format("InjectIRADSSensor_Late: initZoom missing, fallback zoom reset to 1.0 -> displayed FOV = %.1f deg",
                   sensor->base_fov * rad_to_deg);
    }
}

static size_t pointer_span(void** begin, void** end)
{
    return (begin && end && end > begin) ? static_cast<size_t>(end - begin) : 0u;
}

// Logs the indicator state that decides whether the DLIR camera feed renders.
static void log_context_indicators(const char* label, Cc_cockpit_context_layout* context)
{
    if (!context)
    {
        log_format("InjectIRADSSensor_Late: %s ctx is NULL", label);
        return;
    }
    log_format("InjectIRADSSensor_Late: %s ctx=%p indicators [%p .. %p] count=%zu children=%zu", label,
               static_cast<void*>(context), static_cast<void*>(context->indicators.begin),
               static_cast<void*>(context->indicators.end),
               pointer_span(context->indicators.begin, context->indicators.end),
               pointer_span(context->child_contexts.begin, context->child_contexts.end));
}

bool inject_irads_sensor_late()
{
    log_message("InjectIRADSSensor_Late: called");
    if (!initialize())
        return false;

    // Retry Early if DllMain ran before the cockpit context existed.
    if (!s_added)
    {
        log_message("InjectIRADSSensor_Late: Early not done - retrying now.");
        if (!inject_irads_sensor_early())
        {
            log_message("InjectIRADSSensor_Late: Early retry failed - cannot proceed.");
            return false;
        }
    }

    Cc_cockpit_context_layout* ctx = get_global_context();
    Cc_cockpit_context_layout* binding_ctx = get_binding_context();
    log_format("InjectIRADSSensor_Late: global ctx=%p  binding ctx=%p", static_cast<void*>(ctx),
               static_cast<void*>(binding_ctx));
    if (!ctx && !binding_ctx)
    {
        log_message("InjectIRADSSensor_Late: no cockpit context.");
        return false;
    }
    if (!ctx)
        ctx = binding_ctx;

    if (!g_av_tv_sensor_instance)
    {
        log_message("InjectIRADSSensor_Late: no sensor instance!");
        return false;
    }
    Av_tv_sensor_layout* sensor = as_sensor(g_av_tv_sensor_instance);

    initialize_standalone_sensor(sensor);

    // initialize() leaves the sensor caged; clear it so manual gimbal control is available
    // from mission start.
    uncage_sensor();
    log_message("InjectIRADSSensor_Late: sensor uncaged (cage flag cleared)");

    // initialize() has no modelled position for this sensor station, so it uses a default.
    // Override with the F-117A DLIR belly position. Safe to repeat: it overwrites the result.
    log_message("InjectIRADSSensor_Late: setting IRADS camera mount position.");
    set_camera_position(g_av_tv_sensor_instance);

    select_flir_render_pass();
    initialize_zoom(sensor);

    log_context_indicators("global", ctx);
    if (binding_ctx != ctx)
        log_context_indicators("binding", binding_ctx);

    sensor->current_zoom = initial_zoom;
    sensor->target_zoom = initial_zoom;
    log_message("InjectIRADSSensor_Late: done.");
    return true;
}
}  // namespace Cockpit_interop
