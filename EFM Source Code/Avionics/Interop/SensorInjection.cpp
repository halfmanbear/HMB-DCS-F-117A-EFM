// Construction and start-up of the standalone avTVSensor.
// Early runs from DllMain (construct); Late runs from ed_fm_start (initialize, mount, zoom, IR mode).
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <malloc.h>

namespace Cockpit_interop
{
// DCS exports no size for avTVSensor, so the sensor gets a generous block, about nine times
// the current class. After construction the EFM finds the last byte the constructor set and
// refuses the sensor if it reaches past half the block.
static constexpr size_t sensor_block_bytes = 0x4000;
static constexpr size_t sensor_size_limit = sensor_block_bytes / 2;
static bool s_sensor_too_large = false;  // stays set for the process; the check does not change

static size_t constructed_extent(const void* block)
{
    const auto* bytes = static_cast<const std::uint8_t*>(block);
    size_t end = sensor_block_bytes;
    while (end > 0 && bytes[end - 1] == 0)
        --end;
    return end;
}

// Zoom presets are 1.4 / 10 / 80; the sensor starts at the widest.
static constexpr double initial_zoom = 1.4;

// The zoom motor is an avMotor member of the sensor with no accessor. Its position is read from
// the exported zoom functions, then confirmed once against getCurrentZoom() before the EFM drives it.
static int s_zoom_motor_state = 0;  // 0 unchecked, 1 confirmed, -1 rejected

// Set the standalone sensor mount through avPlatform's exported setter, then read it back
// through the exported getter.
static void set_camera_position(void* sensor)
{
    const double mat[16] = {1.0,         0.0,         0.0,         0.0,  // right axis (body X)
                            0.0,         1.0,         0.0,         0.0,  // up axis    (body Y)
                            0.0,         0.0,         1.0,         0.0,  // fwd axis   (body Z)
                            irads_pos_x, irads_pos_y, irads_pos_z, 1.0};

    using Set_pos_fn = void(__fastcall*)(void*, const double*);
    using Get_pos_fn = const double*(__fastcall*)(const void*);
    const HMODULE cockpit = GetModuleHandleA("CockpitBase.dll");
    const auto set_pos =
        reinterpret_cast<Set_pos_fn>(unique_export(cockpit, "?setPlatformInitialLocalPosition@avPlatform@cockpit@@"));
    const auto get_pos =
        reinterpret_cast<Get_pos_fn>(unique_export(cockpit, "?getPlatformInitialLocalPosition@avPlatform@cockpit@@"));
    if (!set_pos || !get_pos)
    {
        log_message("SetCameraPosition: mount position exports missing; default mount kept");
        return;
    }
    set_pos(sensor, mat);
    const double* pos = get_pos(sensor);
    const bool confirmed = pos && pos[12] == irads_pos_x && pos[13] == irads_pos_y && pos[14] == irads_pos_z;
    log_format("SetCameraPosition: pos=(%.2f, %.2f, %.2f) %s", irads_pos_x, irads_pos_y, irads_pos_z,
               confirmed ? "confirmed" : "NOT confirmed");
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

    void* ctx = get_global_context();
    log_format("InjectIRADSSensor_Early: context %s", ctx ? "ok" : "missing");
    if (!ctx)
    {
        log_message("InjectIRADSSensor_Early: context NULL - too early, will retry in Late.");
        return false;
    }

    // -- Construct avTVSensor ----------------------------------------------
    if (s_sensor_too_large)
        return false;
    if (!g_av_tv_sensor_instance)
    {
        void* mem = _aligned_malloc(sensor_block_bytes, 16);
        if (!mem)
        {
            log_message("InjectIRADSSensor_Early: alloc failed.");
            return false;
        }
        memset(mem, 0, sensor_block_bytes);

        void* result = g_av_tv_sensor_ctor(mem);
        if (!result)
        {
            log_message("InjectIRADSSensor_Early: ctor returned null.");
            _aligned_free(mem);
            return false;
        }
        const size_t extent = constructed_extent(mem);
        log_format("InjectIRADSSensor_Early: sensor uses at least %zu of %zu bytes", extent, sensor_block_bytes);
        if (extent > sensor_size_limit)
        {
            // The block is not freed: the constructed sensor may already be registered with DCS.
            log_message("InjectIRADSSensor_Early: sensor larger than expected; IRADS disabled");
            s_sensor_too_large = true;
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
        log_message("InjectIRADSSensor_Early: avTVSensor constructed");
    }

    s_added = true;
    log_message("InjectIRADSSensor_Early: done.");
    return true;
}

// -- Phase B ---------------------------------------------------------------

// Confirms the zoom motor position: avMotor::get must read the same value as the sensor's own
// getCurrentZoom(), and a probe written through avMotor::force must show up there. A mismatch is
// restored at once and zoom control stays off.
static void* zoom_motor()
{
    const size_t offset = zoom_motor_offset();
    if (!offset)
    {
        if (s_zoom_motor_state == 0)
            log_message("IRADS zoom motor position not found; optical zoom disabled");
        s_zoom_motor_state = -1;
        return nullptr;
    }
    void* motor = static_cast<char*>(g_av_tv_sensor_instance) + offset;
    if (s_zoom_motor_state == 0)
    {
        const double current = g_av_tv_sensor_zoom(g_av_tv_sensor_instance);
        bool confirmed = std::isfinite(current) && g_motor_get(motor) == current;
        if (confirmed)
        {
            const double probe = current * 2.0 + 1.0;
            g_motor_force(motor, probe);
            confirmed = g_av_tv_sensor_zoom(g_av_tv_sensor_instance) == probe;
            g_motor_force(motor, current);
        }
        s_zoom_motor_state = confirmed ? 1 : -1;
        log_message(confirmed ? "IRADS zoom motor confirmed" : "IRADS zoom motor not confirmed; optical zoom disabled");
    }
    return s_zoom_motor_state == 1 ? motor : nullptr;
}

bool set_optical_zoom(double zoom)
{
    if (!g_av_tv_sensor_instance || !s_sensor_initialized)
        return false;
    void* motor = zoom_motor();
    if (!motor)
        return false;
    // force() moves the motor without its travel limits; stop() then sets the target to match.
    g_motor_force(motor, zoom);
    g_motor_stop(motor);
    return true;
}

double optical_zoom_target()
{
    if (!g_av_tv_sensor_instance || s_zoom_motor_state != 1)
        return std::numeric_limits<double>::quiet_NaN();
    return g_motor_get_target(zoom_motor());
}

static double displayed_fov_deg()
{
    const double zoom = g_av_tv_sensor_zoom(g_av_tv_sensor_instance);
    return (zoom > 0.001 ? g_av_tv_sensor_view_angle(g_av_tv_sensor_instance) : 0.0) * rad_to_deg;
}

// This is a standalone avBasicSensor; initialize it directly, never through avDevice::start
// or the cockpit device keeper.
static void initialize_standalone_sensor()
{
    if (s_sensor_initialized)
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
    s_sensor_initialized = true;
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
    log_format("InjectIRADSSensor_Late: preffered_IR_effect set to %d", ir_screen_condition_flir);
}

// Zoom is motor-driven and has its own preset list. Initialize the native zoom motor; the
// caller then sets the 1.4/10/80 preset.
static void initialize_zoom()
{
    log_format("InjectIRADSSensor_Late: camera params before fix: fov_base=%.4f zoom=%.4f  (displayed_fov=%.1f deg)",
               g_av_tv_sensor_default_fov(g_av_tv_sensor_instance), g_av_tv_sensor_zoom(g_av_tv_sensor_instance),
               displayed_fov_deg());
    if (g_av_tv_sensor_init_zoom)
    {
        g_av_tv_sensor_init_zoom(g_av_tv_sensor_instance);
        log_format("InjectIRADSSensor_Late: initZoom() ran -> displayed FOV = %.1f deg", displayed_fov_deg());
    }
    else
    {
        log_message("InjectIRADSSensor_Late: initZoom missing; using the explicit zoom preset only");
    }
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

    void* ctx = get_global_context();
    void* binding_ctx = get_binding_context();
    log_format("InjectIRADSSensor_Late: global ctx=%s  binding ctx=%s", ctx ? "ok" : "missing",
               binding_ctx ? "ok" : "missing");
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
    initialize_standalone_sensor();

    // initialize() leaves the sensor caged; clear it so manual gimbal control is available
    // from mission start.
    uncage_sensor();
    log_message("InjectIRADSSensor_Late: sensor uncaged (cage flag cleared)");

    // initialize() has no modelled position for this sensor station, so it uses a default.
    // Override with the F-117A DLIR belly position. Safe to repeat: it overwrites the result.
    log_message("InjectIRADSSensor_Late: setting IRADS camera mount position.");
    set_camera_position(g_av_tv_sensor_instance);

    select_flir_render_pass();
    initialize_zoom();

    if (set_optical_zoom(initial_zoom))
        log_format("InjectIRADSSensor_Late: optical zoom %.2f -> displayed FOV = %.1f deg", initial_zoom,
                   displayed_fov_deg());
    log_message("InjectIRADSSensor_Late: done.");
    return true;
}
}  // namespace Cockpit_interop
