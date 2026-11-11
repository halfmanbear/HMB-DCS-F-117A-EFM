// Pilot input for the DLIR sensor: gimbal slew (keys and axes), optical zoom, power and designation.
#include "stdafx.h"
#include "IradsInternal.h"
#include "../../Inputs.h"
#include <cmath>

namespace Cockpit_interop
{
static constexpr float axis_deadzone = 0.05f;
static constexpr int plane_eos_on_off =
    87;  // iCommandPlaneEOSOnOff; iCommandTVSensor is undefined in DCS input profiles
static constexpr int slew_horizontal_axis_legacy = 20110;
static constexpr int slew_vertical_axis_legacy = 20111;
static constexpr int zoom_axis_legacy = 20112;
static constexpr int slew_horizontal_axis_prior = 2110;
static constexpr int slew_vertical_axis_prior = 2111;
static constexpr int zoom_axis_prior = 2112;

// Optical zoom presets and the slew-rate scaling that follows the displayed field of view.
static constexpr double zoom_preset[] = {1.4, 10.0, 80.0};
static constexpr double min_zoom = 1.4;
static constexpr double max_zoom = 80.0;
static constexpr double slew_rate_at_min_zoom = 60.0;

// Zoom axis detents with hysteresis: wide below -0.4, narrow above 0.4, middle inside +/-0.25.
static constexpr float zoom_axis_low = -0.4f;
static constexpr float zoom_axis_high = 0.4f;
static constexpr float zoom_axis_middle = 0.25f;

// Slew commands can arrive continuously, so only the first of each load are logged.
static constexpr unsigned input_log_limit = 128;
static constexpr unsigned axis_log_limit = 48;
static constexpr unsigned slew_log_limit = 48;
static constexpr float axis_log_threshold = 0.05f;

static void set_external_designation(bool enabled)
{
    if (g_av_tv_sensor_instance && g_av_tv_sensor_set_external_designation)
    {
        g_av_tv_sensor_set_external_designation(g_av_tv_sensor_instance, enabled ? 1 : 0, 0);
    }
}

static bool is_logged_command(int command)
{
    return (command >= 10008 && command <= 10121) ||
           (command >= slew_horizontal_axis_prior && command <= zoom_axis_prior) ||
           (command >= slew_horizontal_axis_legacy && command <= zoom_axis_legacy) || command == plane_eos_on_off;
}

static void log_command(int command, float value)
{
    if (is_logged_command(command))
    {
        static unsigned int logged_inputs = 0;
        if (logged_inputs++ < input_log_limit)
            log_format("IRADS input: command=%d value=%.4f sensor=%p", command, value, g_av_tv_sensor_instance);
    }
    if (command >= 10119 && command <= 10121 && std::isfinite(value) && std::abs(value) > axis_log_threshold)
    {
        static unsigned int axis_logs = 0;
        if (axis_logs++ < axis_log_limit)
            log_format("IRADS routed axis: command=%d value=%.4f", command, value);
    }
}

static bool is_slew_command(int command)
{
    return command == irads_slew_left || command == irads_slew_right || command == irads_slew_up ||
           command == irads_slew_down || command == irads_slew_horizontal_axis || command == irads_slew_vertical_axis ||
           command == slew_horizontal_axis_prior || command == slew_vertical_axis_prior ||
           command == slew_horizontal_axis_legacy || command == slew_vertical_axis_legacy;
}

// Match angular rate to displayed FOV: wide can traverse quickly, narrow retains precision.
// Native velocity uses its zoom-table index, which does not track our explicit 1.4/10/80
// optical zoom values.
static double slew_rate_scale()
{
    double zoom = as_sensor(g_av_tv_sensor_instance)->current_zoom;
    if (!std::isfinite(zoom))
        zoom = min_zoom;
    zoom = (zoom < min_zoom) ? min_zoom : ((zoom > max_zoom) ? max_zoom : zoom);
    return slew_rate_at_min_zoom * (min_zoom / zoom);
}

static bool apply_horizontal()
{
    if (s_irads_horizontal_axis < -axis_deadzone && g_av_tv_sensor_slew_left)
    {
        g_av_tv_sensor_slew_left(g_av_tv_sensor_instance,
                                 -static_cast<double>(s_irads_horizontal_axis) * slew_rate_scale());
        return true;
    }
    if (s_irads_horizontal_axis > axis_deadzone && g_av_tv_sensor_slew_right)
    {
        g_av_tv_sensor_slew_right(g_av_tv_sensor_instance,
                                  static_cast<double>(s_irads_horizontal_axis) * slew_rate_scale());
        return true;
    }
    return false;
}

static bool apply_vertical()
{
    if (s_irads_vertical_axis > axis_deadzone && g_av_tv_sensor_slew_up)
    {
        g_av_tv_sensor_slew_up(g_av_tv_sensor_instance, static_cast<double>(s_irads_vertical_axis) * slew_rate_scale());
        return true;
    }
    if (s_irads_vertical_axis < -axis_deadzone && g_av_tv_sensor_slew_down)
    {
        g_av_tv_sensor_slew_down(g_av_tv_sensor_instance,
                                 -static_cast<double>(s_irads_vertical_axis) * slew_rate_scale());
        return true;
    }
    return false;
}

// Stopping slew stops BOTH gimbal motors and rebases slew state, so after stopping one axis
// re-apply the other.
static bool stop_and_resume(bool resume_vertical)
{
    if (!g_av_tv_sensor_slew_stop)
        return false;
    g_av_tv_sensor_slew_stop(g_av_tv_sensor_instance);
    if (resume_vertical)
        apply_vertical();
    else
        apply_horizontal();
    return true;
}

static bool dispatch_horizontal()
{
    if (apply_horizontal())
        return true;
    if (stop_and_resume(true))
        return true;
    if (g_av_tv_sensor_move_horizontal_abs)
        return g_av_tv_sensor_move_horizontal_abs(g_av_tv_sensor_instance, s_irads_horizontal_axis, axis_deadzone,
                                                  static_cast<double>(s_irads_horizontal_axis));
    return false;
}

static bool dispatch_vertical()
{
    if (apply_vertical())
        return true;
    if (stop_and_resume(false))
        return true;
    if (g_av_tv_sensor_move_vertical_abs)
        return g_av_tv_sensor_move_vertical_abs(g_av_tv_sensor_instance, s_irads_vertical_axis, axis_deadzone,
                                                static_cast<double>(s_irads_vertical_axis));
    return false;
}

static void set_zoom_level(int level)
{
    s_irads_zoom_level = level;
    auto* sensor = as_sensor(g_av_tv_sensor_instance);
    sensor->current_zoom = zoom_preset[level - 1];
    sensor->target_zoom = sensor->current_zoom;
    // Reapply a held direction immediately at the new optical rate.
    apply_horizontal();
    apply_vertical();
    log_format("IRADS optical zoom: level=%d zoom=%.2f fullFovDeg=%.3f", level, sensor->current_zoom,
               sensor->base_fov / sensor->current_zoom * rad_to_deg);
}

// Hysteresis avoids repeated zoom steps at the detent boundaries.
static int zoom_level_for_axis(float value)
{
    int level = s_irads_zoom_level;
    if (value < zoom_axis_low)
        level = 1;
    else if (value > zoom_axis_high)
        level = 3;
    else if (value > -zoom_axis_middle && value < zoom_axis_middle)
        level = 2;
    return level;
}

static void clear_slew_demand()
{
    s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
    s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
    s_irads_horizontal_axis = 0.0f;
    s_irads_vertical_axis = 0.0f;
}

static void toggle_power()
{
    s_irads_power = !s_irads_power;
    clear_slew_demand();
    clear_irads_lock();
    if (g_av_tv_sensor_slew_stop)
        g_av_tv_sensor_slew_stop(g_av_tv_sensor_instance);
    log_format("IRADS power: %s", s_irads_power ? "ON" : "OFF");
}

// A direct axis (stick or throttle) sets the analog demand; a held key overrides it.
static void set_analog_axis(float& analog, float& keyboard, float& axis, bool active, float value)
{
    analog = active ? value : 0.0f;
    axis = keyboard != 0.0f ? keyboard : analog;
}

// A slew key sets the keyboard demand to +/-magnitude while held and releases back to the
// analog demand. `direction` is -1 for left/down and +1 for right/up.
static void set_slew_key(float& analog, float& keyboard, float& axis, float direction, bool active, float magnitude)
{
    if (active)
        keyboard = direction * magnitude;
    else if (keyboard * direction > 0.0f)
        keyboard = 0.0f;
    axis = keyboard != 0.0f ? keyboard : analog;
}

static void log_slew_key(const char* name, float value, float axis)
{
    static unsigned int slew_logs = 0;
    if (slew_logs++ < slew_log_limit)
    {
        const int caged = *(reinterpret_cast<uint8_t*>(g_av_tv_sensor_instance) + sensor_caged_byte_offset);
        log_format("HandleIRADSCommand: %s value=%.2f axis=%.2f caged=%d", name, value, axis, caged);
    }
}

// Slew stop doubles as the designation button: pressing it queues a terrain designation.
static void stop_slew_and_designate(bool active)
{
    clear_slew_demand();
    if (g_av_tv_sensor_slew_stop)
        g_av_tv_sensor_slew_stop(g_av_tv_sensor_instance);
    if (active)
    {
        clear_irads_lock();
        const bool queued = ground_stabilize_irads();
        log_message(queued ? "IRADS terrain designation queued" : "IRADS designation unavailable: no rendered ray");
    }
}

bool handle_irads_command(int command, float value)
{
    log_command(command, value);
    if (!g_av_tv_sensor_instance)
        return false;

    if (!std::isfinite(value))
        return false;
    value = (value < -1.0f) ? -1.0f : ((value > 1.0f) ? 1.0f : value);
    const bool active = value > axis_deadzone || value < -axis_deadzone;
    const float axis_magnitude = (value < 0.0f) ? -value : value;

    if (!s_irads_power && command != plane_eos_on_off)
        return false;

    if (is_slew_command(command) && active && (s_irads_lock_valid || s_irads_terrain_pending))
        clear_irads_lock();

    switch (command)
    {
        case plane_eos_on_off:
            toggle_power();
            return true;
        case slew_horizontal_axis_legacy:
        case slew_horizontal_axis_prior:
        case irads_slew_horizontal_axis:
            set_analog_axis(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, active,
                            value);
            dispatch_horizontal();
            return true;
        case slew_vertical_axis_legacy:
        case slew_vertical_axis_prior:
        case irads_slew_vertical_axis:
            set_analog_axis(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, active, value);
            dispatch_vertical();
            return true;
        case zoom_axis_legacy:
        case zoom_axis_prior:
        case irads_zoom_axis:
            set_zoom_level(zoom_level_for_axis(value));
            return true;
        case irads_slew_left:
            set_slew_key(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, -1.0f, active,
                         axis_magnitude);
            log_slew_key("SlewLeft", value, s_irads_horizontal_axis);
            dispatch_horizontal();
            return true;
        case irads_slew_right:
            set_slew_key(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, 1.0f, active,
                         axis_magnitude);
            log_slew_key("SlewRight", value, s_irads_horizontal_axis);
            dispatch_horizontal();
            return true;
        case irads_slew_up:
            set_slew_key(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, 1.0f, active,
                         axis_magnitude);
            log_slew_key("SlewUp", value, s_irads_vertical_axis);
            dispatch_vertical();
            return true;
        case irads_slew_down:
            set_slew_key(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, -1.0f, active,
                         axis_magnitude);
            log_slew_key("SlewDown", value, s_irads_vertical_axis);
            dispatch_vertical();
            return true;
        case irads_slew_stop:
            stop_slew_and_designate(active);
            return true;
        case tv_sensor_zoom_in:
        case irads_zoom_in:
            // value_down is not set in the keyboard binding so value=0 -> active is always false.
            // Zoom is a momentary press, not an axis, so just call unconditionally.
            log_message("HandleIRADSCommand: ZoomIn");
            set_zoom_level((s_irads_zoom_level < 3) ? s_irads_zoom_level + 1 : 3);
            return true;
        case tv_sensor_zoom_out:
        case irads_zoom_out:
            log_message("HandleIRADSCommand: ZoomOut");
            set_zoom_level((s_irads_zoom_level > 1) ? s_irads_zoom_level - 1 : 1);
            return true;
        case irads_designation_clear:
            clear_irads_lock();
            set_external_designation(false);
            return true;
        default:
            return false;
    }
}

bool is_irads_powered()
{
    return s_irads_power;
}

int get_irads_zoom_level()
{
    return s_irads_zoom_level;
}
}  // namespace Cockpit_interop
