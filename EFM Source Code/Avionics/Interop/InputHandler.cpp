// Pilot input for the DLIR sensor: gimbal slew (keys and axes), optical zoom, power and designation.
#include "stdafx.h"
#include "IradsInternal.h"
#include "../../Inputs.h"
#include <cmath>

namespace CockpitInterop
{
static constexpr float kAxisDeadzone = 0.05f;
static constexpr int kPlaneEosOnOff = 87;  // iCommandPlaneEOSOnOff; iCommandTVSensor is undefined in DCS input profiles
static constexpr int kSlewHorizontalAxisLegacy = 20110;
static constexpr int kSlewVerticalAxisLegacy = 20111;
static constexpr int kZoomAxisLegacy = 20112;
static constexpr int kSlewHorizontalAxisPrior = 2110;
static constexpr int kSlewVerticalAxisPrior = 2111;
static constexpr int kZoomAxisPrior = 2112;

// Optical zoom presets and the slew-rate scaling that follows the displayed field of view.
static constexpr double kZoomPreset[] = {1.4, 10.0, 80.0};
static constexpr double kMinZoom = 1.4;
static constexpr double kMaxZoom = 80.0;
static constexpr double kSlewRateAtMinZoom = 60.0;

// Zoom axis detents with hysteresis: wide below -0.4, narrow above 0.4, middle inside +/-0.25.
static constexpr float kZoomAxisLow = -0.4f;
static constexpr float kZoomAxisHigh = 0.4f;
static constexpr float kZoomAxisMiddle = 0.25f;

// Slew commands can arrive continuously, so only the first of each load are logged.
static constexpr unsigned kInputLogLimit = 128;
static constexpr unsigned kAxisLogLimit = 48;
static constexpr unsigned kSlewLogLimit = 48;
static constexpr float kAxisLogThreshold = 0.05f;

static void SetExternalDesignation(bool enabled)
{
    if (g_avTVSensor_instance && g_avTVSensor_set_external_designation)
    {
        g_avTVSensor_set_external_designation(g_avTVSensor_instance, enabled ? 1 : 0, 0);
    }
}

static bool is_logged_command(int command)
{
    return (command >= 10008 && command <= 10121) ||
           (command >= kSlewHorizontalAxisPrior && command <= kZoomAxisPrior) ||
           (command >= kSlewHorizontalAxisLegacy && command <= kZoomAxisLegacy) || command == kPlaneEosOnOff;
}

static void log_command(int command, float value)
{
    if (is_logged_command(command))
    {
        static unsigned int loggedInputs = 0;
        if (loggedInputs++ < kInputLogLimit)
            Log("IRADS input: command=%d value=%.4f sensor=%p", command, value, g_avTVSensor_instance);
    }
    if (command >= 10119 && command <= 10121 && std::isfinite(value) && std::abs(value) > kAxisLogThreshold)
    {
        static unsigned int axisLogs = 0;
        if (axisLogs++ < kAxisLogLimit)
            Log("IRADS routed axis: command=%d value=%.4f", command, value);
    }
}

static bool is_slew_command(int command)
{
    return command == IRADSSlewLeft || command == IRADSSlewRight || command == IRADSSlewUp ||
           command == IRADSSlewDown || command == IRADSSlewHorizontalAxis || command == IRADSSlewVerticalAxis ||
           command == kSlewHorizontalAxisPrior || command == kSlewVerticalAxisPrior ||
           command == kSlewHorizontalAxisLegacy || command == kSlewVerticalAxisLegacy;
}

// Match angular rate to displayed FOV: wide can traverse quickly, narrow retains precision.
// Native velocity uses its zoom-table index, which does not track our explicit 1.4/10/80
// optical zoom values.
static double slew_rate_scale()
{
    double zoom = AsSensor(g_avTVSensor_instance)->currentZoom;
    if (!std::isfinite(zoom))
        zoom = kMinZoom;
    zoom = (zoom < kMinZoom) ? kMinZoom : ((zoom > kMaxZoom) ? kMaxZoom : zoom);
    return kSlewRateAtMinZoom * (kMinZoom / zoom);
}

static bool apply_horizontal()
{
    if (s_irads_horizontal_axis < -kAxisDeadzone && g_avTVSensor_slew_left)
    {
        g_avTVSensor_slew_left(g_avTVSensor_instance,
                               -static_cast<double>(s_irads_horizontal_axis) * slew_rate_scale());
        return true;
    }
    if (s_irads_horizontal_axis > kAxisDeadzone && g_avTVSensor_slew_right)
    {
        g_avTVSensor_slew_right(g_avTVSensor_instance,
                                static_cast<double>(s_irads_horizontal_axis) * slew_rate_scale());
        return true;
    }
    return false;
}

static bool apply_vertical()
{
    if (s_irads_vertical_axis > kAxisDeadzone && g_avTVSensor_slew_up)
    {
        g_avTVSensor_slew_up(g_avTVSensor_instance, static_cast<double>(s_irads_vertical_axis) * slew_rate_scale());
        return true;
    }
    if (s_irads_vertical_axis < -kAxisDeadzone && g_avTVSensor_slew_down)
    {
        g_avTVSensor_slew_down(g_avTVSensor_instance, -static_cast<double>(s_irads_vertical_axis) * slew_rate_scale());
        return true;
    }
    return false;
}

// Stopping slew stops BOTH gimbal motors and rebases slew state, so after stopping one axis
// re-apply the other.
static bool stop_and_resume(bool resumeVertical)
{
    if (!g_avTVSensor_slew_stop)
        return false;
    g_avTVSensor_slew_stop(g_avTVSensor_instance);
    if (resumeVertical)
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
    if (g_avTVSensor_move_horizontal_abs)
        return g_avTVSensor_move_horizontal_abs(g_avTVSensor_instance, s_irads_horizontal_axis, kAxisDeadzone,
                                                static_cast<double>(s_irads_horizontal_axis));
    return false;
}

static bool dispatch_vertical()
{
    if (apply_vertical())
        return true;
    if (stop_and_resume(false))
        return true;
    if (g_avTVSensor_move_vertical_abs)
        return g_avTVSensor_move_vertical_abs(g_avTVSensor_instance, s_irads_vertical_axis, kAxisDeadzone,
                                              static_cast<double>(s_irads_vertical_axis));
    return false;
}

static void set_zoom_level(int level)
{
    s_irads_zoom_level = level;
    auto* sensor = AsSensor(g_avTVSensor_instance);
    sensor->currentZoom = kZoomPreset[level - 1];
    sensor->targetZoom = sensor->currentZoom;
    // Reapply a held direction immediately at the new optical rate.
    apply_horizontal();
    apply_vertical();
    Log("IRADS optical zoom: level=%d zoom=%.2f fullFovDeg=%.3f", level, sensor->currentZoom,
        sensor->baseFov / sensor->currentZoom * kRadToDeg);
}

// Hysteresis avoids repeated zoom steps at the detent boundaries.
static int zoom_level_for_axis(float value)
{
    int level = s_irads_zoom_level;
    if (value < kZoomAxisLow)
        level = 1;
    else if (value > kZoomAxisHigh)
        level = 3;
    else if (value > -kZoomAxisMiddle && value < kZoomAxisMiddle)
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
    ClearIRADSLock();
    if (g_avTVSensor_slew_stop)
        g_avTVSensor_slew_stop(g_avTVSensor_instance);
    Log("IRADS power: %s", s_irads_power ? "ON" : "OFF");
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
    static unsigned int slewLogs = 0;
    if (slewLogs++ < kSlewLogLimit)
    {
        const int caged = *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + kSensorCagedByteOffset);
        Log("HandleIRADSCommand: %s value=%.2f axis=%.2f caged=%d", name, value, axis, caged);
    }
}

// Slew stop doubles as the designation button: pressing it queues a terrain designation.
static void stop_slew_and_designate(bool active)
{
    clear_slew_demand();
    if (g_avTVSensor_slew_stop)
        g_avTVSensor_slew_stop(g_avTVSensor_instance);
    if (active)
    {
        ClearIRADSLock();
        const bool queued = GroundStabilizeIRADS();
        Log_(queued ? "IRADS terrain designation queued" : "IRADS designation unavailable: no rendered ray");
    }
}

bool HandleIRADSCommand(int command, float value)
{
    log_command(command, value);
    if (!g_avTVSensor_instance)
        return false;

    if (!std::isfinite(value))
        return false;
    value = (value < -1.0f) ? -1.0f : ((value > 1.0f) ? 1.0f : value);
    const bool active = value > kAxisDeadzone || value < -kAxisDeadzone;
    const float axisMagnitude = (value < 0.0f) ? -value : value;

    if (!s_irads_power && command != kPlaneEosOnOff)
        return false;

    if (is_slew_command(command) && active && (s_irads_lock_valid || s_irads_terrain_pending))
        ClearIRADSLock();

    switch (command)
    {
        case kPlaneEosOnOff:
            toggle_power();
            return true;
        case kSlewHorizontalAxisLegacy:
        case kSlewHorizontalAxisPrior:
        case IRADSSlewHorizontalAxis:
            set_analog_axis(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, active,
                            value);
            dispatch_horizontal();
            return true;
        case kSlewVerticalAxisLegacy:
        case kSlewVerticalAxisPrior:
        case IRADSSlewVerticalAxis:
            set_analog_axis(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, active, value);
            dispatch_vertical();
            return true;
        case kZoomAxisLegacy:
        case kZoomAxisPrior:
        case IRADSZoomAxis:
            set_zoom_level(zoom_level_for_axis(value));
            return true;
        case IRADSSlewLeft:
            set_slew_key(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, -1.0f, active,
                         axisMagnitude);
            log_slew_key("SlewLeft", value, s_irads_horizontal_axis);
            dispatch_horizontal();
            return true;
        case IRADSSlewRight:
            set_slew_key(s_irads_analog_horizontal, s_irads_keyboard_horizontal, s_irads_horizontal_axis, 1.0f, active,
                         axisMagnitude);
            log_slew_key("SlewRight", value, s_irads_horizontal_axis);
            dispatch_horizontal();
            return true;
        case IRADSSlewUp:
            set_slew_key(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, 1.0f, active,
                         axisMagnitude);
            log_slew_key("SlewUp", value, s_irads_vertical_axis);
            dispatch_vertical();
            return true;
        case IRADSSlewDown:
            set_slew_key(s_irads_analog_vertical, s_irads_keyboard_vertical, s_irads_vertical_axis, -1.0f, active,
                         axisMagnitude);
            log_slew_key("SlewDown", value, s_irads_vertical_axis);
            dispatch_vertical();
            return true;
        case IRADSSlewStop:
            stop_slew_and_designate(active);
            return true;
        case TVSensorZoomIn:
        case IRADSZoomIn:
            // value_down is not set in the keyboard binding so value=0 -> active is always false.
            // Zoom is a momentary press, not an axis, so just call unconditionally.
            Log_("HandleIRADSCommand: ZoomIn");
            set_zoom_level((s_irads_zoom_level < 3) ? s_irads_zoom_level + 1 : 3);
            return true;
        case TVSensorZoomOut:
        case IRADSZoomOut:
            Log_("HandleIRADSCommand: ZoomOut");
            set_zoom_level((s_irads_zoom_level > 1) ? s_irads_zoom_level - 1 : 1);
            return true;
        case IRADSDesignationClear:
            ClearIRADSLock();
            SetExternalDesignation(false);
            return true;
        default:
            return false;
    }
}

bool IsIRADSPowered()
{
    return s_irads_power;
}

int GetIRADSZoomLevel()
{
    return s_irads_zoom_level;
}
}  // namespace CockpitInterop
