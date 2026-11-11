#include "stdafx.h"
#include "CommandHandlers.h"

#include <algorithm>
#include <cmath>
#include "../Cockpit/ParamBridge.h"
#include "../Damage/Damage.h"
#include "../FlightControls/FlightControls.h"
#include "../FlightModel/State.h"
#include "../Actuators/Actuators.h"
#include "../Systems/Gear.h"
#include "../Weapons/ReleaseSequence.h"
#include "../Inputs.h"

using namespace F117;
using namespace weapons;

bool handle_roll_command(int command, float value)
{
    if (command == trimLeft || command == trimRight)
    {
        F117::rollTrim += command == trimLeft ? kRollTrimStep : -kRollTrimStep;
        return true;
    }
    double demand;
    if (command == JoystickRoll)
        demand = limit(value, -1.0, 1.0);
    else if (command == RollLeft || command == RollRight)
        demand = (-value + (command == RollLeft ? -0.025 : 0.025)) / 2.0 * 100.0;
    else if (command == RollLeftStop || command == RollRightStop)
    {
        F117::FLIGHTCONTROLS::latStickInput = 0;
        return true;
    }
    else
        return false;
    F117::roll_cmd = demand;
    F117::FLIGHTCONTROLS::latStickInput = demand;
    return true;
}

bool handle_pitch_command(int command, float value)
{
    if (command == trimUp || command == trimDown)
    {
        F117::pitchTrim += command == trimUp ? -kPitchTrimStep : kPitchTrimStep;
        return true;
    }
    double demand;
    if (command == JoystickPitch)
        demand = limit(-value, -1.0, 1.0);
    else if (command == PitchUp || command == PitchDown)
        demand = command == PitchUp ? -1.0 : 1.0;
    else if (command == PitchUpStop || command == PitchDownStop)
        demand = 0.0;
    else
        return false;
    F117::FLIGHTCONTROLS::longStickInput = demand;
    return true;
}

bool handle_yaw_command(int command, float value)
{
    if (command == NwsAuthorityOn || command == NwsAuthorityOff)
    {
        F117::nwsAuthorityHeld = command == NwsAuthorityOn;
        return true;
    }
    if (command == ruddertrimLeft || command == ruddertrimRight)
    {
        F117::yawTrim += command == ruddertrimLeft ? kYawTrimStep : -kYawTrimStep;
        return true;
    }
    double demand;
    if (command == JoystickYaw)
        demand = -value * (501.0 / (beta_DEG * beta_DEG + 500.0));
    else if (command == rudderleft || command == rudderright)
    {
        const double magnitude = value + 101.0 / (beta_DEG * beta_DEG + 100.0);
        demand = command == rudderleft ? magnitude : -magnitude;
    }
    else if (command == rudderleftend || command == rudderrightend)
        demand = 0.0;
    else
        return false;
    F117::pedInput = limit(demand, -1.0, 1.0);
    return true;
}

bool handle_engine_and_throttle_command(int command, float value)
{
    using namespace F117::ENGINE_START;
    if (command == EnginesOff || command == LeftEngineOff || command == RightEngineOff)
    {
        if (command != RightEngineOff)
            g_engineStarts.request_stop(LEFT);
        if (command != LeftEngineOff)
            g_engineStarts.request_stop(RIGHT);
        F117::engineswitch = g_engineStarts.any_running();
        if (!F117::engineswitch)
            F117::throttleInput = 0.0;
        return true;
    }
    if (command == EnginesOn)
    {
        const double cores[] = {F117::engines[LEFT].N2, F117::engines[RIGHT].N2};
        g_engineStarts.request_both(cores, external_start_air());
        return true;
    }
    if (command == LeftEngineOn || command == RightEngineOn)
    {
        const int side = command == LeftEngineOn ? LEFT : RIGHT;
        g_engineStarts.request_start(side, F117::engines[side].N2, external_start_air());
        return true;
    }
    if (command == JoystickThrottle)
    {
        if (F117::engineswitch)
        {
            const double percent = limit(((-value + 1.0) * 0.5) * 100.0, 0.0, 100.0);
            if (g_autothrottle.accept_axis(percent))
                F117::throttleInput = percent;
        }
        return true;
    }
    if (command != ThrottleIncrease && command != ThrottleDecrease)
        return false;
    if (F117::engineswitch && (command == ThrottleDecrease || F117::internal_fuel >= 5.0))
    {
        g_autothrottle.pilot_throttle_step();
        const double step = command == ThrottleIncrease ? kThrottleStep : -kThrottleStep;
        F117::throttleInput = limit(F117::throttleInput + step, 0.0, 100.0);
    }
    return true;
}

bool handle_actuator_toggle_command(int command)
{
    switch (command)
    {
        case tailhook:
            if (F117::ACTUATORS::tailhook.position < kToggleLowThreshold)
            {
                F117::tailhook_command = 1.0;
            }
            else if (F117::ACTUATORS::tailhook.position > kToggleHighThreshold)
            {
                F117::tailhook_command = 0.0;
            }
            return true;
        case dragChute:
            if (F117::ACTUATORS::dragChute.position < kToggleLowThreshold)
            {
                F117::dragchute_command = 1.0;
            }
            else if (F117::ACTUATORS::dragChute.position > kToggleHighThreshold)
            {
                F117::dragchute_command = 0.0;
            }
            printf("Drag chute = %f \n", dragchute_command);
            return true;
        case geardown:
            set_gear_command(kGearCommandDown);
            return true;
        case gearup:
            set_gear_command(kGearCommandUp);
            return true;
        case geartoggle:
            if (F117::ACTUATORS::gear.position > kGearToggleThreshold)
            {
                set_gear_command(kGearCommandUp);
            }
            else if (F117::ACTUATORS::gear.position < kGearToggleThreshold)
            {
                set_gear_command(kGearCommandDown);
            }
            return true;
        case WheelBrakeOn:
            F117::rolling_friction = kWheelBrakeOnFriction;
            return true;
        case WheelBrakeOff:
            F117::rolling_friction = kWheelBrakeOffFriction;
            return true;
        case AirRefuel:
            F117::airRefuelDoorOpen = !F117::airRefuelDoorOpen;
            return true;
        case bombay:
            reset_weapon_release_sequence();
            toggle_all_weapon_bays();
            return true;
        case bombayOpen:
            reset_weapon_release_sequence();
            for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
                command_weapon_bay_open(bay);
            return true;
        case bombayClose:
            reset_weapon_release_sequence();
            for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
                command_weapon_bay_close(bay);
            return true;
        case bombaySelected:
            reset_weapon_release_sequence();
            toggle_weapon_bay(selected_weapon_bay());
            return true;
        case bombayLeft:
            reset_weapon_release_sequence();
            toggle_weapon_bay(0);
            return true;
        case bombayRight:
            reset_weapon_release_sequence();
            toggle_weapon_bay(1);
            return true;
        case pickleOn:
        case weaponReleaseHoldOn:
            handle_weapon_release_pressed();
            return true;
        case pickleOff:
        case weaponReleaseHoldOff:
            handle_weapon_release_released();
            return true;
        case MasterArmToggle:
            F117::master_armed = !F117::master_armed;
            reset_weapon_release_sequence();  // require fresh consent after changing arm state
            return true;
        case IRADSLaserArmToggle:
            F117::laser_armed = !F117::laser_armed;
            return true;
        case luaDamageHit:
            apply_lua_damage_hit();
            return true;
    }

    return false;
}

bool handle_autopilot_command(int command)
{
    int* selected = nullptr;
    if (command == autopilot_alt)
        selected = &alt_hold;
    if (command == autopilot_horiz)
        selected = &horiz_hold;
    if (command == autopilot_alt_roll)
        selected = &altroll_hold;
    if (!selected && command != autopilot_reset)
        return false;

    const bool engage = selected && *selected == 0;
    alt_hold = horiz_hold = altroll_hold = 0;
    if (engage)
        *selected = 1;
    if (command == autopilot_reset)
    {
        g_autothrottle.disconnect_button();
        g_paars.disengage();
        g_paarsEngagedAutothrottle = false;
    }
    return true;
}

// Either engine running.
bool engines_running()
{
    return F117::engineswitch && (F117::internal_fuel >= 5.0);
}

bool handle_autothrottle_command(int command)
{
    switch (command)
    {
        case AutothrottleEngage:  // A/T engage switch
            g_paarsEngagedAutothrottle = false;
            if (g_autothrottle.engaged())
            {
                g_autothrottle.disengage();
            }
            else
            {
                g_autothrottle.engage(F117::calibratedAirspeed_KT, F117::throttleInput, !F117::weight_on_wheels,
                                      engines_running());
            }
            return true;
        case AutothrottleDisconnect:  // inboard throttle disengage button
            g_paarsEngagedAutothrottle = false;
            g_autothrottle.disconnect_button();
            return true;
    }

    return false;
}

// ARS deselected or not functional: a pilot-engaged A/T stays engaged (manual 4-6);
// the A/T PAARS engaged to move the power levers is released.
void release_paars()
{
    g_paars.disengage();
    if (g_paarsEngagedAutothrottle)
    {
        g_autothrottle.disengage();
        g_paarsEngagedAutothrottle = false;
    }
}

bool handle_paars_command(int command)
{
    if (command != PaarsToggle)  // stick ARS switch
    {
        return false;
    }
    if (g_paars.engaged())
    {
        release_paars();
    }
    else if (g_paars.engage(F117::pitch_angle, F117::roll_angle, F117::weight_on_wheels, F117::airRefuelDoorOpen) &&
             !g_autothrottle.engaged())
    {
        g_paarsEngagedAutothrottle = g_autothrottle.engage(F117::calibratedAirspeed_KT, F117::throttleInput,
                                                           !F117::weight_on_wheels, engines_running());
    }
    return true;
}

void update_paars()
{
    Paars::Command command{};
    const bool active = g_paars.update(F117::pitch_angle, F117::roll_angle, F117::rollRate_RPS * F117::radiansToDegrees,
                                       F117::pitchRate_RPS * F117::radiansToDegrees, F117::gearDown > 0.5,
                                       F117::weight_on_wheels, F117::airRefuelDoorOpen, &command);
    if (!active && g_paarsEngagedAutothrottle)
    {
        release_paars();
    }
    F117::FLIGHTCONTROLS::set_recovery_nz_command(active, command.nz);
    g_paarsRollRateDegS = active ? command.rollRateDegS : 0.0;

    double pulloutFt = 0.0;
    if (active)
    {
        const double speed = sqrt(vx_world * vx_world + vy_world * vy_world + vz_world * vz_world);
        const double flightPathDeg =
            speed > 1.0 ? asin(limit(vy_world / speed, -1.0, 1.0)) * F117::radiansToDegrees : 0.0;
        pulloutFt = Paars::displayed_pullout_altitude_ft(Paars::minimum_pullout_altitude_ft(
            F117::altitude_m * 3.28084, F117::totalVelocity_MPS, flightPathDeg, F117::roll_angle));
    }
    static void* s_activeParam = nullptr;
    static void* s_pulloutParam = nullptr;
    publish_named_param(s_activeParam, "F117_ARS_ACTIVE", active ? 1.0 : 0.0);
    publish_named_param(s_pulloutParam, "F117_ARS_MIN_ALT", pulloutFt);
}

void update_autothrottle(double dt)
{
    F117::calibratedAirspeed_KT = Autothrottle::calibrated_airspeed_kt(
        F117::totalVelocity_MPS, F117::ambientTemperature_DegK, F117::ambientPressure_PA);

    // Air data for the cockpit displays. With wind set, DCS's own true airspeed reads ground speed and its
    // indicated airspeed reads high, so the HUD and gauges use the airspeed this model flies on instead.
    static void* s_airDataValidParam = nullptr;
    static void* s_casParam = nullptr;
    static void* s_tasParam = nullptr;
    const bool airDataValid = std::isfinite(F117::calibratedAirspeed_KT) && std::isfinite(F117::totalVelocity_MPS);
    publish_named_param(s_casParam, "F117_CAS_MPS", airDataValid ? F117::calibratedAirspeed_KT / 1.943844 : 0.0);
    publish_named_param(s_tasParam, "F117_TAS_MPS", airDataValid ? F117::totalVelocity_MPS : 0.0);
    publish_named_param(s_airDataValidParam, "F117_AIR_DATA_VALID", airDataValid ? 1.0 : 0.0);
    F117::throttleInput = g_autothrottle.update(dt, F117::calibratedAirspeed_KT, F117::throttleInput,
                                                !F117::weight_on_wheels, engines_running());

    static void* s_engagedParam = nullptr;
    static void* s_discParam = nullptr;
    static void* s_casCommandParam = nullptr;
    publish_named_param(s_engagedParam, "F117_AT_ENGAGED", g_autothrottle.engaged() ? 1.0 : 0.0);
    publish_named_param(s_discParam, "F117_AT_DISC", g_autothrottle.disconnect_light() ? 1.0 : 0.0);
    publish_named_param(s_casCommandParam, "F117_AT_CAS_COMMAND", g_autothrottle.target_cas_kt());
}
