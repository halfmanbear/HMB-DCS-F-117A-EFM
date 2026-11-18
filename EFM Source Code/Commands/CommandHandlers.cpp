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

bool handle_roll_command(int command, float value)
{
    if (command == trim_left || command == trim_right)
    {
        F117::roll_trim += command == trim_left ? roll_trim_step : -roll_trim_step;
        return true;
    }
    double demand;
    if (command == joystick_roll)
        demand = limit(value, -1.0, 1.0);
    else if (command == roll_left || command == roll_right)
        demand = (-value + (command == roll_left ? -0.025 : 0.025)) / 2.0 * 100.0;
    else if (command == roll_left_stop || command == roll_right_stop)
    {
        F117::Flight_controls::lat_stick_input = 0;
        return true;
    }
    else
        return false;
    F117::roll_cmd = demand;
    F117::Flight_controls::lat_stick_input = demand;
    return true;
}

bool handle_pitch_command(int command, float value)
{
    // DCS repeats trim up/down every frame while the switch is held, then sends trim_stop. The control laws
    // move the trim at the manual's rate while a direction is held.
    if (command == trim_up || command == trim_down)
    {
        F117::pitch_trim_direction = command == trim_up ? 1 : -1;
        return true;
    }
    if (command == trim_stop)
    {
        F117::pitch_trim_direction = 0;
        return true;
    }
    double demand;
    if (command == joystick_pitch)
        demand = limit(-value, -1.0, 1.0);
    else if (command == pitch_up || command == pitch_down)
        demand = command == pitch_up ? -1.0 : 1.0;
    else if (command == pitch_up_stop || command == pitch_down_stop)
        demand = 0.0;
    else
        return false;
    F117::Flight_controls::long_stick_input = demand;
    return true;
}

bool handle_yaw_command(int command, float value)
{
    if (command == nws_authority_on || command == nws_authority_off)
    {
        F117::nws_authority_held = command == nws_authority_on;
        return true;
    }
    if (command == ruddertrim_left || command == ruddertrim_right)
    {
        F117::yaw_trim += command == ruddertrim_left ? yaw_trim_step : -yaw_trim_step;
        return true;
    }
    double demand;
    if (command == joystick_yaw)
        demand = -value * (501.0 / (F117::beta_deg * F117::beta_deg + 500.0));
    else if (command == rudderleft || command == rudderright)
    {
        const double magnitude = value + 101.0 / (F117::beta_deg * F117::beta_deg + 100.0);
        demand = command == rudderleft ? magnitude : -magnitude;
    }
    else if (command == rudderleftend || command == rudderrightend)
        demand = 0.0;
    else
        return false;
    F117::ped_input = limit(demand, -1.0, 1.0);
    return true;
}

// The combined lever (A/T reference, cockpit lever) is the mean of the two engine inputs.
static void sync_throttle_lever()
{
    F117::throttle_input = 0.5 * (F117::throttle_side_input[0] + F117::throttle_side_input[1]);
}

void set_throttle_levers(double percent)
{
    F117::throttle_side_input[0] = F117::throttle_side_input[1] = percent;
    sync_throttle_lever();
}

bool handle_engine_and_throttle_command(int command, float value)
{
    if (command == engines_off || command == left_engine_off || command == right_engine_off)
    {
        if (command != right_engine_off)
            g_engine_starts.request_stop(F117::Engine_start::left);
        if (command != left_engine_off)
            g_engine_starts.request_stop(F117::Engine_start::right);
        F117::engineswitch = g_engine_starts.any_running();
        if (!F117::engineswitch)
            set_throttle_levers(0.0);
        return true;
    }
    if (command == engines_on)
    {
        const double cores[] = {F117::engines[F117::Engine_start::left].n2,
                                F117::engines[F117::Engine_start::right].n2};
        g_engine_starts.request_both(cores, external_start_air());
        return true;
    }
    if (command == left_engine_on || command == right_engine_on)
    {
        const int side = command == left_engine_on ? F117::Engine_start::left : F117::Engine_start::right;
        g_engine_starts.request_start(side, F117::engines[side].n2, external_start_air());
        return true;
    }
    if (command == joystick_throttle || command == throttle_axis_left || command == throttle_axis_right)
    {
        if (F117::engineswitch)
        {
            const double percent = limit(((-value + 1.0) * 0.5) * 100.0, 0.0, 100.0);
            const int channel = command == joystick_throttle ? 0 : (command == throttle_axis_left ? 1 : 2);
            if (g_autothrottle.accept_axis(percent, channel))
            {
                if (channel != 2)
                    F117::throttle_side_input[F117::Engine_start::left] = percent;
                if (channel != 1)
                    F117::throttle_side_input[F117::Engine_start::right] = percent;
                sync_throttle_lever();
            }
        }
        return true;
    }

    // Common steps move both levers by the same amount; left and right steps move one.
    bool step_left = true;
    bool step_right = true;
    double direction = 0.0;
    switch (command)
    {
        case throttle_increase:
            direction = 1.0;
            break;
        case throttle_decrease:
            direction = -1.0;
            break;
        case throttle_left_increase:
            direction = 1.0;
            step_right = false;
            break;
        case throttle_left_decrease:
            direction = -1.0;
            step_right = false;
            break;
        case throttle_right_increase:
            direction = 1.0;
            step_left = false;
            break;
        case throttle_right_decrease:
            direction = -1.0;
            step_left = false;
            break;
        default:
            return false;
    }
    if (F117::engineswitch && (direction < 0.0 || F117::internal_fuel >= 5.0))
    {
        g_autothrottle.pilot_throttle_step();
        const bool step_side[2] = {step_left, step_right};
        for (int side = 0; side < 2; ++side)
        {
            if (step_side[side])
                F117::throttle_side_input[side] =
                    limit(F117::throttle_side_input[side] + direction * throttle_step, 0.0, 100.0);
        }
        sync_throttle_lever();
    }
    return true;
}

bool handle_actuator_toggle_command(int command)
{
    switch (command)
    {
        case tailhook:
            if (F117::Actuators::tailhook.position < toggle_low_threshold)
            {
                F117::tailhook_command = 1.0;
            }
            else if (F117::Actuators::tailhook.position > toggle_high_threshold)
            {
                F117::tailhook_command = 0.0;
            }
            return true;
        case drag_chute:
            if (F117::Actuators::drag_chute.position < toggle_low_threshold)
            {
                F117::dragchute_command = 1.0;
            }
            else if (F117::Actuators::drag_chute.position > toggle_high_threshold)
            {
                F117::dragchute_command = 0.0;
            }
            return true;
        case geardown:
            set_gear_command(gear_command_down);
            return true;
        case gearup:
            set_gear_command(gear_command_up);
            return true;
        case geartoggle:
            if (F117::Actuators::gear.position > gear_toggle_threshold)
            {
                set_gear_command(gear_command_up);
            }
            else if (F117::Actuators::gear.position < gear_toggle_threshold)
            {
                set_gear_command(gear_command_down);
            }
            return true;
        case wheel_brake_on:
            F117::rolling_friction = wheel_brake_on_friction;
            return true;
        case wheel_brake_off:
            F117::rolling_friction = wheel_brake_off_friction;
            return true;
        case air_refuel:
            F117::air_refuel_door_open = !F117::air_refuel_door_open;
            return true;
        case bombay:
            Weapons::reset_weapon_release_sequence();
            Weapons::toggle_all_weapon_bays();
            return true;
        case bombay_open:
            Weapons::reset_weapon_release_sequence();
            for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
                Weapons::command_weapon_bay_open(bay);
            return true;
        case bombay_close:
            Weapons::reset_weapon_release_sequence();
            for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
                Weapons::command_weapon_bay_close(bay);
            return true;
        case bombay_selected:
            Weapons::reset_weapon_release_sequence();
            Weapons::toggle_weapon_bay(Weapons::selected_weapon_bay());
            return true;
        case bombay_left:
            Weapons::reset_weapon_release_sequence();
            Weapons::toggle_weapon_bay(0);
            return true;
        case bombay_right:
            Weapons::reset_weapon_release_sequence();
            Weapons::toggle_weapon_bay(1);
            return true;
        case pickle_on:
        case weapon_release_hold_on:
            Weapons::handle_weapon_release_pressed();
            return true;
        case pickle_off:
        case weapon_release_hold_off:
            Weapons::handle_weapon_release_released();
            return true;
        case ahcp_master_arm:
        case ahcp_master_safe:
        {
            const bool arm = command == ahcp_master_arm;
            if (F117::master_armed != arm)
            {
                F117::master_armed = arm;
                Weapons::reset_weapon_release_sequence();  // require fresh consent after changing arm state
            }
            return true;
        }
        case ahcp_laser_arm:
            F117::laser_armed = true;
            return true;
        case ahcp_laser_safe:
            F117::laser_armed = false;
            return true;
        case lua_damage_hit:
            apply_lua_damage_hit();
            return true;
    }

    return false;
}

bool handle_autopilot_command(int command)
{
    int* selected = nullptr;
    if (command == autopilot_alt)
        selected = &F117::alt_hold;
    if (command == autopilot_horiz)
        selected = &F117::horiz_hold;
    if (command == autopilot_alt_roll)
        selected = &F117::altroll_hold;
    if (!selected && command != autopilot_reset)
        return false;

    const bool engage = selected && *selected == 0;
    F117::alt_hold = F117::horiz_hold = F117::altroll_hold = 0;
    if (engage)
        *selected = 1;
    if (command == autopilot_reset)
    {
        g_autothrottle.disconnect_button();
        g_paars.disengage();
        g_paars_engaged_autothrottle = false;
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
        case autothrottle_engage:  // A/T engage switch
            g_paars_engaged_autothrottle = false;
            if (g_autothrottle.engaged())
            {
                g_autothrottle.disengage();
            }
            else
            {
                g_autothrottle.engage(F117::calibrated_airspeed_kt, F117::throttle_input, !F117::weight_on_wheels,
                                      engines_running());
            }
            return true;
        case autothrottle_disconnect:  // inboard throttle disengage button
            g_paars_engaged_autothrottle = false;
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
    if (g_paars_engaged_autothrottle)
    {
        g_autothrottle.disengage();
        g_paars_engaged_autothrottle = false;
    }
}

bool handle_paars_command(int command)
{
    if (command != paars_toggle)  // stick ARS switch
    {
        return false;
    }
    if (g_paars.engaged())
    {
        release_paars();
    }
    else if (g_paars.engage(F117::pitch_angle, F117::roll_angle, F117::weight_on_wheels, F117::air_refuel_door_open) &&
             !g_autothrottle.engaged())
    {
        g_paars_engaged_autothrottle = g_autothrottle.engage(F117::calibrated_airspeed_kt, F117::throttle_input,
                                                             !F117::weight_on_wheels, engines_running());
    }
    return true;
}

void update_paars()
{
    Paars::Command command{};
    Paars::Controller::Flight_state state{};
    state.pitch_deg = F117::pitch_angle;
    state.bank_deg = F117::roll_angle;
    state.roll_rate_deg_s = F117::roll_rate_rps * F117::radians_to_degrees;
    state.pitch_rate_deg_s = F117::pitch_rate_rps * F117::radians_to_degrees;
    state.gear_down = F117::gear_down > 0.5;
    state.weight_on_wheels = F117::weight_on_wheels;
    state.air_refuel_door_open = F117::air_refuel_door_open;
    const bool active = g_paars.update(state, &command);
    if (!active && g_paars_engaged_autothrottle)
    {
        release_paars();
    }
    F117::Flight_controls::set_recovery_nz_command(active, command.nz);
    g_paars_roll_rate_deg_s = active ? command.roll_rate_deg_s : 0.0;

    double pullout_ft = 0.0;
    if (active)
    {
        const double speed = sqrt(vx_world * vx_world + vy_world * vy_world + vz_world * vz_world);
        const double flight_path_deg =
            speed > 1.0 ? asin(limit(vy_world / speed, -1.0, 1.0)) * F117::radians_to_degrees : 0.0;
        pullout_ft = Paars::displayed_pullout_altitude_ft(Paars::minimum_pullout_altitude_ft(
            F117::altitude_m * 3.28084, F117::total_velocity_mps, flight_path_deg, F117::roll_angle));
    }
    static void* s_active_param = nullptr;
    static void* s_pullout_param = nullptr;
    publish_named_param(s_active_param, "F117_ARS_ACTIVE", active ? 1.0 : 0.0);
    publish_named_param(s_pullout_param, "F117_ARS_MIN_ALT", pullout_ft);
}

void update_autothrottle(double dt)
{
    F117::calibrated_airspeed_kt = Autothrottle::calibrated_airspeed_kt(
        F117::total_velocity_mps, F117::ambient_temperature_deg_k, F117::ambient_pressure_pa);

    // Air data for the cockpit displays. With wind set, DCS's own true airspeed reads ground speed and its
    // indicated airspeed reads high, so the HUD and gauges use the airspeed this model flies on instead.
    static void* s_air_data_valid_param = nullptr;
    static void* s_cas_param = nullptr;
    static void* s_tas_param = nullptr;
    const bool air_data_valid = std::isfinite(F117::calibrated_airspeed_kt) && std::isfinite(F117::total_velocity_mps);
    publish_named_param(s_cas_param, "F117_CAS_MPS", air_data_valid ? F117::calibrated_airspeed_kt / 1.943844 : 0.0);
    publish_named_param(s_tas_param, "F117_TAS_MPS", air_data_valid ? F117::total_velocity_mps : 0.0);
    publish_named_param(s_air_data_valid_param, "F117_AIR_DATA_VALID", air_data_valid ? 1.0 : 0.0);
    const double lever = g_autothrottle.update(dt, F117::calibrated_airspeed_kt, F117::throttle_input,
                                               !F117::weight_on_wheels, engines_running());
    // An engaged A/T drives both levers together; otherwise each side keeps the pilot's setting.
    if (g_autothrottle.engaged())
        set_throttle_levers(lever);

    static void* s_engaged_param = nullptr;
    static void* s_disc_param = nullptr;
    static void* s_cas_command_param = nullptr;
    publish_named_param(s_engaged_param, "F117_AT_ENGAGED", g_autothrottle.engaged() ? 1.0 : 0.0);
    publish_named_param(s_disc_param, "F117_AT_DISC", g_autothrottle.disconnect_light() ? 1.0 : 0.0);
    publish_named_param(s_cas_command_param, "F117_AT_CAS_COMMAND", g_autothrottle.target_cas_kt());
}
