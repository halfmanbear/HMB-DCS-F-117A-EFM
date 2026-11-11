#include "stdafx.h"
#include "ControlLaws.h"

#include <algorithm>
#include <cmath>

#include "../Actuators/Actuators.h"
#include "../Aero/AeroAssembly.h"
#include "../Aerodynamics/AeroModel.h"
#include "../Damage/Damage.h"
#include "../EfmMath.h"
#include "../FlightControls/FlightControls.h"
#include "../FlightModel/State.h"
#include "../Systems/Gear.h"

namespace F117
{

namespace
{
constexpr double pitch_law_cm_per_deg = 0.0105;  // Elevon pitch power the pitch law's gains were tuned with.
constexpr double pitch_damper_cmq = -5.0;        // Pitch damping the flight controls add, per unit of q c / 2V.

// Pitch authority: -25 deg TEU (nose-up) to +37.5 deg TED (nose-down), per Loschke. The physical actuator
// stop is 45 deg; pitch authority does not reach it.
constexpr double elevator_nose_up_limit_deg = -25.0;
constexpr double elevator_nose_down_limit_deg = 37.5;
constexpr double elevon_physical_stop_deg = 45.0;

constexpr double rudder_limit_deg = 25.0;  // Lets full pedal reach 8 deg of steady sideslip (TO 1F-117A-1 page 6-3).
constexpr double min_control_speed_mps = 30.0;    // Floor for speed in the damper and turn-rate terms.
constexpr double default_roll_inertia = 40000.0;  // kg m^2, when DCS has not yet reported one
constexpr double max_pitch_law_elevon_scale = 8.0;
constexpr double pitch_trim_stick_scale = 15.0;  // pitch trim units that add 1.0 to the elevator actuator input

// Rudder limiter: the high-beta, high-yaw-rate rudder limit stays symmetric left vs right.
constexpr double rudder_limiter_beta_scale = 7500.0;
constexpr double rudder_limiter_yaw_rate_scale_degps = 90.0;

// Autopilot holds: cockpit integrity below which they disengage, and the hold gains.
constexpr double autopilot_cockpit_integrity = 0.7;
constexpr double horiz_hold_pitch_gain = 2.0;
constexpr double horiz_hold_roll_gain = 0.1;
constexpr double vertical_speed_hold_limit_mps = 10.0;

// The pitch law's gains are in degrees of an elevon giving kPitchLawCmPerDeg of pitching moment. This
// turns its command into the deflection that gives the same moment on this airframe at this AOA, as the
// roll law does with its rolling moment.
double pitch_law_elevon_scale()
{
    return limit(pitch_law_cm_per_deg / (std::max)(std::abs(Aero::cm_per_elevator_deg), 1e-4), 1.0,
                 max_pitch_law_elevon_scale);
}

// Pitch damper. The tailless airframe has little natural pitch damping (Cmq about -2.2). The manual
// describes the flight controls as giving essentially dead-beat damping, so the elevons add more.
// Nose-up pitch rate gives trailing edge down. In a steady pull the pitch law's integrator takes up
// whatever the damper adds, so the elevon there is set by the airframe's own moments and the damper
// costs no lift.
double pitch_damper_elevator_deg()
{
    const double speed = (std::max)(total_velocity_mps, min_control_speed_mps);
    const double added_cm = pitch_damper_cmq * mean_chord_m / (2.0 * speed) * pitch_rate_rps;
    return added_cm / (std::min)(Aero::cm_per_elevator_deg, -1e-4);
}

void command_pitch_axis(double dt, double control_degradation)
{
    Flight_controls::update_pitch_mode_auto_inputs(gear_down, air_refuel_door_open);
    // Suspension contact only, with the gear down. The strut draw args in weight_on_wheels aren't used here.
    Flight_controls::update_ground_contact(
        gear_down > 0.5 && (suspension_wow[0] || suspension_wow[1] || suspension_wow[2]), dt);

    elevator_deg_commanded =
        -(Flight_controls::fcs_pitch_controller(
            Flight_controls::long_stick_input * control_degradation, 0.0, alpha_deg,
            pitch_rate_rps * radians_to_degrees, (az / standard_gravity_mps2), 0.0, dynamic_pressure_pa, dt, roll_angle,
            pitch_angle, roll_rate_rps * radians_to_degrees, total_velocity_mps, mach, thrust_n, Aero::cx_total)) *
        pitch_law_elevon_scale();
    elevator_deg_commanded += pitch_damper_elevator_deg();
    elevator_deg = limit(elevator_deg_commanded + pitch_trim, elevator_nose_up_limit_deg, elevator_nose_down_limit_deg);
}

void command_roll_axis(double dt, double control_degradation)
{
    // What the roll law needs to know about the airframe at this flight condition. It uses the inertia DCS is
    // flying, so changing moment_of_inertia in the mod's FM/config.lua doesn't change the loop's bandwidth.
    const double roll_moment_per_surface_deg = (std::max)(std::abs(Aero::cl_per_aileron_deg), 0.0002);
    const double roll_inertia = inertia.x > 1.0 ? inertia.x : default_roll_inertia;
    const double roll_accel_per_surface_deg = roll_moment_per_surface_deg * dynamic_pressure_pa * wing_area_m2 *
                                              wing_span_m / roll_inertia * radians_to_degrees;
    const double surface_per_roll_rate = std::abs(Aero::clp) * wing_span_m /
                                         (2.0 * (std::max)(total_velocity_mps, min_control_speed_mps)) /
                                         radians_to_degrees / roll_moment_per_surface_deg;

    aileron_deg_commanded = Flight_controls::fcs_roll_controller(
        Flight_controls::lat_stick_input * control_degradation, Flight_controls::long_stick_force,
        ay / standard_gravity_mps2, ped_input, beta_deg, roll_angle, roll_rate_rps * radians_to_degrees,
        g_paars_roll_rate_deg_s,  // PAARS roll-rate command, added to the pilot's
        dynamic_pressure_pa, roll_accel_per_surface_deg, surface_per_roll_rate, gear_down > 0.5, air_refuel_door_open,
        dt);

    // Real F-117 pitch/roll mixer: pitch has priority.
    // Each elevon physical stop is 45 deg. Pitch occupies elevator_DEG of that travel.
    // Roll gets whatever remains: max roll deflection = 45 deg - |elevator_DEG|.
    // This means rolling pull-outs automatically have less roll authority at high pitch demand,
    // exactly as the real aircraft (and consistent with Loschke's mixer description).
    const double max_roll_deg = (std::max)(0.0, elevon_physical_stop_deg - std::abs(elevator_deg));
    aileron_deg = limit(aileron_deg_commanded + roll_trim, -max_roll_deg, max_roll_deg);
}

// Body yaw rate of a steady level turn at this bank and pitch: g sin(bank) cos(pitch) / V.
double level_turn_yaw_rate_degps()
{
    return standard_gravity_mps2 * std::sin(roll_angle / radians_to_degrees) *
           std::cos(pitch_angle / radians_to_degrees) / (std::max)(total_velocity_mps, min_control_speed_mps) *
           radians_to_degrees;
}

void command_yaw_axis(double dt)
{
    // Keep the high-beta/high-yaw-rate rudder limiter symmetric left vs right.
    const double beta_magnitude_deg = std::abs(beta_deg);
    const double yaw_rate_magnitude_deg_s = std::abs(yaw_rate_rps * radians_to_degrees);
    const float rudder_limiter_scale =
        static_cast<float>(1.0 + (beta_magnitude_deg * beta_magnitude_deg) / rudder_limiter_beta_scale *
                                     (1.0 + yaw_rate_magnitude_deg_s / rudder_limiter_yaw_rate_scale_degps));

    rudder_deg_commanded = Flight_controls::fcs_yaw_controller(
        ped_input, 0.0, beta_deg, yaw_rate_rps * radians_to_degrees, level_turn_yaw_rate_degps(),
        roll_rate_rps * radians_to_degrees, pitch_rate_rps * radians_to_degrees, Flight_controls::alpha_filtered,
        dynamic_pressure_pa, gear_down > 0.5, dt);
    rudder_deg = yaw_trim + rudder_deg_commanded;
    rudder_deg = limit(rudder_deg, -rudder_limit_deg / rudder_limiter_scale, rudder_limit_deg / rudder_limiter_scale);
}
}  // namespace

void update_autopilot_hold_state()
{
    if (g_damage.cockpit <= autopilot_cockpit_integrity)
    {
        alt_hold = horiz_hold = altroll_hold = 0;
        return;
    }
    if (horiz_hold)
        pitch_trim = horiz_hold_pitch_gain * pitch_angle;
    else if (alt_hold || altroll_hold)
        pitch_trim = limit(vspeed, -vertical_speed_hold_limit_mps, vertical_speed_hold_limit_mps);
    if (horiz_hold || altroll_hold)
        roll_trim = horiz_hold_roll_gain * roll_angle;
}

void update_flight_control_commands(double dt, double control_degradation)
{
    command_pitch_axis(dt, control_degradation);
    command_roll_axis(dt, control_degradation);

    dragchute = Actuators::drive_drag_chute(dragchute_command, dt, total_velocity_mps, weight_on_wheels);

    command_yaw_axis(dt);
}

double update_airframe_actuators(double dt)
{
    elev_pos = static_cast<float>(Actuators::drive(
        Actuators::elevator, Flight_controls::long_stick_input + (pitch_trim / pitch_trim_stick_scale), dt));

    const double tail_integrity = (g_damage.left_tail + g_damage.right_tail) / 2.0;
    rudder_pos = static_cast<float>(Actuators::drive(Actuators::rudder, ped_input, dt) * tail_integrity);

    const double nose_wheel_range_target = nws_authority_held ? nose_wheel_yaw_high_rad : nose_wheel_yaw_low_rad;
    const double nose_wheel_range_step = nose_wheel_yaw_range_rate_radps * dt;
    nose_wheel_yaw_range_rad +=
        limit(nose_wheel_range_target - nose_wheel_yaw_range_rad, -nose_wheel_range_step, nose_wheel_range_step);

    gear_command = clamp_gear_command_for_weight_on_wheels(gear_command);
    gear_down = Actuators::drive_gear(gear_command, dt, gear_retraction_blocked());

    tailhook_pos = Actuators::drive(Actuators::tailhook, tailhook_command, dt);
    for (int bay = 0; bay < weapon_bay_count; ++bay)
        bay_state[bay] = static_cast<float>(Actuators::drive(Actuators::weapon_bay[bay], bay_cmd[bay], dt));

    return tail_integrity;
}

}  // namespace F117
