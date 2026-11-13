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
// Elevon degrees of trim or autopilot bias that add 1.0 to the elevator actuator input.
constexpr double pitch_trim_stick_scale = 15.0;

// Pilot pitch trim, TO 1F-117A-1 page 1-46. Autotrim mode (gear up, A/R closed): trim biases the 1 g
// reference by up to +1.0 g nose-up and -0.4 g nose-down, slow at neutral and faster with more trim;
// end to end takes 20 to 25 s. These rates give about 23 s.
constexpr double autotrim_nose_up_nz = 1.0;
constexpr double autotrim_nose_down_nz = 0.4;
constexpr double autotrim_rate_at_neutral = 0.03;  // g/s
constexpr double autotrim_rate_per_g = 0.1;        // extra g/s per g of trim already applied
// Autotrim off: a conventional elevon trim, end to end in 6 to 9 s. The manual gives no angle.
constexpr double manual_trim_travel_s = 7.5;
constexpr double manual_trim_elevon_deg = 10.0;   // elevon at full trim, either way
constexpr double manual_trim_elevon_rate = 10.0;  // deg/s, eases the bias in and out as the mode changes

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
// costs no lift. The pitch rate the hands-off 1 g needs at this attitude isn't damped, so rolling
// inverted or into a bank doesn't unload while the integrator catches up.
double pitch_damper_elevator_deg()
{
    const double speed = (std::max)(total_velocity_mps, min_control_speed_mps);
    const double hands_off_rate = Flight_controls::one_g_pitch_rate_rps(roll_angle / radians_to_degrees,
                                                                        pitch_angle / radians_to_degrees, speed);
    const double added_cm = pitch_damper_cmq * mean_chord_m / (2.0 * speed) * (pitch_rate_rps - hands_off_rate);
    return added_cm / (std::min)(Aero::cm_per_elevator_deg, -1e-4);
}

// Pilot trim in g while autotrim is active: position +1 is full nose-up, -1 full nose-down.
double autotrim_nz(double position)
{
    return position >= 0.0 ? position * autotrim_nose_up_nz : position * autotrim_nose_down_nz;
}

// Moves the pilot's pitch trim at the manual's rate while the switch is held (TO 1F-117A-1 page 1-46),
// and eases the elevon bias that stands for it when autotrim is off.
void update_pitch_trim(double dt, bool autotrim)
{
    if (pitch_trim_direction != 0)
    {
        if (autotrim)
        {
            double nz = autotrim_nz(pitch_trim_position);
            nz += pitch_trim_direction * (autotrim_rate_at_neutral + autotrim_rate_per_g * std::abs(nz)) * dt;
            nz = limit(nz, -autotrim_nose_down_nz, autotrim_nose_up_nz);
            pitch_trim_position = nz >= 0.0 ? nz / autotrim_nose_up_nz : nz / autotrim_nose_down_nz;
        }
        else
        {
            pitch_trim_position =
                limit(pitch_trim_position + pitch_trim_direction * 2.0 / manual_trim_travel_s * dt, -1.0, 1.0);
        }
    }
    // Nose-up trim is trailing edge up, which is negative elevon.
    const double elevon_target = autotrim ? 0.0 : -pitch_trim_position * manual_trim_elevon_deg;
    const double step = manual_trim_elevon_rate * dt;
    pitch_trim_elevon_deg += limit(elevon_target - pitch_trim_elevon_deg, -step, step);
}

void command_pitch_axis(double dt, double control_degradation)
{
    Flight_controls::update_pitch_mode_auto_inputs(gear_down, air_refuel_door_open);
    const bool autotrim = Flight_controls::pitch_autotrim_active();
    update_pitch_trim(dt, autotrim);
    // Suspension contact only, with the gear down. The strut draw args in weight_on_wheels aren't used here.
    Flight_controls::update_ground_contact(
        gear_down > 0.5 && (suspension_wow[0] || suspension_wow[1] || suspension_wow[2]), dt);

    Flight_controls::Pitch_inputs pitch_in{};
    pitch_in.long_stick_input_command = Flight_controls::long_stick_input * control_degradation;
    pitch_in.trim_nz = autotrim ? autotrim_nz(pitch_trim_position) : 0.0;
    pitch_in.angle_of_attack_ind = alpha_deg;
    pitch_in.pitch_rate_deg_s = pitch_rate_rps * radians_to_degrees;
    pitch_in.az = az / standard_gravity_mps2;
    pitch_in.dyn_pressure_pa = dynamic_pressure_pa;
    pitch_in.roll_angle_deg = roll_angle;
    pitch_in.pitch_angle_deg = pitch_angle;
    pitch_in.roll_rate_deg_s = roll_rate_rps * radians_to_degrees;
    pitch_in.velocity_mps = total_velocity_mps;
    pitch_in.mach = mach;
    pitch_in.thrust_n = thrust_n;
    pitch_in.cx_total = Aero::cx_total;
    elevator_deg_commanded = -Flight_controls::fcs_pitch_controller(pitch_in, dt) * pitch_law_elevon_scale();
    elevator_deg_commanded += pitch_damper_elevator_deg();
    elevator_deg = limit(elevator_deg_commanded + autopilot_pitch_bias_deg + pitch_trim_elevon_deg,
                         elevator_nose_up_limit_deg, elevator_nose_down_limit_deg);
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

    Flight_controls::Roll_inputs roll_in{};
    roll_in.lat_stick_input_command = Flight_controls::lat_stick_input * control_degradation;
    roll_in.long_stick_force_command = Flight_controls::long_stick_force;
    roll_in.ay = ay / standard_gravity_mps2;
    roll_in.ped_input = ped_input;
    roll_in.beta_deg = beta_deg;
    roll_in.roll_angle_deg = roll_angle;
    roll_in.roll_rate = roll_rate_rps * radians_to_degrees;
    roll_in.roll_rate_trim = g_paars_roll_rate_deg_s;  // PAARS roll-rate command, added to the pilot's
    roll_in.dyn_pressure_pa = dynamic_pressure_pa;
    roll_in.roll_accel_per_surface_deg = roll_accel_per_surface_deg;
    roll_in.surface_per_roll_rate = surface_per_roll_rate;
    roll_in.gear_down = gear_down > 0.5;
    roll_in.air_refuel_door_open = air_refuel_door_open;
    aileron_deg_commanded = Flight_controls::fcs_roll_controller(roll_in, dt);

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

    Flight_controls::Yaw_inputs yaw_in{};
    yaw_in.ped_input = ped_input;
    yaw_in.ped_trim = 0.0;
    yaw_in.beta_deg = beta_deg;
    yaw_in.yaw_rate = yaw_rate_rps * radians_to_degrees;
    yaw_in.turn_yaw_rate = level_turn_yaw_rate_degps();
    yaw_in.roll_rate = roll_rate_rps * radians_to_degrees;
    yaw_in.pitch_rate = pitch_rate_rps * radians_to_degrees;
    yaw_in.aoa_filtered = Flight_controls::alpha_filtered;
    yaw_in.dyn_pressure_pa = dynamic_pressure_pa;
    yaw_in.gear_down = gear_down > 0.5;
    rudder_deg_commanded = Flight_controls::fcs_yaw_controller(yaw_in, dt);
    rudder_deg = yaw_trim + rudder_deg_commanded;
    rudder_deg = limit(rudder_deg, -rudder_limit_deg / rudder_limiter_scale, rudder_limit_deg / rudder_limiter_scale);
}
}  // namespace

void update_autopilot_hold_state()
{
    if (g_damage.cockpit <= autopilot_cockpit_integrity)
    {
        alt_hold = horiz_hold = altroll_hold = 0;
        autopilot_pitch_bias_deg = 0.0;
        return;
    }
    if (horiz_hold)
        autopilot_pitch_bias_deg = horiz_hold_pitch_gain * pitch_angle;
    else if (alt_hold || altroll_hold)
        autopilot_pitch_bias_deg = limit(vspeed, -vertical_speed_hold_limit_mps, vertical_speed_hold_limit_mps);
    else
        autopilot_pitch_bias_deg = 0.0;
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
        Actuators::elevator,
        Flight_controls::long_stick_input + (autopilot_pitch_bias_deg + pitch_trim_elevon_deg) / pitch_trim_stick_scale,
        dt));

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
