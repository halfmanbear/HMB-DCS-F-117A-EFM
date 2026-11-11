// Lateral axis: parabolic stick-to-roll-rate command with a roll-rate feedback loop.
#include "stdafx.h"
#include "Internal.h"

#include <algorithm>
#include <cmath>

#include "TustinFilter.h"

namespace F117
{
namespace Flight_controls
{

namespace
{
namespace cfg = Roll_controller_config;

Tustin_filter lat_stick_force_filter;
Tustin_filter roll_command_filter;
Tustin_filter roll_actuator_dynamics_filter;
Tustin_filter roll_rate_filter1;
Tustin_filter roll_rate_filter2;

bool roll_controller_initialized = false;
double roll_control_accumulator = 0.0;
double roll_held_output = 0.0;

void init_roll_filters(double dt)
{
    double numerators[2] = {0.0, cfg::lat_force_filter_pole};
    double denominators[2] = {1.0, cfg::lat_force_filter_pole};
    lat_stick_force_filter.configure(numerators, denominators, 1, dt);

    double numerators1[2] = {0.0, cfg::roll_command_filter_pole};
    double denominators1[2] = {1.0, cfg::roll_command_filter_pole};
    roll_command_filter.configure(numerators1, denominators1, 1, dt);

    const double servo_omega_squared = std::pow(cfg::roll_servo_natural_frequency, 2.0);
    double numerators2[3] = {0.0, 0.0, servo_omega_squared};
    double denominators2[3] = {1.0, 2.0 * cfg::roll_servo_damping_ratio * cfg::roll_servo_natural_frequency,
                               servo_omega_squared};
    roll_actuator_dynamics_filter.configure(numerators2, denominators2, 2, dt);

    double numerators3[2] = {0.0, cfg::roll_rate_filter_pole};
    double denominators3[2] = {1.0, cfg::roll_rate_filter_pole};
    roll_rate_filter1.configure(numerators3, denominators3, 1, dt);

    double numerators4[3] = {cfg::roll_rate_filter2_num0, cfg::roll_rate_filter2_num1,
                             cfg::roll_rate_filter2_omega_squared};
    double denominators4[3] = {1.0, cfg::roll_rate_filter2_den1, cfg::roll_rate_filter2_omega_squared};
    roll_rate_filter2.configure(numerators4, denominators4, 2, dt);

    roll_control_accumulator = 0.0;
    roll_held_output = 0.0;
    Detail::debug_snapshot.roll = Detail::Roll_snapshot{};
    roll_controller_initialized = true;
}

// Feel gain that eases the roll command as the pilot pulls or pushes on the pitch axis.
double roll_feel_gain(double long_stick_force_command)
{
    const double long_stick_force_gained = long_stick_force_command * cfg::long_stick_feel_gain;
    double roll_feel_gain = 0.0;
    if (std::abs(long_stick_force_command) > cfg::roll_feel_force_threshold)
    {
        roll_feel_gain = cfg::roll_feel_fixed_gain;
    }
    else if (long_stick_force_command >= 0.0)
    {
        roll_feel_gain = -cfg::roll_feel_slope * long_stick_force_gained + 1.0;
    }
    else if (long_stick_force_command < 0.0)
    {
        roll_feel_gain = cfg::roll_feel_slope * long_stick_force_gained + 1.0;
    }
    return roll_feel_gain;
}

// Parabolic stick-force-to-roll-rate shaping.
// rollRateCmd = sign * K * (|force| - deadband)^2, clamped to +/-kRollRateMax.
// This matches the real F-117's non-linear parabolic shaping: gentle onset at centre
// (good precision) with rapidly increasing authority toward full deflection (good jinking).
double parabolic_roll_rate_command(double lat_stick_force_final, double dyn_pressure_pa)
{
    double roll_rate_command = 0.0;
    if (std::abs(lat_stick_force_final) > cfg::stick_deadband)
    {
        const double sign = (lat_stick_force_final > 0.0) ? 1.0 : -1.0;
        const double force_beyond_deadband = std::abs(lat_stick_force_final) - cfg::stick_deadband;
        const double speed_fraction =
            limit((dyn_pressure_pa - cfg::roll_rate_qbar_low) / (cfg::roll_rate_qbar_high - cfg::roll_rate_qbar_low),
                  0.0, 1.0);
        const double roll_rate_max = cfg::roll_rate_max_low_speed +
                                     speed_fraction * (cfg::roll_rate_max_high_speed - cfg::roll_rate_max_low_speed);
        const double parabolic_gain = roll_rate_max / (cfg::roll_usable_force * cfg::roll_usable_force);
        roll_rate_command = sign * parabolic_gain * force_beyond_deadband * force_beyond_deadband;
        roll_rate_command = limit(roll_rate_command, -roll_rate_max, roll_rate_max);
    }
    return roll_rate_command;
}

// Flat-turn interconnect: with the rudder held and the stick near centre, aileron resists the
// inside wing dropping. Fades out on the ground.
double flat_turn_roll_command(const Roll_inputs& in, double effective_lateral_stick)
{
    const double flat_turn_pedal_blend = limit(std::abs(in.ped_input) / cfg::flat_turn_pedal_full, 0.0, 1.0);
    const double flat_turn_stick_blend =
        1.0 - limit(std::abs(effective_lateral_stick) / cfg::flat_turn_lat_stick_fade, 0.0, 1.0);
    const double flat_turn_blend = flat_turn_pedal_blend * flat_turn_stick_blend;
    return (1.0 - Detail::ground_contact_fade) * flat_turn_blend *
           limit(in.beta_deg * cfg::flat_turn_beta_gain - in.roll_angle_deg * cfg::flat_turn_bank_gain,
                 -cfg::flat_turn_command_limit, cfg::flat_turn_command_limit);
}

double fcs_roll_controller_step(const Roll_inputs& in, bool reset_filters)
{
    // Air refuelling: lateral stick gain reduced for control harmony with reduced pitch gain.
    const double effective_lateral_stick = in.air_refuel_door_open
                                               ? in.lat_stick_input_command * cfg::aar_lateral_stick_gain_scale
                                               : in.lat_stick_input_command;

    const double lat_stick_force_cmd = effective_lateral_stick * cfg::lat_stick_force_scale;
    const double lat_stick_force = lat_stick_force_filter.step(lat_stick_force_cmd, reset_filters);
    const double ay_bias_blend = limit(std::abs(effective_lateral_stick) / cfg::ay_bias_full_stick, 0.0, 1.0);
    const double lat_stick_force_biased = lat_stick_force - (in.ay * cfg::ay_bias_gain * ay_bias_blend);

    const double lat_stick_force_final{lat_stick_force_biased * roll_feel_gain(in.long_stick_force_command)};
    const double roll_rate_command = parabolic_roll_rate_command(lat_stick_force_final, in.dyn_pressure_pa);
    const double flat_turn_roll_cmd = flat_turn_roll_command(in, effective_lateral_stick);

    const double roll_rate_command_total = roll_rate_command + flat_turn_roll_cmd;
    const double roll_rate_command_filtered = roll_command_filter.step(roll_rate_command_total, reset_filters);
    const double roll_rate_filtered1 = roll_rate_filter1.step(in.roll_rate, reset_filters);
    const double roll_rate_filtered2 = roll_rate_filter2.step(roll_rate_filtered1, reset_filters);
    const double roll_rate_command_with_trim = roll_rate_command_filtered + in.roll_rate_trim;

    // Positive surface rolls left. Feedback sets how the rate error dies away; the feed-forward holds
    // the surface that balances roll damping at the commanded rate, so the rate doesn't droop.
    double roll_loop_gain =
        (std::min)(cfg::roll_loop_bandwidth / (std::max)(in.roll_accel_per_surface_deg, 1.0), cfg::roll_loop_gain_max);

    // Gear down: increase roll rate feedback gain to reduce response to turbulence on approach.
    if (in.gear_down)
    {
        roll_loop_gain *= cfg::gear_down_rate_feedback_gain_factor;
    }

    const double roll_feed_forward = limit(in.surface_per_roll_rate, 0.0, cfg::roll_feed_forward_max);
    // On the ground the roll rate is ignored, which leaves the stick's path to the elevons unchanged.
    const double roll_rate_feedback = roll_rate_filtered2 * (1.0 - Detail::ground_contact_fade);
    const double roll_command_gained = limit((roll_rate_feedback - roll_rate_command_with_trim) * roll_loop_gain -
                                                 roll_rate_command_with_trim * roll_feed_forward,
                                             -cfg::roll_command_limit, cfg::roll_command_limit);
    const double roll_surface_command = roll_actuator_dynamics_filter.step(roll_command_gained, reset_filters);

    Detail::Roll_snapshot& snapshot = Detail::debug_snapshot.roll;
    snapshot.stick_input = effective_lateral_stick;
    snapshot.stick_force = lat_stick_force_final;
    snapshot.rate_command = roll_rate_command;
    snapshot.flat_turn_command = flat_turn_roll_cmd;
    snapshot.rate_command_filtered = roll_rate_command_filtered;
    snapshot.command_gained = roll_command_gained;
    snapshot.surface_command = roll_surface_command;

    return roll_surface_command;
}
}  // namespace

namespace Detail
{

void reset_roll_state()
{
    roll_controller_initialized = false;
    roll_control_accumulator = 0.0;
    roll_held_output = 0.0;
}

}  // namespace Detail

double fcs_roll_controller(const Roll_inputs& in, double dt)
{
    const bool needs_init = !roll_controller_initialized;
    if (needs_init)
    {
        init_roll_filters(fixed_control_time_step);
    }

    Detail::advance_control_accumulator(roll_control_accumulator, dt);
    if (needs_init && roll_control_accumulator < fixed_control_time_step)
    {
        roll_control_accumulator = fixed_control_time_step;
    }

    bool reset_filters = needs_init;
    while (roll_control_accumulator >= fixed_control_time_step)
    {
        roll_held_output = fcs_roll_controller_step(in, reset_filters);
        reset_filters = false;
        roll_control_accumulator -= fixed_control_time_step;
    }

    return roll_held_output;
}

}  // namespace Flight_controls
}  // namespace F117
