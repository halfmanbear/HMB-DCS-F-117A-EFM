// Directional axis: Beta-command augmentation (proportional only).
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
namespace cfg = Yaw_controller_config;

Tustin_filter rudder_command_filter;
Tustin_filter yaw_rate_filter;
Tustin_filter yaw_servo_filter;

bool yaw_controller_initialized = false;
double yaw_control_accumulator = 0.0;
double yaw_held_output = 0.0;
double auto_beta_trim_state = 0.0;   // automatic Beta trim accumulator (deg)
double auto_beta_trim_wait_s = 0.0;  // time left before the automatic Beta trim may act

struct Yaw_inputs
{
    double ped_input;
    double ped_trim;
    double beta_deg;
    double yaw_rate;
    double turn_yaw_rate;
    double roll_rate;
    double pitch_rate;
    double aoa_filtered;
    double dyn_pressure_pa;
    bool gear_down;
};

void init_yaw_filters(double dt)
{
    // Beta command low-pass filter
    double numerators[2] = {0.0, cfg::command_filter_rate};
    double denominators[2] = {1.0, cfg::command_filter_rate};
    rudder_command_filter.configure(numerators, denominators, 1, dt);

    // Yaw rate lead-lag shaping
    double numerators2[2] = {cfg::yaw_lead_lag_numerator0, cfg::yaw_lead_lag_pole};
    double denominators2[2] = {1.0, cfg::yaw_lead_lag_pole};
    yaw_rate_filter.configure(numerators2, denominators2, 1, dt);

    // Fin servo dynamics
    const double servo_omega_squared = std::pow(cfg::servo_natural_frequency, 2.0);
    double numerators3[3] = {0.0, 0.0, servo_omega_squared};
    double denominators3[3] = {1.0, 2.0 * cfg::servo_damping_ratio * cfg::servo_natural_frequency, servo_omega_squared};
    yaw_servo_filter.configure(numerators3, denominators3, 2, dt);

    yaw_control_accumulator = 0.0;
    yaw_held_output = 0.0;
    auto_beta_trim_state = 0.0;
    auto_beta_trim_wait_s = 0.0;
    Detail::debug_snapshot.yaw = Detail::Yaw_snapshot{};
    Detail::debug_snapshot.aero = Detail::Aero_snapshot{};
    yaw_controller_initialized = true;
}

// Real F-117 directional axis: Beta-command augmentation (proportional only).
// Pedal position commands a target sideslip angle (Beta). The fins are driven to
// satisfy that command. Beta feedback gain is AoA-scheduled (doubles at high AoA to
// resist the destabilising pitching moment from Beta). Automatic Beta trim bleeds
// residual Beta when gear is up and pedal force is below threshold. Inertia coupling
// (pitch_rate x roll_rate) prevents directional divergence during high-rate manoeuvres.
// The ARI (aileron-rudder interconnect) present in early design was removed from the
// real aircraft after evaluation - it is NOT implemented here.
double fcs_yaw_controller_step(const Yaw_inputs& in, double dt, bool reset_filters)
{
    const double ped_input = in.ped_input;
    const double beta_deg = in.beta_deg;
    const double aoa_filtered = in.aoa_filtered;

    // 1. Pedal -> Beta command (with small deadband)
    double raw_pedal_cmd = 0.0;
    if (std::abs(ped_input) > cfg::beta_pedal_deadband)
    {
        const double sign = (ped_input > 0.0) ? 1.0 : -1.0;
        raw_pedal_cmd = sign * (std::abs(ped_input) - cfg::beta_pedal_deadband) / (1.0 - cfg::beta_pedal_deadband) *
                        cfg::beta_command_gain;
    }

    // Dynamic pressure schedule: limit Beta command at high speed
    // 0 at and below kBetaCmdQbarLow, 1 at and above kBetaCmdQbarHigh.
    const double qbar_fraction = limit(
        (in.dyn_pressure_pa - cfg::beta_cmd_qbar_low) / (cfg::beta_cmd_qbar_high - cfg::beta_cmd_qbar_low), 0.0, 1.0);
    const double beta_cmd_limit =
        cfg::beta_cmd_limit_high + qbar_fraction * (cfg::beta_cmd_limit_low - cfg::beta_cmd_limit_high);
    const double beta_cmd_limited = limit(raw_pedal_cmd, -beta_cmd_limit, beta_cmd_limit);

    // Low-pass filter the Beta command to smooth pedal transients
    const double beta_cmd_filtered = rudder_command_filter.step(beta_cmd_limited, reset_filters);

    // 2. Automatic Beta trim (series trim): active when gear is up and pedal near centre.
    // Slowly trims away residual Beta (e.g. from engine thrust asymmetry).
    const bool pedal_centred = std::abs(ped_input) < cfg::auto_beta_trim_pedal_threshold;
    auto_beta_trim_wait_s = pedal_centred ? (std::max)(auto_beta_trim_wait_s - dt, 0.0) : cfg::auto_beta_trim_delay_s;
    if (!in.gear_down && pedal_centred && auto_beta_trim_wait_s <= 0.0)
    {
        const double beta_excess =
            beta_deg - limit(beta_deg, -cfg::auto_beta_trim_deadband, cfg::auto_beta_trim_deadband);
        auto_beta_trim_state -= cfg::auto_beta_trim_rate * beta_excess * dt;
        auto_beta_trim_state = limit(auto_beta_trim_state, -cfg::auto_beta_trim_limit, cfg::auto_beta_trim_limit);
    }

    const double beta_command_total = beta_cmd_filtered + in.ped_trim + auto_beta_trim_state;

    // 3. Beta feedback with AoA-scheduled gain.
    // Gain doubles linearly from base at 0 deg AoA to 2x base at kBetaGainAoADoubleAlpha. Negative AOA counts too:
    // the wing's dihedral effect follows its lift, so it reverses there and destabilises as much.
    const double aoa_fraction = limit(std::abs(aoa_filtered) / cfg::beta_gain_ao_a_double_alpha, 0.0, 1.0);
    const double beta_gain = cfg::beta_feedback_gain_base * (1.0 + aoa_fraction);
    const double beta_error = beta_command_total - beta_deg;
    const double beta_feedback = beta_gain * beta_error;

    // 4. Stability-axis yaw rate damping (stability axis correction: subtract roll x alpha coupling).
    // The yaw rate a steady banked turn needs is taken out first, so the damper never opposes a turn and
    // needs no washout. A washout leaves a tail after every yaw transient that pulls the sideslip back
    // towards zero, which overshot pedal steps by 20 to 50%.
    const double alpha_rad = aoa_filtered * (pi / 180.0);
    const double yaw_rate_stability_axis = in.yaw_rate - in.roll_rate * alpha_rad - in.turn_yaw_rate;
    const double yaw_rate_smoothed = yaw_rate_filter.step(yaw_rate_stability_axis, reset_filters);
    const double yaw_damping_gain =
        cfg::yaw_damping_gain_low_speed +
        qbar_fraction * (cfg::yaw_damping_gain_high_speed - cfg::yaw_damping_gain_low_speed);
    const double yaw_damping = yaw_damping_gain * yaw_rate_smoothed;

    // 5. Inertia coupling: pitch_rate x roll_rate to prevent inertia-coupled yaw divergence.
    const double pitch_rate_rad_s = in.pitch_rate * (pi / 180.0);
    const double roll_rate_rad_s = in.roll_rate * (pi / 180.0);
    const double inertia_coupling = cfg::inertia_coupling_gain * pitch_rate_rad_s * roll_rate_rad_s;

    // 6. Combine and pass through fin servo dynamics
    const double combined_command = beta_feedback + yaw_damping + inertia_coupling;
    const double yaw_surface_command = yaw_servo_filter.step(combined_command, reset_filters);

    Detail::Yaw_snapshot& snapshot = Detail::debug_snapshot.yaw;
    snapshot.ped_input = ped_input;
    snapshot.rate_deg_s = in.yaw_rate;
    snapshot.rudder_command = raw_pedal_cmd;
    snapshot.rudder_command_filtered = beta_cmd_filtered;
    snapshot.pedal_command = beta_command_total;
    snapshot.damping = yaw_damping;
    snapshot.side_accel_feedback = inertia_coupling;  // re-used log slot
    snapshot.ari_command = 0.0;                       // ARI removed from real aircraft
    snapshot.combined_command = combined_command;
    snapshot.surface_command = yaw_surface_command;

    return yaw_surface_command;
}
}  // namespace

namespace Detail
{

void reset_yaw_state()
{
    yaw_controller_initialized = false;
    yaw_control_accumulator = 0.0;
    yaw_held_output = 0.0;
    auto_beta_trim_state = 0.0;
    auto_beta_trim_wait_s = 0.0;
}

}  // namespace Detail

double fcs_yaw_controller(double ped_input, double ped_trim, double beta_deg, double yaw_rate, double turn_yaw_rate,
                          double roll_rate, double pitch_rate, double aoa_filtered, double dyn_pressure_pa,
                          bool gear_down, double dt)
{
    const bool needs_init = !yaw_controller_initialized;
    if (needs_init)
    {
        init_yaw_filters(fixed_control_time_step);
    }

    Detail::advance_control_accumulator(yaw_control_accumulator, dt);
    if (needs_init && yaw_control_accumulator < fixed_control_time_step)
    {
        yaw_control_accumulator = fixed_control_time_step;
    }

    const Yaw_inputs inputs = {ped_input, ped_trim,   beta_deg,     yaw_rate,        turn_yaw_rate,
                               roll_rate, pitch_rate, aoa_filtered, dyn_pressure_pa, gear_down};
    bool reset_filters = needs_init;
    while (yaw_control_accumulator >= fixed_control_time_step)
    {
        yaw_held_output = fcs_yaw_controller_step(inputs, fixed_control_time_step, reset_filters);
        reset_filters = false;
        yaw_control_accumulator -= fixed_control_time_step;
    }

    return yaw_held_output;
}

}  // namespace Flight_controls
}  // namespace F117
