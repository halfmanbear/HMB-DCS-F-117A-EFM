// Longitudinal axis: normal-acceleration P+I law with an AOA limiter.
#include "stdafx.h"
#include "Internal.h"
#include "PitchLog.h"

#include <algorithm>
#include <cmath>

#include "TustinFilter.h"

namespace F117
{
namespace Flight_controls
{

namespace
{
namespace cfg = Pitch_controller_config;

Tustin_filter pitch_actuator_dynamics_filter;
Tustin_filter pitch_rate_washout_filter;    // AOA limiter pitch-rate anticipation
Tustin_filter pitch_rate_feedback_washout;  // main-loop pitch-rate blend (P+I feedback)

double stick_command_pos_filtered{};
double az_filtered{};
double nz_reference = 1.0;        // current 1G reference; reset alongside filter state
bool recovery_nz_active = false;  // PAARS load-factor command replaces the 1G reference
double recovery_nz = 1.0;
double pitch_integrator_state = 0.0;  // tracks raw integrator output for state anti-windup

bool pitch_controller_initialized = false;
double pitch_control_accumulator = 0.0;
double pitch_held_output = 0.0;

double pitch_mode_gear_down = 0.0;
bool pitch_mode_air_refuel_door_open = false;

double aoa_limiter_integrator = 0.0;  // P+I AOA limiter integrator state
bool aoa_limiter_active = false;      // Limiter overrode the g command on the last step

struct Pitch_inputs
{
    double long_stick_input_command;
    double pitch_trim;
    double angle_of_attack_ind;
    double pitch_rate_deg_s;
    double az;
    double dyn_pressure_pa;
    double roll_angle_deg;
    double pitch_angle_deg;
    double roll_rate_deg_s;
    double velocity_mps;
    double mach;
    double thrust_n;
    double cx_total;
};

// Everything the P+I law produces for one step; the debug log records the intermediate values.
struct Nz_law_output
{
    double nz_command;
    double nz_error;
    double proportional;
    double control;
    double integrator;
    double pitch_rate_blend;
    double command;
};

void init_pitch_filters(double dt)
{
    const double servo_omega_squared = std::pow(cfg::pitch_servo_natural_frequency, 2.0);
    double numerators[3] = {0.0, 0.0, servo_omega_squared};
    double denominators[3] = {1.0, 2.0 * cfg::pitch_servo_damping_ratio * cfg::pitch_servo_natural_frequency,
                              servo_omega_squared};
    pitch_actuator_dynamics_filter.configure(numerators, denominators, 2, dt);

    // AOA limiter pitch-rate anticipation washout filter (tau = 0.5 s)
    double washout_num[2] = {1.0, 0.0};
    double washout_den[2] = {1.0, cfg::ao_a_anticipation_washout_tau_inverse};
    pitch_rate_washout_filter.configure(washout_num, washout_den, 1, dt);

    // Main-loop pitch-rate feedback blend washout filter (tau = 1.0 s)
    // Separate from the AOA anticipation filter - different time constant.
    double blend_washout_num[2] = {1.0, 0.0};
    double blend_washout_den[2] = {1.0, cfg::pitch_rate_blend_washout_tau_inv};
    pitch_rate_feedback_washout.configure(blend_washout_num, blend_washout_den, 1, dt);

    stick_command_pos_filtered = 0.0;
    az_filtered = 0.0;
    nz_reference = 1.0;
    pitch_integrator_state = 0.0;
    aoa_limiter_integrator = 0.0;
    aoa_limiter_active = false;
    Detail::ground_contact_fade = 0.0;
    pitch_control_accumulator = 0.0;
    pitch_held_output = 0.0;
    pitch_controller_initialized = true;
    Detail::open_pitch_debug_log();
}

// Stick force to scheduled load-factor demand. Choose the sign branch first, then its
// inner/outer segment; the deadband is inclusive.
double load_from_force(double force)
{
    if (std::abs(force) <= cfg::stick_command_deadband)
        return 0.0;
    if (force < 0.0)
        return force <= -cfg::stick_command_breakpoint ? cfg::nz_slope_neg2 * force + cfg::nz_intercept_neg2
                                                       : cfg::nz_slope_neg1 * force + cfg::nz_intercept_neg1;
    return force >= cfg::stick_command_breakpoint ? cfg::nz_slope_pos2 * force + cfg::nz_intercept_pos2
                                                  : cfg::nz_slope_pos1 * force + cfg::nz_intercept_pos1;
}

double fcs_pitch_controller_force_command(double long_stick_input_command, double pitch_trim, double dt)
{
    double long_stick_input_force = 0.0;
    if (long_stick_input_command > 0.0)
    {
        long_stick_input_force = long_stick_input_command * cfg::positive_stick_force_scale + pitch_trim;
    }
    else
    {
        long_stick_input_force = long_stick_input_command * cfg::negative_stick_force_scale;
    }
    long_stick_force = long_stick_input_force =
        limit(long_stick_input_force, cfg::stick_force_min, cfg::stick_force_max);

    const double long_stick_command_with_trim_limited_g =
        limit(pitch_trim - load_from_force(long_stick_force), cfg::nz_command_min, cfg::nz_command_max);
    // Command rate limit. A step of stick would step the elevons, and on a tailless aircraft a sudden up-elevon
    // first loses lift before the nose comes up; ramping large commands keeps that dip and the overshoot after
    // it small. Small corrections pass through almost at once.
    const double max_step = cfg::stick_command_rate_limit * dt;
    stick_command_pos_filtered +=
        limit(long_stick_command_with_trim_limited_g - stick_command_pos_filtered, -max_step, max_step);

    return stick_command_pos_filtered;
}

// The 1G reference follows gravity's component along the load-factor axis while the stick is
// centred, then feeds gravity forward as the aircraft rolls and pitches.
void update_nz_reference(double stick_command_pos, double gravity_component, double roll_rad, double pitch_rad,
                         const Pitch_inputs& in, double dt)
{
    if (std::fabs(stick_command_pos) < cfg::nz_stick_deadband)
    {
        const double gravity_error = gravity_component - nz_reference;
        const double track_rate = (gravity_error > 0.0) ? cfg::nz_reference_snap_rate : cfg::nz_reference_track_rate;
        const double reference_blend = 1.0 - std::exp(-track_rate * dt);
        nz_reference += gravity_error * reference_blend;
    }

    {
        const double roll_rate_rad_s = in.roll_rate_deg_s * (pi / 180.0);
        const double pitch_rate_rad_s = in.pitch_rate_deg_s * (pi / 180.0);
        const double gravity_rate = (-std::sin(roll_rad) * std::cos(pitch_rad) * roll_rate_rad_s) +
                                    (-std::cos(roll_rad) * std::sin(pitch_rad) * pitch_rate_rad_s);
        nz_reference += cfg::gravity_feed_forward_gain * gravity_rate * dt;
        nz_reference = limit(nz_reference, cfg::nz_min, cfg::nz_max);
    }

    // PAARS commands the load factor; pilot stick still adds on top, and on release
    // the reference continues from the last commanded value.
    if (recovery_nz_active)
    {
        nz_reference = limit(recovery_nz, cfg::nz_min, cfg::nz_max);
    }
}

bool gear_is_down()
{
    return pitch_mode_gear_down > cfg::gear_down_threshold;
}

// Integrator with anti-windup. Gear-down and AAR: proportional only (no integrator) - gives
// classic phugoid + short period, and removes the need for WoW switching at touchdown/lift-off.
// AAR: per Loschke, pitch law reverts to proportional-only with reduced stick gain.
void update_pitch_integrator(double nz_measured, double nz_error, double nz_proportional, double dt)
{
    const double integrator_limit = cfg::pitch_integrator_limit;
    double integrator_gain = cfg::pitch_integrator_gain;

    if (gear_is_down() || pitch_mode_air_refuel_door_open)
    {
        // Freeze and bleed integrator to zero; proportional path only
        pitch_integrator_state = pitch_integrator_state * (1.0 - limit(dt * 5.0, 0.0, 1.0));
        return;
    }

    const bool nz_at_upper_limit = (nz_measured >= cfg::nz_max && nz_error > 0.0);
    const bool nz_at_lower_limit = (nz_measured <= cfg::nz_min && nz_error < 0.0);

    const double unsaturated_command_before_integration = nz_proportional + pitch_integrator_state;
    const bool actuator_saturated_high = (unsaturated_command_before_integration >= integrator_limit && nz_error > 0.0);
    const bool actuator_saturated_low = (unsaturated_command_before_integration <= -integrator_limit && nz_error < 0.0);
    // Frozen while the AOA limiter holds the nose, or it stores a pull that carries on after the stick is released.
    const double integrator_input = (nz_at_upper_limit || nz_at_lower_limit || actuator_saturated_high ||
                                     actuator_saturated_low || aoa_limiter_active)
                                        ? 0.0
                                        : nz_error;

    if ((pitch_integrator_state > 0.0 && integrator_input < 0.0) ||
        (pitch_integrator_state < 0.0 && integrator_input > 0.0))
    {
        integrator_gain *= cfg::pitch_integrator_unwind_gain;
    }

    pitch_integrator_state =
        limit(pitch_integrator_state + integrator_gain * dt * integrator_input, -integrator_limit, integrator_limit);
}

// P+I g-command law per Loschke. Pitch-rate blend damps transients via the error signal;
// 1/qbar scaling on the proportional gain automatically scales the blend contribution across
// the envelope. All forward-loop gains are scheduled on inverse dynamic pressure so stick feel
// is consistent across the flight envelope.
Nz_law_output nz_command_law(double nz_cmd, double nz_measured, double dyn_pressure_pa, double pitch_rate_rad_s,
                             double dt, bool reset_filters)
{
    const double qbar_scaled = (std::max)(dyn_pressure_pa, cfg::qbar_min);
    const double qbar_gain_scale =
        limit(cfg::qbar_ref / qbar_scaled, cfg::qbar_gain_scale_min, cfg::qbar_gain_scale_max);
    const double scaled_nz_error_gain = cfg::nz_error_gain * qbar_gain_scale;

    // Washed-out pitch rate blended INTO the Nz feedback before the error is formed.
    // Per Loschke: "a specified blend of washed out pitch rate and the normal acceleration".
    // Washout (tau = 1 s) ensures zero steady-state contribution so the integrator never
    // fights a persistent damping offset in sustained manoeuvres.
    const double pitch_rate_washed_blend = pitch_rate_feedback_washout.step(pitch_rate_rad_s, reset_filters);

    // On the ground the load factor is ignored, which leaves the stick's path to the elevons unchanged.
    const double blended_nz_feedback = nz_measured + Detail::ground_contact_fade * (nz_reference - nz_measured) +
                                       cfg::pitch_rate_blend_gain * pitch_rate_washed_blend;

    Nz_law_output out = {};
    out.nz_command = nz_cmd;
    out.nz_error = nz_cmd - blended_nz_feedback;
    out.proportional = scaled_nz_error_gain * out.nz_error;
    out.control = out.proportional;

    update_pitch_integrator(nz_measured, out.nz_error, out.proportional, dt);

    out.command =
        limit(out.proportional + pitch_integrator_state, -cfg::pitch_integrator_limit, cfg::pitch_integrator_limit);
    out.integrator = pitch_integrator_state;
    out.pitch_rate_blend = cfg::pitch_rate_blend_gain * pitch_rate_washed_blend;
    return out;
}

// ---- Real F-117 AOA limiter: high-gain P+I, activated by AoA + washed-out pitch-rate anticipation ----
// Threshold is Mach-scheduled from flight test limits (falls at high Mach).
// Pitch-rate anticipation is dynamic-pressure-scheduled (stronger at low q for aggressive pull-ups).
// When active the limiter has MORE authority than the pilot.
double apply_aoa_limiter(double pitch_command_total, const Pitch_inputs& in, double dt, bool reset_filters)
{
    const double limiter_threshold = aoa_limit_for_mach(in.mach, gear_is_down());

    // Washed-out pitch rate for anticipation (q-scheduled gain)
    const double pitch_rate_for_anticipation = pitch_rate_washout_filter.step(in.pitch_rate_deg_s, reset_filters);
    const double qbar_for_anticiption = (std::max)(in.dyn_pressure_pa, cfg::qbar_min);
    const double anticipation_gain = cfg::ao_a_anticipation_gain_ref * (cfg::qbar_ref / qbar_for_anticiption);
    const double anticipation =
        limit(anticipation_gain * pitch_rate_for_anticipation, -cfg::ao_a_anticipation_max, cfg::ao_a_anticipation_max);

    const double limiter_input = alpha_filtered + anticipation;
    const double limiter_error = limiter_input - limiter_threshold;

    // The limiter's command keeps rising as AOA falls below the limit, and the lower of it and the pilot's
    // command wins, so the handover is continuous. Switching straight to the pilot's command as soon as AOA
    // dipped under the limit stepped the surface and made a 7 Hz load factor buzz at the limit.
    const double limiter_command = -(cfg::ao_a_limiter_proportional_gain * limiter_error + aoa_limiter_integrator);
    aoa_limiter_active = limiter_command < pitch_command_total;

    if (aoa_limiter_active)
    {
        pitch_command_total = limiter_command;
        // Either sign, because holding the limit can take a little nose-up surface.
        aoa_limiter_integrator = limit(aoa_limiter_integrator + cfg::ao_a_limiter_integrator_gain * dt * limiter_error,
                                       -cfg::ao_a_limiter_integrator_limit, cfg::ao_a_limiter_integrator_limit);
    }
    else
    {
        // Decay integrator when not active so re-entry is smooth
        const double decay = cfg::ao_a_limiter_integrator_gain * dt * 2.0;
        aoa_limiter_integrator = limit(0.0, aoa_limiter_integrator - decay, aoa_limiter_integrator + decay);
    }
    return pitch_command_total;
}

// Roll-rate^2 inertia coupling (pitch axis): prevents inertia-coupled pitch departures during
// high-AoA rolling manoeuvres. Returns the amount to subtract from the pitch command.
double roll_rate_squared_correction(double roll_rate_deg_s)
{
    const double roll_rate_rad_s = roll_rate_deg_s * (pi / 180.0);
    if (alpha_filtered > cfg::roll_rate_squared_ao_a_threshold &&
        std::abs(roll_rate_rad_s) > cfg::roll_rate_squared_rate_threshold)
    {
        const double roll_rate_sq = roll_rate_rad_s * roll_rate_rad_s;
        return cfg::roll_rate_squared_gain * roll_rate_sq;
    }
    return 0.0;
}

// Speed stability (below 200 kts, AoA > 7 deg): apparent speed stability on approach, a
// nose-down tendency as speed falls. Returns the amount to subtract from the pitch command.
double speed_stability_correction(double velocity_mps)
{
    if (velocity_mps < cfg::speed_stability_velocity_mps && alpha_filtered > cfg::speed_stability_ao_a_threshold)
    {
        const double speed_fade = 1.0 - velocity_mps / cfg::speed_stability_velocity_mps;
        const double aoa_excess = alpha_filtered - cfg::speed_stability_ao_a_threshold;
        return cfg::speed_stability_gain * aoa_excess * speed_fade;
    }
    return 0.0;
}

double fcs_pitch_controller_step(const Pitch_inputs& in, double dt, bool reset_filters)
{
    double stick_command_pos = fcs_pitch_controller_force_command(in.long_stick_input_command, in.pitch_trim, dt);

    // AAR: reduce pitch stick position gain (per Loschke: "stick position input gain is decreased")
    if (pitch_mode_air_refuel_door_open)
    {
        stick_command_pos *= Roll_controller_config::aar_lateral_stick_gain_scale;
    }
    const double roll_rad = in.roll_angle_deg * (pi / 180.0);
    const double pitch_rad = in.pitch_angle_deg * (pi / 180.0);

    az_filtered = in.az;

    const double alpha_limited = limit(in.angle_of_attack_ind, -5.0, cfg::ao_a_max);
    const double alpha_blend = 1.0 - std::exp(-cfg::ao_a_filter_rate * dt);
    alpha_filtered += (alpha_limited - alpha_filtered) * alpha_blend;

    const double pitch_rate_rad_s = in.pitch_rate_deg_s * (pi / 180.0);
    const double gravity_component = std::cos(roll_rad) * std::cos(pitch_rad);
    const double nz_measured = az_filtered + gravity_component;

    update_nz_reference(stick_command_pos, gravity_component, roll_rad, pitch_rad, in, dt);

    const double nz_cmd = limit(nz_reference + stick_command_pos, cfg::nz_min, cfg::nz_max);
    const Nz_law_output law =
        nz_command_law(nz_cmd, nz_measured, in.dyn_pressure_pa, pitch_rate_rad_s, dt, reset_filters);

    double pitch_command_total = law.command;
    pitch_command_total = apply_aoa_limiter(pitch_command_total, in, dt, reset_filters);
    pitch_command_total -= roll_rate_squared_correction(in.roll_rate_deg_s);
    pitch_command_total -= speed_stability_correction(in.velocity_mps);

    // Every path goes through the servo model, the AOA limiter included. Without this the limiter
    // switched the elevons between stops at about 15 Hz at low dynamic pressure.
    const double final_pitch_command_total = pitch_actuator_dynamics_filter.step(pitch_command_total, reset_filters);

    Detail::Pitch_debug_sample sample;
    sample.roll_angle_deg = in.roll_angle_deg;
    sample.pitch_angle_deg = in.pitch_angle_deg;
    sample.roll_rate_deg_s = in.roll_rate_deg_s;
    sample.raw_az = in.az;
    sample.gravity_component = gravity_component;
    sample.nz_measured = nz_measured;
    sample.nz_reference = nz_reference;
    sample.stick_command_pos = stick_command_pos;
    sample.nz_command = law.nz_command;
    sample.nz_error = law.nz_error;
    sample.nz_proportional = law.proportional;
    sample.pitch_rate_blend = law.pitch_rate_blend;
    sample.nz_control = law.control;
    sample.integrator_out = law.integrator;
    sample.raw_ao_a = in.angle_of_attack_ind;
    sample.filtered_ao_a = alpha_filtered;
    sample.raw_stick_input = in.long_stick_input_command;
    sample.pitch_rate = in.pitch_rate_deg_s;
    sample.dyn_pressure_pa = in.dyn_pressure_pa;
    sample.velocity_mps = in.velocity_mps;
    sample.mach = in.mach;
    sample.thrust_n = in.thrust_n;
    sample.cx_total = in.cx_total;
    sample.elevator_out = final_pitch_command_total;
    Detail::log_pitch_debug_sample(sample);

    return final_pitch_command_total;
}
}  // namespace

namespace Detail
{

void reset_pitch_state()
{
    stick_command_pos_filtered = 0.0;
    az_filtered = 0.0;
    nz_reference = 1.0;
    recovery_nz_active = false;
    recovery_nz = 1.0;
    pitch_integrator_state = 0.0;
    pitch_controller_initialized = false;
    pitch_control_accumulator = 0.0;
    pitch_held_output = 0.0;
    pitch_mode_gear_down = 0.0;
    pitch_mode_air_refuel_door_open = false;
    aoa_limiter_integrator = 0.0;
    aoa_limiter_active = false;
    close_pitch_debug_log();
}

}  // namespace Detail

void set_recovery_nz_command(bool active, double nz)
{
    recovery_nz_active = active;
    recovery_nz = nz;
}

void update_pitch_mode_auto_inputs(double gear_down, bool air_refuel_door_open)
{
    pitch_mode_gear_down = gear_down;
    pitch_mode_air_refuel_door_open = air_refuel_door_open;
}

double fcs_pitch_controller(double long_stick_input_command, double pitch_trim, double angle_of_attack_ind,
                            double pitch_rate_deg_s, double az, double differential_command, double dyn_pressure_pa,
                            double dt, double roll_angle_deg, double pitch_angle_deg, double roll_rate_deg_s,
                            double velocity_mps, double mach, double thrust_n, double cx_total)
{
    // TODO: differentialCommand (elevon differential for pitch/roll mixing) is not yet implemented.
    (void)differential_command;

    const bool needs_init = !pitch_controller_initialized;
    if (needs_init)
    {
        init_pitch_filters(fixed_control_time_step);
    }

    Detail::advance_control_accumulator(pitch_control_accumulator, dt);
    if (needs_init && pitch_control_accumulator < fixed_control_time_step)
    {
        pitch_control_accumulator = fixed_control_time_step;
    }

    const Pitch_inputs inputs = {long_stick_input_command,
                                 pitch_trim,
                                 angle_of_attack_ind,
                                 pitch_rate_deg_s,
                                 az,
                                 dyn_pressure_pa,
                                 roll_angle_deg,
                                 pitch_angle_deg,
                                 roll_rate_deg_s,
                                 velocity_mps,
                                 mach,
                                 thrust_n,
                                 cx_total};
    bool reset_filters = needs_init;
    while (pitch_control_accumulator >= fixed_control_time_step)
    {
        pitch_held_output = fcs_pitch_controller_step(inputs, fixed_control_time_step, reset_filters);
        reset_filters = false;
        pitch_control_accumulator -= fixed_control_time_step;
    }

    return pitch_held_output;
}

}  // namespace Flight_controls
}  // namespace F117
