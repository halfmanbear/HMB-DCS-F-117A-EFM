#pragma once
// Tuning constants for the three control axes. Values are unchanged from the original single-file
// implementation; sources are the F-117A flight manual (TO 1F-117A-1) and Loschke's description of
// the control laws, as cited beside each value.
#include "../EfmMath.h"

namespace F117
{
namespace Flight_controls
{

constexpr double pi = 3.14159265358979323846;
constexpr double fixed_control_time_step = 0.005;
constexpr int max_control_substeps_per_frame = 20;
constexpr double max_accumulated_control_time = fixed_control_time_step * max_control_substeps_per_frame;
constexpr int pitch_debug_log_interval_frames = 10;

// 1 with a wheel on the ground, 0 in the air, moving between them in half a second so liftoff and
// touchdown don't step the surfaces. On the ground, runway bumps reach the roll rate and load factor
// sensors but the gear holds the aircraft, so feeding them back only moves the elevons.
constexpr double ground_contact_fade_rate = 2.0;  // per second

namespace Yaw_controller_config
{
// Beta-command architecture (real F-117 directional axis).
// Pedal position commands a target Beta; fins are driven to satisfy that Beta.

// Pedal → Beta command gain (deg Beta per unit normalised pedal).
// The law droops, so these commands sit above the sideslip they produce.
constexpr double beta_command_gain = 16.0;
// Beta command limit schedule vs dynamic pressure.
// Below kBetaCmdQbarLow the full limit applies (crosswind landing authority).
// Above kBetaCmdQbarHigh the limit is reduced to protect the airframe.
constexpr double beta_cmd_qbar_low = 2000.0;    // Pa (~80 kts)
constexpr double beta_cmd_qbar_high = 28000.0;  // Pa (~370 kts)
// Set so full pedal gives the steady sideslip on TO 1F-117A-1 page 6-3: 8 deg at 170 KCAS, 4 deg at 500 KCAS.
constexpr double beta_cmd_limit_high = 9.35;  // deg (low-speed limit)
constexpr double beta_cmd_limit_low = 4.7;    // deg (high-speed limit)
// Small pedal deadband at neutral (normalised units).
constexpr double beta_pedal_deadband = 0.02;
// Base Beta feedback gain (deg fin per deg Beta error).
// Gain doubles linearly as AOA increases from 0 to kBetaGainAoADoubleAlpha
// to keep Beta tight at high AOA where its destabilising pitching moment is largest.
constexpr double beta_feedback_gain_base = 3.0;
constexpr double beta_gain_ao_a_double_alpha = 20.0;  // deg AOA where gain doubles
// Automatic Beta trim: bleeds residual Beta out slowly while gear is up and
// pedal force is below threshold. Acts through a small deadband.
constexpr double auto_beta_trim_deadband = 0.5;  // deg
constexpr double auto_beta_trim_rate = 0.4;      // (deg trim)/s per deg Beta excess
// It waits this long after the pedal is centred, so it trims what is left rather than chasing the
// sideslip still dying away, which overshot the release.
constexpr double auto_beta_trim_delay_s = 3.0;
constexpr double auto_beta_trim_limit = 5.0;             // deg max trim authority
constexpr double auto_beta_trim_pedal_threshold = 0.05;  // normalised pedal (~22 lb)
// Inertia-coupling term: product of pitch_rate × roll_rate fed back to yaw
// to prevent divergence during high roll-rate manoeuvres (real F-117 feature).
constexpr double inertia_coupling_gain = -0.4;  // deg fin per (rad/s)^2
// Yaw-rate damping, deg fin per deg/s, scheduled on dynamic pressure like the Beta command limit:
// the low-speed value below kBetaCmdQbarLow, the high-speed one above kBetaCmdQbarHigh. These give
// under 10% sideslip overshoot from a pedal step and from its release (TO 1F-117A-1 page 6-1:
// "essentially dead-beat damping in all three axes"). More at high speed starts a small oscillation.
constexpr double yaw_damping_gain_low_speed = 2.0;
constexpr double yaw_damping_gain_high_speed = 1.5;
// Beta command low-pass filter rate (smooths pedal transients).
constexpr double command_filter_rate = 4.0;
// Yaw rate lead-lag shaping.
constexpr double yaw_lead_lag_numerator0 = 3.0;
constexpr double yaw_lead_lag_pole = 15.0;
// Fin servo dynamics.
constexpr double servo_natural_frequency = 52.0;
constexpr double servo_damping_ratio = 0.95;
}  // namespace Yaw_controller_config

namespace Pitch_controller_config
{
// Threshold on gearDown signal (0–1) above which gear is considered extended.
constexpr double gear_down_threshold = 0.1;
constexpr double positive_stick_force_scale = 80.0;
constexpr double negative_stick_force_scale = 180.0;
constexpr double stick_force_min = -180.0;
constexpr double stick_force_max = 80.0;
constexpr double stick_command_deadband = 8.0;
constexpr double stick_command_breakpoint = 33.0;
constexpr double stick_command_rate_limit = 8.0;  // g per second
constexpr double nz_command_min = -2.0;
constexpr double nz_command_max = 7.0;
// Integrator strength on Nz error. Higher = more steady-state authority and more stored pull;
// lower = less overshoot/carry-through, but slower trim-in to the commanded G.
constexpr double pitch_integrator_gain = 0.75;
// Extra multiplier used only when the integrator is unwinding against the current error.
// Higher = faster dump of stored pull/push and less overshoot; lower = smoother, but more carry-through.
constexpr double pitch_integrator_unwind_gain = 3.0;  // was 4.0 TEST
constexpr double ao_a_filter_rate = 35.0;
// Real F-117: full aft stick = +7g, full forward = -2g.
constexpr double nz_max = 7.0;
constexpr double nz_min = -2.0;
constexpr double nz_reference_track_rate = 2.0;  // 1/s, how fast the reference eases to 1 g plus trim
// Proportional gain on Nz error (scaled by 1/qbar at runtime).
constexpr double nz_error_gain = 1.7;
// Pitch-rate blend: washed-out pitch rate is blended INTO the Nz feedback before the
// error is computed (per Loschke: "specified blend of washed out pitch rate and the
// normal acceleration"). This gives a true P+I architecture — pitch rate damps transient
// response via the error signal; the washout ensures zero steady-state contribution so the
// integrator does not fight the damper in sustained pull/push manoeuvres.
// Gain derived to match old separate-damping authority at reference q:
//   kPitchRateBlendGain = old_kPitchDampingGain_magnitude / kNzErrorGain = 7.0 / 1.7 ≈ 4.1
constexpr double pitch_rate_blend_gain = 4.1;             // g·s/rad
constexpr double pitch_rate_blend_washout_tau_inv = 1.0;  // rad/s (1-second washout time constant)
constexpr double pitch_integrator_limit = 25.0;
constexpr double ao_a_max = 20.0;
// High-gain P+I AOA limiter (has more authority than the pilot).
// The limiter is triggered by alpha + washed-out pitch rate anticipation exceeding
// the threshold from aoa_limit_for_mach.
constexpr double ao_a_limiter_proportional_gain = 3.0;
constexpr double ao_a_limiter_integrator_gain = 2.5;
constexpr double ao_a_limiter_integrator_limit = 30.0;
// Pitch-rate anticipation for the AOA limiter.
// Washed-out pitch rate is scaled by kAoAAnticipationGainRef / qbar so the anticipation
// is stronger at low q (where departure risk is highest on aggressive pull-ups).
constexpr double ao_a_anticipation_washout_tau_inverse = 2.0;  // 0.5 s washout
constexpr double ao_a_anticipation_gain_ref = 0.2;             // deg AoA per deg/s at kQbarRef
constexpr double ao_a_anticipation_max = 6.0;                  // deg (anticipation clamp)
// Dynamic pressure scheduling of forward-loop gains (1/qbar).
// All forward-loop gains are proportional to qbarRef/qbar, clamped to [min, max].
constexpr double qbar_ref = 14000.0;          // Pa (~300 kts SL)
constexpr double qbar_min = 2000.0;           // Pa (prevents divide issues)
constexpr double qbar_gain_scale_min = 0.25;  // Down to 56 kPa, about 600 KCAS at sea level
constexpr double qbar_gain_scale_max = 3.0;
// Roll-rate^2 inertia coupling fed back to pitch axis.
// Prevents inertia-coupled pitch departures during high-AoA rolling manoeuvres.
constexpr double roll_rate_squared_ao_a_threshold = 12.0;  // deg, AoA above which active
constexpr double roll_rate_squared_rate_threshold = 1.0;   // rad/s, rate above which active
constexpr double roll_rate_squared_gain = 0.05;            // deg elevator per (rad/s)^2
// Speed stability: non-linear AoA feedback at AoA > 7 deg and below 200 kts.
// Provides apparent speed stability on approach (pitches down as speed falls).
constexpr double speed_stability_velocity_mps = 103.0;  // 200 kts
constexpr double speed_stability_ao_a_threshold = 7.0;  // deg
constexpr double speed_stability_gain = 0.3;            // deg per deg AoA excess
constexpr double pitch_servo_natural_frequency = 52.0;
constexpr double pitch_servo_damping_ratio = 1.5;

// Force-to-Nz schedule: target G commands at the piecewise-linear breakpoints.
// Changing a breakpoint force (above) or a target G (here) automatically
// keeps all derived slopes/intercepts consistent.
constexpr double nz_at_neg_force_break = -0.4;  // Nz at –kStickCommandBreakpoint lbf (push)
constexpr double nz_at_pos_force_break = 0.4;   // Nz at +kStickCommandBreakpoint lbf (pull) //0.8
constexpr double nz_at_max_pull_force = 4.0;    // Nz at +kStickForceMax lbf (pull)

// Push (negative): segment 1 – deadband to first breakpoint
constexpr double nz_slope_neg1 = nz_at_neg_force_break / (-stick_command_breakpoint + stick_command_deadband);
constexpr double nz_intercept_neg1 = nz_slope_neg1 * stick_command_deadband;
// Push (negative): segment 2 – beyond first breakpoint (steeper empirical ramp)
constexpr double nz_slope_neg2 = 0.067;
constexpr double nz_intercept_neg2 = nz_at_neg_force_break + nz_slope_neg2 * stick_command_breakpoint;
// Pull (positive): segment 1 – deadband to first breakpoint
constexpr double nz_slope_pos1 = nz_at_pos_force_break / (stick_command_breakpoint - stick_command_deadband);
constexpr double nz_intercept_pos1 = -nz_slope_pos1 * stick_command_deadband;
// Pull (positive): segment 2 – first breakpoint to max force
constexpr double nz_slope_pos2 =
    (nz_at_max_pull_force - nz_at_pos_force_break) / (stick_force_max - stick_command_breakpoint);
constexpr double nz_intercept_pos2 = nz_at_pos_force_break - nz_slope_pos2 * stick_command_breakpoint;
}  // namespace Pitch_controller_config

namespace Roll_controller_config
{
constexpr double lat_stick_force_scale = 75.0;
// Lateral-acceleration bias gain used to shape the roll command in loaded flight.
// Higher = more roll command added from ay; lower = cleaner stick-recenter stop, but less coupled feel under load.
constexpr double ay_bias_gain = 8.9;
// Lateral-stick magnitude where the ay bias reaches full strength.
// Lower = bias comes in sooner; higher = bias fades out more around center stick, improving roll stop after release.
constexpr double ay_bias_full_stick = 0.35;
// Beta-to-roll-rate interconnect used only in rudder-held, near-center-stick flat turns.
// Higher = more aileron help to resist the inside wing dropping during yaw-only turns;
// lower = more natural bank build-up from the airframe with less FCS help.
constexpr double flat_turn_beta_gain = 2.5;
// Bank-angle hold term paired with the beta interconnect above.
// Higher = stronger wings-level hold in a flat turn; lower = less obvious automatic roll correction.
constexpr double flat_turn_bank_gain = 3.0;
// Lateral-stick fade point for the flat-turn interconnect.
// Lower = pilot roll input overrides the helper sooner; higher = helper remains active farther off-center.
constexpr double flat_turn_lat_stick_fade = 0.2;
// Pedal magnitude where the flat-turn interconnect reaches full strength.
// Lower = more help with small rudder inputs; higher = helper mainly appears at strong pedal.
constexpr double flat_turn_pedal_full = 0.6;
constexpr double flat_turn_command_limit = 12.0;
constexpr double long_stick_feel_gain = 0.0667;
constexpr double roll_feel_force_threshold = 25.0;
constexpr double roll_feel_fixed_gain = 0.7;
constexpr double roll_feel_slope = 0.012;
constexpr double stick_deadband = 3.0;
// Roll rate loop bandwidth, rad/s. The gain comes from the aircraft's current control power and
// roll inertia, so the loop responds the same at every speed and doesn't need retuning if the inertia changes.
constexpr double roll_loop_bandwidth = 15.0;
constexpr double roll_loop_gain_max = 3.0;      // deg surface per deg/s, for very low dynamic pressure
constexpr double roll_feed_forward_max = 0.35;  // deg surface per deg/s
constexpr double roll_command_limit = 21.5;
constexpr double lat_force_filter_pole = 60.0;
constexpr double roll_command_filter_pole = 10.0;
constexpr double roll_servo_natural_frequency = 52.0;
constexpr double roll_servo_damping_ratio = 0.85;
constexpr double roll_rate_filter_pole = 50.0;
constexpr double roll_rate_filter2_num0 = 4.0;
constexpr double roll_rate_filter2_num1 = 64.0;
constexpr double roll_rate_filter2_den1 = 80.0;
constexpr double roll_rate_filter2_omega_squared = 6400.0;

// Parabolic stick-force-to-roll-rate shaping (real F-117 uses non-linear parabolic
// shaping per Loschke). The schedule is: rollRateCmd = K * (force - deadband)^2
// giving a gentle onset near centre and increasing responsiveness toward full stick.
// The rate at full stick follows TO 1F-117A-1 page 6-3: about 80 deg/s at low speed to about
// 190 deg/s above 300 KCAS. Sideslip adds a little to the low speed figure, so that command sits below it.
constexpr double roll_rate_max_low_speed = 74.0;    // deg/s commanded at or below kRollRateQbarLow
constexpr double roll_rate_max_high_speed = 190.0;  // deg/s commanded at or above kRollRateQbarHigh
constexpr double roll_rate_qbar_low = 4700.0;       // Pa, about 170 KCAS
constexpr double roll_rate_qbar_high = 14600.0;     // Pa, about 300 KCAS
constexpr double roll_usable_force = lat_stick_force_scale - stick_deadband;  // 72 lbf
// Gear down: roll rate feedback gain is increased to reduce turbulence response on approach.
constexpr double gear_down_rate_feedback_gain_factor = 1.6;
// Air refuelling: lateral stick input gain is reduced for control harmony with the
// reduced pitch gain in AAR mode.
constexpr double aar_lateral_stick_gain_scale = 0.6;
}  // namespace Roll_controller_config

// Maximum indicated AOA against Mach from the flight test limits in TO 1F-117A-1 Figure 5-7.
inline double aoa_limit_for_mach(double mach, bool gear_down)
{
    if (gear_down)
    {
        return mach <= 0.35 ? 14.6 : 14.6 - (limit(mach, 0.35, 0.70) - 0.35) * (1.9 / 0.35);
    }
    if (mach <= 0.53)
    {
        return 12.6;
    }
    if (mach <= 0.70)
    {
        return 12.6 - (mach - 0.53) * (1.1 / 0.17);
    }
    return 11.5 - (limit(mach, 0.70, 0.90) - 0.70) * (3.3 / 0.20);
}
}  // namespace Flight_controls
}  // namespace F117
