#pragma once
// Tuning constants for the three control axes. Values are unchanged from the original single-file
// implementation; sources are the F-117A flight manual (TO 1F-117A-1) and Loschke's description of
// the control laws, as cited beside each value.
#include "../EfmMath.h"

namespace F117
{
namespace FLIGHTCONTROLS
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kFixedControlTimeStep = 0.005;
constexpr int kMaxControlSubstepsPerFrame = 20;
constexpr double kMaxAccumulatedControlTime = kFixedControlTimeStep * kMaxControlSubstepsPerFrame;
constexpr int kPitchDebugLogIntervalFrames = 10;

// 1 with a wheel on the ground, 0 in the air, moving between them in half a second so liftoff and
// touchdown don't step the surfaces. On the ground, runway bumps reach the roll rate and load factor
// sensors but the gear holds the aircraft, so feeding them back only moves the elevons.
constexpr double kGroundContactFadeRate = 2.0;  // per second

namespace YawControllerConfig
{
// Beta-command architecture (real F-117 directional axis).
// Pedal position commands a target Beta; fins are driven to satisfy that Beta.

// Pedal → Beta command gain (deg Beta per unit normalised pedal).
// The law droops, so these commands sit above the sideslip they produce.
constexpr double kBetaCommandGain = 16.0;
// Beta command limit schedule vs dynamic pressure.
// Below kBetaCmdQbarLow the full limit applies (crosswind landing authority).
// Above kBetaCmdQbarHigh the limit is reduced to protect the airframe.
constexpr double kBetaCmdQbarLow = 2000.0;    // Pa (~80 kts)
constexpr double kBetaCmdQbarHigh = 28000.0;  // Pa (~370 kts)
// Set so full pedal gives the steady sideslip on TO 1F-117A-1 page 6-3: 8 deg at 170 KCAS, 4 deg at 500 KCAS.
constexpr double kBetaCmdLimitHigh = 9.35;  // deg (low-speed limit)
constexpr double kBetaCmdLimitLow = 4.7;    // deg (high-speed limit)
// Small pedal deadband at neutral (normalised units).
constexpr double kBetaPedalDeadband = 0.02;
// Base Beta feedback gain (deg fin per deg Beta error).
// Gain doubles linearly as AOA increases from 0 to kBetaGainAoADoubleAlpha
// to keep Beta tight at high AOA where its destabilising pitching moment is largest.
constexpr double kBetaFeedbackGainBase = 3.0;
constexpr double kBetaGainAoADoubleAlpha = 20.0;  // deg AOA where gain doubles
// Automatic Beta trim: bleeds residual Beta out slowly while gear is up and
// pedal force is below threshold. Acts through a small deadband.
constexpr double kAutoBetaTrimDeadband = 0.5;  // deg
constexpr double kAutoBetaTrimRate = 0.4;      // (deg trim)/s per deg Beta excess
// It waits this long after the pedal is centred, so it trims what is left rather than chasing the
// sideslip still dying away, which overshot the release.
constexpr double kAutoBetaTrimDelay_S = 3.0;
constexpr double kAutoBetaTrimLimit = 5.0;            // deg max trim authority
constexpr double kAutoBetaTrimPedalThreshold = 0.05;  // normalised pedal (~22 lb)
// Inertia-coupling term: product of pitch_rate × roll_rate fed back to yaw
// to prevent divergence during high roll-rate manoeuvres (real F-117 feature).
constexpr double kInertiaCouplingGain = -0.4;  // deg fin per (rad/s)^2
// Yaw-rate damping, deg fin per deg/s, scheduled on dynamic pressure like the Beta command limit:
// the low-speed value below kBetaCmdQbarLow, the high-speed one above kBetaCmdQbarHigh. These give
// under 10% sideslip overshoot from a pedal step and from its release (TO 1F-117A-1 page 6-1:
// "essentially dead-beat damping in all three axes"). More at high speed starts a small oscillation.
constexpr double kYawDampingGainLowSpeed = 2.0;
constexpr double kYawDampingGainHighSpeed = 1.5;
// Beta command low-pass filter rate (smooths pedal transients).
constexpr double kCommandFilterRate = 4.0;
// Yaw rate lead-lag shaping.
constexpr double kYawLeadLagNumerator0 = 3.0;
constexpr double kYawLeadLagPole = 15.0;
// Fin servo dynamics.
constexpr double kServoNaturalFrequency = 52.0;
constexpr double kServoDampingRatio = 0.95;
}  // namespace YawControllerConfig

namespace PitchControllerConfig
{
// Threshold on gearDown signal (0–1) above which gear is considered extended.
constexpr double kGearDownThreshold = 0.1;
constexpr double kPositiveStickForceScale = 80.0;
constexpr double kNegativeStickForceScale = 180.0;
constexpr double kStickForceMin = -180.0;
constexpr double kStickForceMax = 80.0;
constexpr double kStickCommandDeadband = 8.0;
constexpr double kStickCommandBreakpoint = 33.0;
constexpr double kStickCommandRateLimit = 8.0;  // g per second
constexpr double kNzCommandMin = -2.0;
constexpr double kNzCommandMax = 7.0;
// Integrator strength on Nz error. Higher = more steady-state authority and more stored pull;
// lower = less overshoot/carry-through, but slower trim-in to the commanded G.
constexpr double kPitchIntegratorGain = 0.75;
// Extra multiplier used only when the integrator is unwinding against the current error.
// Higher = faster dump of stored pull/push and less overshoot; lower = smoother, but more carry-through.
constexpr double kPitchIntegratorUnwindGain = 3.0;  // was 4.0 TEST
constexpr double kAoAFilterRate = 35.0;
// Real F-117: full aft stick = +7g, full forward = -2g.
constexpr double kNzMax = 7.0;
constexpr double kNzMin = -2.0;
constexpr double kNzStickDeadband = 0.05;
constexpr double kNzReferenceTrackRate = 2.0;
// Proportional gain on Nz error (scaled by 1/qbar at runtime).
constexpr double kNzErrorGain = 1.7;
// Pitch-rate blend: washed-out pitch rate is blended INTO the Nz feedback before the
// error is computed (per Loschke: "specified blend of washed out pitch rate and the
// normal acceleration"). This gives a true P+I architecture — pitch rate damps transient
// response via the error signal; the washout ensures zero steady-state contribution so the
// integrator does not fight the damper in sustained pull/push manoeuvres.
// Gain derived to match old separate-damping authority at reference q:
//   kPitchRateBlendGain = old_kPitchDampingGain_magnitude / kNzErrorGain = 7.0 / 1.7 ≈ 4.1
constexpr double kPitchRateBlendGain = 4.1;           // g·s/rad
constexpr double kPitchRateBlendWashoutTauInv = 1.0;  // rad/s (1-second washout time constant)
constexpr double kPitchIntegratorLimit = 25.0;
constexpr double kAoAMax = 20.0;
// High-gain P+I AOA limiter (has more authority than the pilot).
// The limiter is triggered by alpha + washed-out pitch rate anticipation exceeding
// the threshold from aoa_limit_for_mach.
constexpr double kAoALimiterProportionalGain = 3.0;
constexpr double kAoALimiterIntegratorGain = 2.5;
constexpr double kAoALimiterIntegratorLimit = 30.0;
// Pitch-rate anticipation for the AOA limiter.
// Washed-out pitch rate is scaled by kAoAAnticipationGainRef / qbar so the anticipation
// is stronger at low q (where departure risk is highest on aggressive pull-ups).
constexpr double kAoAAnticipationWashoutTauInverse = 2.0;  // 0.5 s washout
constexpr double kAoAAnticipationGainRef = 0.2;            // deg AoA per deg/s at kQbarRef
constexpr double kAoAAnticipationMax = 6.0;                // deg (anticipation clamp)
// Dynamic pressure scheduling of forward-loop gains (1/qbar).
// All forward-loop gains are proportional to qbarRef/qbar, clamped to [min, max].
constexpr double kQbarRef = 14000.0;        // Pa (~300 kts SL)
constexpr double kQbarMin = 2000.0;         // Pa (prevents divide issues)
constexpr double kQbarGainScaleMin = 0.25;  // Down to 56 kPa, about 600 KCAS at sea level
constexpr double kQbarGainScaleMax = 3.0;
// Roll-rate^2 inertia coupling fed back to pitch axis.
// Prevents inertia-coupled pitch departures during high-AoA rolling manoeuvres.
constexpr double kRollRateSquaredAoAThreshold = 12.0;  // deg, AoA above which active
constexpr double kRollRateSquaredRateThreshold = 1.0;  // rad/s, rate above which active
constexpr double kRollRateSquaredGain = 0.05;          // deg elevator per (rad/s)^2
// Speed stability: non-linear AoA feedback at AoA > 7 deg and below 200 kts.
// Provides apparent speed stability on approach (pitches down as speed falls).
constexpr double kSpeedStabilityVelocityMPS = 103.0;  // 200 kts
constexpr double kSpeedStabilityAoAThreshold = 7.0;   // deg
constexpr double kSpeedStabilityGain = 0.3;           // deg per deg AoA excess
constexpr double kPitchServoNaturalFrequency = 52.0;
constexpr double kPitchServoDampingRatio = 1.5;

// Gain on the gravity feed-forward that updates nz_reference each frame.
// Higher = nz_reference follows rapid bank/pitch changes more aggressively;
// lower = less brief unload/pitch-down during fast roll recoveries, but more gravity-comp lag.
constexpr double kGravityFeedForwardGain = 1.0;

// When the stick is in the deadband and nz_reference is BELOW the gravity component
// (e.g. after being clamped at kNzMin by the feed-forward during an aggressive push),
// snap back up at this faster rate so nz_cmd recovers immediately on stick release
// rather than lagging behind for ~0.5 s at the -3 G floor.
constexpr double kNzReferenceSnapRate = 20.0;

// Force-to-Nz schedule: target G commands at the piecewise-linear breakpoints.
// Changing a breakpoint force (above) or a target G (here) automatically
// keeps all derived slopes/intercepts consistent.
constexpr double kNzAtNegForceBreak = -0.4;  // Nz at –kStickCommandBreakpoint lbf (push)
constexpr double kNzAtPosForceBreak = 0.4;   // Nz at +kStickCommandBreakpoint lbf (pull) //0.8
constexpr double kNzAtMaxPullForce = 4.0;    // Nz at +kStickForceMax lbf (pull)

// Push (negative): segment 1 – deadband to first breakpoint
constexpr double kNzSlopeNeg1 = kNzAtNegForceBreak / (-kStickCommandBreakpoint + kStickCommandDeadband);
constexpr double kNzInterceptNeg1 = kNzSlopeNeg1 * kStickCommandDeadband;
// Push (negative): segment 2 – beyond first breakpoint (steeper empirical ramp)
constexpr double kNzSlopeNeg2 = 0.067;
constexpr double kNzInterceptNeg2 = kNzAtNegForceBreak + kNzSlopeNeg2 * kStickCommandBreakpoint;
// Pull (positive): segment 1 – deadband to first breakpoint
constexpr double kNzSlopePos1 = kNzAtPosForceBreak / (kStickCommandBreakpoint - kStickCommandDeadband);
constexpr double kNzInterceptPos1 = -kNzSlopePos1 * kStickCommandDeadband;
// Pull (positive): segment 2 – first breakpoint to max force
constexpr double kNzSlopePos2 = (kNzAtMaxPullForce - kNzAtPosForceBreak) / (kStickForceMax - kStickCommandBreakpoint);
constexpr double kNzInterceptPos2 = kNzAtPosForceBreak - kNzSlopePos2 * kStickCommandBreakpoint;
}  // namespace PitchControllerConfig

namespace RollControllerConfig
{
constexpr double kLatStickForceScale = 75.0;
// Lateral-acceleration bias gain used to shape the roll command in loaded flight.
// Higher = more roll command added from ay; lower = cleaner stick-recenter stop, but less coupled feel under load.
constexpr double kAyBiasGain = 8.9;
// Lateral-stick magnitude where the ay bias reaches full strength.
// Lower = bias comes in sooner; higher = bias fades out more around center stick, improving roll stop after release.
constexpr double kAyBiasFullStick = 0.35;
// Beta-to-roll-rate interconnect used only in rudder-held, near-center-stick flat turns.
// Higher = more aileron help to resist the inside wing dropping during yaw-only turns;
// lower = more natural bank build-up from the airframe with less FCS help.
constexpr double kFlatTurnBetaGain = 2.5;
// Bank-angle hold term paired with the beta interconnect above.
// Higher = stronger wings-level hold in a flat turn; lower = less obvious automatic roll correction.
constexpr double kFlatTurnBankGain = 3.0;
// Lateral-stick fade point for the flat-turn interconnect.
// Lower = pilot roll input overrides the helper sooner; higher = helper remains active farther off-center.
constexpr double kFlatTurnLatStickFade = 0.2;
// Pedal magnitude where the flat-turn interconnect reaches full strength.
// Lower = more help with small rudder inputs; higher = helper mainly appears at strong pedal.
constexpr double kFlatTurnPedalFull = 0.6;
constexpr double kFlatTurnCommandLimit = 12.0;
constexpr double kLongStickFeelGain = 0.0667;
constexpr double kRollFeelForceThreshold = 25.0;
constexpr double kRollFeelFixedGain = 0.7;
constexpr double kRollFeelSlope = 0.012;
constexpr double kStickDeadband = 3.0;
// Roll rate loop bandwidth, rad/s. The gain comes from the aircraft's current control power and
// roll inertia, so the loop responds the same at every speed and doesn't need retuning if the inertia changes.
constexpr double kRollLoopBandwidth = 15.0;
constexpr double kRollLoopGainMax = 3.0;      // deg surface per deg/s, for very low dynamic pressure
constexpr double kRollFeedForwardMax = 0.35;  // deg surface per deg/s
constexpr double kRollCommandLimit = 21.5;
constexpr double kLatForceFilterPole = 60.0;
constexpr double kRollCommandFilterPole = 10.0;
constexpr double kRollServoNaturalFrequency = 52.0;
constexpr double kRollServoDampingRatio = 0.85;
constexpr double kRollRateFilterPole = 50.0;
constexpr double kRollRateFilter2Num0 = 4.0;
constexpr double kRollRateFilter2Num1 = 64.0;
constexpr double kRollRateFilter2Den1 = 80.0;
constexpr double kRollRateFilter2OmegaSquared = 6400.0;

// Parabolic stick-force-to-roll-rate shaping (real F-117 uses non-linear parabolic
// shaping per Loschke). The schedule is: rollRateCmd = K * (force - deadband)^2
// giving a gentle onset near centre and increasing responsiveness toward full stick.
// The rate at full stick follows TO 1F-117A-1 page 6-3: about 80 deg/s at low speed to about
// 190 deg/s above 300 KCAS. Sideslip adds a little to the low speed figure, so that command sits below it.
constexpr double kRollRateMaxLowSpeed = 74.0;    // deg/s commanded at or below kRollRateQbarLow
constexpr double kRollRateMaxHighSpeed = 190.0;  // deg/s commanded at or above kRollRateQbarHigh
constexpr double kRollRateQbarLow = 4700.0;      // Pa, about 170 KCAS
constexpr double kRollRateQbarHigh = 14600.0;    // Pa, about 300 KCAS
constexpr double kRollUsableForce = kLatStickForceScale - kStickDeadband;  // 72 lbf
// Gear down: roll rate feedback gain is increased to reduce turbulence response on approach.
constexpr double kGearDownRateFeedbackGainFactor = 1.6;
// Air refuelling: lateral stick input gain is reduced for control harmony with the
// reduced pitch gain in AAR mode.
constexpr double kAARLateralStickGainScale = 0.6;
}  // namespace RollControllerConfig

// Maximum indicated AOA against Mach from the flight test limits in TO 1F-117A-1 Figure 5-7.
inline double aoa_limit_for_mach(double mach, bool gearDown)
{
    if (gearDown)
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
}  // namespace FLIGHTCONTROLS
}  // namespace F117
