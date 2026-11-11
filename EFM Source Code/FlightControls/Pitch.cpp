// Longitudinal axis: normal-acceleration P+I law with an AOA limiter.
#include "stdafx.h"
#include "Internal.h"
#include "PitchLog.h"

#include <algorithm>
#include <cmath>

#include "TustinFilter.h"

namespace F117
{
namespace FLIGHTCONTROLS
{

namespace
{
namespace cfg = PitchControllerConfig;

TustinFilter pitchActuatorDynamicsFilter;
TustinFilter pitchRateWashoutFilter;    // AOA limiter pitch-rate anticipation
TustinFilter pitchRateFeedbackWashout;  // main-loop pitch-rate blend (P+I feedback)

double stickCommandPosFiltered{};
double azFiltered{};
double nz_reference = 1.0;      // current 1G reference; reset alongside filter state
bool recoveryNzActive = false;  // PAARS load-factor command replaces the 1G reference
double recoveryNz = 1.0;
double pitchIntegratorState = 0.0;  // tracks raw integrator output for state anti-windup

bool pitchControllerInitialized = false;
double pitchControlAccumulator = 0.0;
double pitchHeldOutput = 0.0;

double pitchModeGearDown = 0.0;
bool pitchModeAirRefuelDoorOpen = false;

double aoaLimiterIntegrator = 0.0;  // P+I AOA limiter integrator state
bool aoaLimiterActive = false;      // Limiter overrode the g command on the last step

struct PitchInputs
{
    double longStickInputCommand;
    double pitchTrim;
    double angle_of_attack_ind;
    double pitch_rate_DEG_s;
    double az;
    double dynPressure_PA;
    double roll_angle_DEG;
    double pitch_angle_DEG;
    double roll_rate_DEG_s;
    double velocity_mps;
    double mach;
    double thrust_N;
    double Cx_total;
};

// Everything the P+I law produces for one step; the debug log records the intermediate values.
struct NzLawOutput
{
    double nzCommand;
    double nzError;
    double proportional;
    double control;
    double integrator;
    double pitchRateBlend;
    double command;
};

void init_pitch_filters(double dt)
{
    const double servoOmegaSquared = std::pow(cfg::kPitchServoNaturalFrequency, 2.0);
    double numerators[3] = {0.0, 0.0, servoOmegaSquared};
    double denominators[3] = {1.0, 2.0 * cfg::kPitchServoDampingRatio * cfg::kPitchServoNaturalFrequency,
                              servoOmegaSquared};
    pitchActuatorDynamicsFilter.configure(numerators, denominators, 2, dt);

    // AOA limiter pitch-rate anticipation washout filter (tau = 0.5 s)
    double washoutNum[2] = {1.0, 0.0};
    double washoutDen[2] = {1.0, cfg::kAoAAnticipationWashoutTauInverse};
    pitchRateWashoutFilter.configure(washoutNum, washoutDen, 1, dt);

    // Main-loop pitch-rate feedback blend washout filter (tau = 1.0 s)
    // Separate from the AOA anticipation filter - different time constant.
    double blendWashoutNum[2] = {1.0, 0.0};
    double blendWashoutDen[2] = {1.0, cfg::kPitchRateBlendWashoutTauInv};
    pitchRateFeedbackWashout.configure(blendWashoutNum, blendWashoutDen, 1, dt);

    stickCommandPosFiltered = 0.0;
    azFiltered = 0.0;
    nz_reference = 1.0;
    pitchIntegratorState = 0.0;
    aoaLimiterIntegrator = 0.0;
    aoaLimiterActive = false;
    detail::ground_contact_fade = 0.0;
    pitchControlAccumulator = 0.0;
    pitchHeldOutput = 0.0;
    pitchControllerInitialized = true;
    detail::open_pitch_debug_log();
}

// Stick force to scheduled load-factor demand. Choose the sign branch first, then its
// inner/outer segment; the deadband is inclusive.
double load_from_force(double force)
{
    if (std::abs(force) <= cfg::kStickCommandDeadband)
        return 0.0;
    if (force < 0.0)
        return force <= -cfg::kStickCommandBreakpoint ? cfg::kNzSlopeNeg2 * force + cfg::kNzInterceptNeg2
                                                      : cfg::kNzSlopeNeg1 * force + cfg::kNzInterceptNeg1;
    return force >= cfg::kStickCommandBreakpoint ? cfg::kNzSlopePos2 * force + cfg::kNzInterceptPos2
                                                 : cfg::kNzSlopePos1 * force + cfg::kNzInterceptPos1;
}

double fcs_pitch_controller_force_command(double longStickInputCommand, double pitchTrim, double dt)
{
    double longStickInputForce = 0.0;
    if (longStickInputCommand > 0.0)
    {
        longStickInputForce = longStickInputCommand * cfg::kPositiveStickForceScale + pitchTrim;
    }
    else
    {
        longStickInputForce = longStickInputCommand * cfg::kNegativeStickForceScale;
    }
    longStickForce = longStickInputForce = limit(longStickInputForce, cfg::kStickForceMin, cfg::kStickForceMax);

    const double longStickCommandWithTrimLimited_G =
        limit(pitchTrim - load_from_force(longStickForce), cfg::kNzCommandMin, cfg::kNzCommandMax);
    // Command rate limit. A step of stick would step the elevons, and on a tailless aircraft a sudden up-elevon
    // first loses lift before the nose comes up; ramping large commands keeps that dip and the overshoot after
    // it small. Small corrections pass through almost at once.
    const double maxStep = cfg::kStickCommandRateLimit * dt;
    stickCommandPosFiltered += limit(longStickCommandWithTrimLimited_G - stickCommandPosFiltered, -maxStep, maxStep);

    return stickCommandPosFiltered;
}

// The 1G reference follows gravity's component along the load-factor axis while the stick is
// centred, then feeds gravity forward as the aircraft rolls and pitches.
void update_nz_reference(double stickCommandPos, double gravity_component, double roll_RAD, double pitch_RAD,
                         const PitchInputs& in, double dt)
{
    if (std::fabs(stickCommandPos) < cfg::kNzStickDeadband)
    {
        const double gravityError = gravity_component - nz_reference;
        const double trackRate = (gravityError > 0.0) ? cfg::kNzReferenceSnapRate : cfg::kNzReferenceTrackRate;
        const double referenceBlend = 1.0 - std::exp(-trackRate * dt);
        nz_reference += gravityError * referenceBlend;
    }

    {
        const double roll_rate_rad_s = in.roll_rate_DEG_s * (kPi / 180.0);
        const double pitch_rate_rad_s = in.pitch_rate_DEG_s * (kPi / 180.0);
        const double gravity_rate = (-std::sin(roll_RAD) * std::cos(pitch_RAD) * roll_rate_rad_s) +
                                    (-std::cos(roll_RAD) * std::sin(pitch_RAD) * pitch_rate_rad_s);
        nz_reference += cfg::kGravityFeedForwardGain * gravity_rate * dt;
        nz_reference = limit(nz_reference, cfg::kNzMin, cfg::kNzMax);
    }

    // PAARS commands the load factor; pilot stick still adds on top, and on release
    // the reference continues from the last commanded value.
    if (recoveryNzActive)
    {
        nz_reference = limit(recoveryNz, cfg::kNzMin, cfg::kNzMax);
    }
}

bool gear_is_down()
{
    return pitchModeGearDown > cfg::kGearDownThreshold;
}

// Integrator with anti-windup. Gear-down and AAR: proportional only (no integrator) - gives
// classic phugoid + short period, and removes the need for WoW switching at touchdown/lift-off.
// AAR: per Loschke, pitch law reverts to proportional-only with reduced stick gain.
void update_pitch_integrator(double nz_measured, double nz_error, double nzProportional, double dt)
{
    const double integratorLimit = cfg::kPitchIntegratorLimit;
    double integratorGain = cfg::kPitchIntegratorGain;

    if (gear_is_down() || pitchModeAirRefuelDoorOpen)
    {
        // Freeze and bleed integrator to zero; proportional path only
        pitchIntegratorState = pitchIntegratorState * (1.0 - limit(dt * 5.0, 0.0, 1.0));
        return;
    }

    const bool nzAtUpperLimit = (nz_measured >= cfg::kNzMax && nz_error > 0.0);
    const bool nzAtLowerLimit = (nz_measured <= cfg::kNzMin && nz_error < 0.0);

    const double unsaturatedCommandBeforeIntegration = nzProportional + pitchIntegratorState;
    const bool actuatorSaturatedHigh = (unsaturatedCommandBeforeIntegration >= integratorLimit && nz_error > 0.0);
    const bool actuatorSaturatedLow = (unsaturatedCommandBeforeIntegration <= -integratorLimit && nz_error < 0.0);
    // Frozen while the AOA limiter holds the nose, or it stores a pull that carries on after the stick is released.
    const double integratorInput =
        (nzAtUpperLimit || nzAtLowerLimit || actuatorSaturatedHigh || actuatorSaturatedLow || aoaLimiterActive)
            ? 0.0
            : nz_error;

    if ((pitchIntegratorState > 0.0 && integratorInput < 0.0) || (pitchIntegratorState < 0.0 && integratorInput > 0.0))
    {
        integratorGain *= cfg::kPitchIntegratorUnwindGain;
    }

    pitchIntegratorState =
        limit(pitchIntegratorState + integratorGain * dt * integratorInput, -integratorLimit, integratorLimit);
}

// P+I g-command law per Loschke. Pitch-rate blend damps transients via the error signal;
// 1/qbar scaling on the proportional gain automatically scales the blend contribution across
// the envelope. All forward-loop gains are scheduled on inverse dynamic pressure so stick feel
// is consistent across the flight envelope.
NzLawOutput nz_command_law(double nz_cmd, double nz_measured, double dynPressure_PA, double pitchRate_RAD_s, double dt,
                           bool resetFilters)
{
    const double qbarScaled = (std::max)(dynPressure_PA, cfg::kQbarMin);
    const double qbarGainScale = limit(cfg::kQbarRef / qbarScaled, cfg::kQbarGainScaleMin, cfg::kQbarGainScaleMax);
    const double scaledNzErrorGain = cfg::kNzErrorGain * qbarGainScale;

    // Washed-out pitch rate blended INTO the Nz feedback before the error is formed.
    // Per Loschke: "a specified blend of washed out pitch rate and the normal acceleration".
    // Washout (tau = 1 s) ensures zero steady-state contribution so the integrator never
    // fights a persistent damping offset in sustained manoeuvres.
    const double pitchRateWashedBlend = pitchRateFeedbackWashout.step(pitchRate_RAD_s, resetFilters);

    // On the ground the load factor is ignored, which leaves the stick's path to the elevons unchanged.
    const double blendedNzFeedback = nz_measured + detail::ground_contact_fade * (nz_reference - nz_measured) +
                                     cfg::kPitchRateBlendGain * pitchRateWashedBlend;

    NzLawOutput out = {};
    out.nzCommand = nz_cmd;
    out.nzError = nz_cmd - blendedNzFeedback;
    out.proportional = scaledNzErrorGain * out.nzError;
    out.control = out.proportional;

    update_pitch_integrator(nz_measured, out.nzError, out.proportional, dt);

    out.command =
        limit(out.proportional + pitchIntegratorState, -cfg::kPitchIntegratorLimit, cfg::kPitchIntegratorLimit);
    out.integrator = pitchIntegratorState;
    out.pitchRateBlend = cfg::kPitchRateBlendGain * pitchRateWashedBlend;
    return out;
}

// ---- Real F-117 AOA limiter: high-gain P+I, activated by AoA + washed-out pitch-rate anticipation ----
// Threshold is Mach-scheduled from flight test limits (falls at high Mach).
// Pitch-rate anticipation is dynamic-pressure-scheduled (stronger at low q for aggressive pull-ups).
// When active the limiter has MORE authority than the pilot.
double apply_aoa_limiter(double pitchCommandTotal, const PitchInputs& in, double dt, bool resetFilters)
{
    const double limiterThreshold = aoa_limit_for_mach(in.mach, gear_is_down());

    // Washed-out pitch rate for anticipation (q-scheduled gain)
    const double pitchRateForAnticipation = pitchRateWashoutFilter.step(in.pitch_rate_DEG_s, resetFilters);
    const double qbarForAnticiption = (std::max)(in.dynPressure_PA, cfg::kQbarMin);
    const double anticipationGain = cfg::kAoAAnticipationGainRef * (cfg::kQbarRef / qbarForAnticiption);
    const double anticipation =
        limit(anticipationGain * pitchRateForAnticipation, -cfg::kAoAAnticipationMax, cfg::kAoAAnticipationMax);

    const double limiterInput = alphaFiltered + anticipation;
    const double limiterError = limiterInput - limiterThreshold;

    // The limiter's command keeps rising as AOA falls below the limit, and the lower of it and the pilot's
    // command wins, so the handover is continuous. Switching straight to the pilot's command as soon as AOA
    // dipped under the limit stepped the surface and made a 7 Hz load factor buzz at the limit.
    const double limiterCommand = -(cfg::kAoALimiterProportionalGain * limiterError + aoaLimiterIntegrator);
    aoaLimiterActive = limiterCommand < pitchCommandTotal;

    if (aoaLimiterActive)
    {
        pitchCommandTotal = limiterCommand;
        // Either sign, because holding the limit can take a little nose-up surface.
        aoaLimiterIntegrator = limit(aoaLimiterIntegrator + cfg::kAoALimiterIntegratorGain * dt * limiterError,
                                     -cfg::kAoALimiterIntegratorLimit, cfg::kAoALimiterIntegratorLimit);
    }
    else
    {
        // Decay integrator when not active so re-entry is smooth
        const double decay = cfg::kAoALimiterIntegratorGain * dt * 2.0;
        aoaLimiterIntegrator = limit(0.0, aoaLimiterIntegrator - decay, aoaLimiterIntegrator + decay);
    }
    return pitchCommandTotal;
}

// Roll-rate^2 inertia coupling (pitch axis): prevents inertia-coupled pitch departures during
// high-AoA rolling manoeuvres. Returns the amount to subtract from the pitch command.
double roll_rate_squared_correction(double roll_rate_DEG_s)
{
    const double rollRateRadS = roll_rate_DEG_s * (kPi / 180.0);
    if (alphaFiltered > cfg::kRollRateSquaredAoAThreshold &&
        std::abs(rollRateRadS) > cfg::kRollRateSquaredRateThreshold)
    {
        const double rollRateSq = rollRateRadS * rollRateRadS;
        return cfg::kRollRateSquaredGain * rollRateSq;
    }
    return 0.0;
}

// Speed stability (below 200 kts, AoA > 7 deg): apparent speed stability on approach, a
// nose-down tendency as speed falls. Returns the amount to subtract from the pitch command.
double speed_stability_correction(double velocity_mps)
{
    if (velocity_mps < cfg::kSpeedStabilityVelocityMPS && alphaFiltered > cfg::kSpeedStabilityAoAThreshold)
    {
        const double speedFade = 1.0 - velocity_mps / cfg::kSpeedStabilityVelocityMPS;
        const double aoaExcess = alphaFiltered - cfg::kSpeedStabilityAoAThreshold;
        return cfg::kSpeedStabilityGain * aoaExcess * speedFade;
    }
    return 0.0;
}

double fcs_pitch_controller_step(const PitchInputs& in, double dt, bool resetFilters)
{
    double stickCommandPos = fcs_pitch_controller_force_command(in.longStickInputCommand, in.pitchTrim, dt);

    // AAR: reduce pitch stick position gain (per Loschke: "stick position input gain is decreased")
    if (pitchModeAirRefuelDoorOpen)
    {
        stickCommandPos *= RollControllerConfig::kAARLateralStickGainScale;
    }
    const double roll_RAD = in.roll_angle_DEG * (kPi / 180.0);
    const double pitch_RAD = in.pitch_angle_DEG * (kPi / 180.0);

    azFiltered = in.az;

    const double alphaLimited = limit(in.angle_of_attack_ind, -5.0, cfg::kAoAMax);
    const double alphaBlend = 1.0 - std::exp(-cfg::kAoAFilterRate * dt);
    alphaFiltered += (alphaLimited - alphaFiltered) * alphaBlend;

    const double pitchRate_RAD_s = in.pitch_rate_DEG_s * (kPi / 180.0);
    const double gravity_component = std::cos(roll_RAD) * std::cos(pitch_RAD);
    const double nz_measured = azFiltered + gravity_component;

    update_nz_reference(stickCommandPos, gravity_component, roll_RAD, pitch_RAD, in, dt);

    const double nz_cmd = limit(nz_reference + stickCommandPos, cfg::kNzMin, cfg::kNzMax);
    const NzLawOutput law = nz_command_law(nz_cmd, nz_measured, in.dynPressure_PA, pitchRate_RAD_s, dt, resetFilters);

    double pitchCommandTotal = law.command;
    pitchCommandTotal = apply_aoa_limiter(pitchCommandTotal, in, dt, resetFilters);
    pitchCommandTotal -= roll_rate_squared_correction(in.roll_rate_DEG_s);
    pitchCommandTotal -= speed_stability_correction(in.velocity_mps);

    // Every path goes through the servo model, the AOA limiter included. Without this the limiter
    // switched the elevons between stops at about 15 Hz at low dynamic pressure.
    const double finalPitchCommandTotal = pitchActuatorDynamicsFilter.step(pitchCommandTotal, resetFilters);

    detail::PitchDebugSample sample;
    sample.rollAngleDeg = in.roll_angle_DEG;
    sample.pitchAngleDeg = in.pitch_angle_DEG;
    sample.rollRateDegS = in.roll_rate_DEG_s;
    sample.rawAz = in.az;
    sample.gravityComponent = gravity_component;
    sample.nzMeasured = nz_measured;
    sample.nzReference = nz_reference;
    sample.stickCommandPos = stickCommandPos;
    sample.nzCommand = law.nzCommand;
    sample.nzError = law.nzError;
    sample.nzProportional = law.proportional;
    sample.pitchRateBlend = law.pitchRateBlend;
    sample.nzControl = law.control;
    sample.integratorOut = law.integrator;
    sample.rawAoA = in.angle_of_attack_ind;
    sample.filteredAoA = alphaFiltered;
    sample.rawStickInput = in.longStickInputCommand;
    sample.pitchRate = in.pitch_rate_DEG_s;
    sample.dynPressure_PA = in.dynPressure_PA;
    sample.velocity_MPS = in.velocity_mps;
    sample.mach = in.mach;
    sample.thrust_N = in.thrust_N;
    sample.cxTotal = in.Cx_total;
    sample.elevatorOut = finalPitchCommandTotal;
    detail::log_pitch_debug_sample(sample);

    return finalPitchCommandTotal;
}
}  // namespace

namespace detail
{

void reset_pitch_state()
{
    stickCommandPosFiltered = 0.0;
    azFiltered = 0.0;
    nz_reference = 1.0;
    recoveryNzActive = false;
    recoveryNz = 1.0;
    pitchIntegratorState = 0.0;
    pitchControllerInitialized = false;
    pitchControlAccumulator = 0.0;
    pitchHeldOutput = 0.0;
    pitchModeGearDown = 0.0;
    pitchModeAirRefuelDoorOpen = false;
    aoaLimiterIntegrator = 0.0;
    aoaLimiterActive = false;
    close_pitch_debug_log();
}

}  // namespace detail

void set_recovery_nz_command(bool active, double nz)
{
    recoveryNzActive = active;
    recoveryNz = nz;
}

void update_pitch_mode_auto_inputs(double gearDown, bool airRefuelDoorOpen)
{
    pitchModeGearDown = gearDown;
    pitchModeAirRefuelDoorOpen = airRefuelDoorOpen;
}

double fcs_pitch_controller(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind,
                            double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_PA,
                            double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s,
                            double velocity_mps, double mach, double thrust_N, double Cx_total)
{
    // TODO: differentialCommand (elevon differential for pitch/roll mixing) is not yet implemented.
    (void)differentialCommand;

    const bool needsInit = !pitchControllerInitialized;
    if (needsInit)
    {
        init_pitch_filters(kFixedControlTimeStep);
    }

    detail::advance_control_accumulator(pitchControlAccumulator, dt);
    if (needsInit && pitchControlAccumulator < kFixedControlTimeStep)
    {
        pitchControlAccumulator = kFixedControlTimeStep;
    }

    const PitchInputs inputs = {longStickInputCommand,
                                pitchTrim,
                                angle_of_attack_ind,
                                pitch_rate_DEG_s,
                                az,
                                dynPressure_PA,
                                roll_angle_DEG,
                                pitch_angle_DEG,
                                roll_rate_DEG_s,
                                velocity_mps,
                                mach,
                                thrust_N,
                                Cx_total};
    bool resetFilters = needsInit;
    while (pitchControlAccumulator >= kFixedControlTimeStep)
    {
        pitchHeldOutput = fcs_pitch_controller_step(inputs, kFixedControlTimeStep, resetFilters);
        resetFilters = false;
        pitchControlAccumulator -= kFixedControlTimeStep;
    }

    return pitchHeldOutput;
}

}  // namespace FLIGHTCONTROLS
}  // namespace F117
