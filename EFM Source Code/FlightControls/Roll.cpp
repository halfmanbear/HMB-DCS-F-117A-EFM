// Lateral axis: parabolic stick-to-roll-rate command with a roll-rate feedback loop.
#include "stdafx.h"
#include "Internal.h"

#include <algorithm>
#include <cmath>

#include "TustinFilter.h"

namespace F117 { namespace FLIGHTCONTROLS {

namespace
{
    namespace cfg = RollControllerConfig;

    TustinFilter latStickForceFilter;
    TustinFilter rollCommandFilter;
    TustinFilter rollActuatorDynamicsFilter;
    TustinFilter rollRateFilter1;
    TustinFilter rollRateFilter2;

    bool rollControllerInitialized = false;
    double rollControlAccumulator = 0.0;
    double rollHeldOutput = 0.0;

    struct RollInputs
    {
        double latStickInputCommand;
        double longStickForceCommand;
        double ay;
        double pedInput;
        double beta_deg;
        double roll_angle_deg;
        double roll_rate;
        double roll_rate_trim;
        double dynPressure_PA;
        double rollAccelPerSurfaceDeg;
        double surfacePerRollRate;
        bool gearDown;
        bool airRefuelDoorOpen;
    };

    void init_roll_filters(double dt)
    {
        double numerators[2] = { 0.0, cfg::kLatForceFilterPole };
        double denominators[2] = { 1.0, cfg::kLatForceFilterPole };
        latStickForceFilter.configure(numerators, denominators, 1, dt);

        double numerators1[2] = { 0.0, cfg::kRollCommandFilterPole };
        double denominators1[2] = { 1.0, cfg::kRollCommandFilterPole };
        rollCommandFilter.configure(numerators1, denominators1, 1, dt);

        const double servoOmegaSquared = std::pow(cfg::kRollServoNaturalFrequency, 2.0);
        double numerators2[3] = { 0.0, 0.0, servoOmegaSquared };
        double denominators2[3] = { 1.0, 2.0 * cfg::kRollServoDampingRatio * cfg::kRollServoNaturalFrequency, servoOmegaSquared };
        rollActuatorDynamicsFilter.configure(numerators2, denominators2, 2, dt);

        double numerators3[2] = { 0.0, cfg::kRollRateFilterPole };
        double denominators3[2] = { 1.0, cfg::kRollRateFilterPole };
        rollRateFilter1.configure(numerators3, denominators3, 1, dt);

        double numerators4[3] = { cfg::kRollRateFilter2Num0, cfg::kRollRateFilter2Num1, cfg::kRollRateFilter2OmegaSquared };
        double denominators4[3] = { 1.0, cfg::kRollRateFilter2Den1, cfg::kRollRateFilter2OmegaSquared };
        rollRateFilter2.configure(numerators4, denominators4, 2, dt);

        rollControlAccumulator = 0.0;
        rollHeldOutput = 0.0;
        detail::debug_snapshot.roll = detail::RollSnapshot{};
        rollControllerInitialized = true;
    }

    // Feel gain that eases the roll command as the pilot pulls or pushes on the pitch axis.
    double roll_feel_gain(double longStickForceCommand)
    {
        const double longStickForceGained = longStickForceCommand * cfg::kLongStickFeelGain;
        double rollFeelGain = 0.0;
        if (std::abs(longStickForceCommand) > cfg::kRollFeelForceThreshold)
        {
            rollFeelGain = cfg::kRollFeelFixedGain;
        }
        else if (longStickForceCommand >= 0.0)
        {
            rollFeelGain = -cfg::kRollFeelSlope * longStickForceGained + 1.0;
        }
        else if (longStickForceCommand < 0.0)
        {
            rollFeelGain = cfg::kRollFeelSlope * longStickForceGained + 1.0;
        }
        return rollFeelGain;
    }

    // Parabolic stick-force-to-roll-rate shaping.
    // rollRateCmd = sign * K * (|force| - deadband)^2, clamped to +/-kRollRateMax.
    // This matches the real F-117's non-linear parabolic shaping: gentle onset at centre
    // (good precision) with rapidly increasing authority toward full deflection (good jinking).
    double parabolic_roll_rate_command(double latStickForceFinal, double dynPressure_PA)
    {
        double rollRateCommand = 0.0;
        if (std::abs(latStickForceFinal) > cfg::kStickDeadband)
        {
            const double sign = (latStickForceFinal > 0.0) ? 1.0 : -1.0;
            const double forceBeyondDeadband = std::abs(latStickForceFinal) - cfg::kStickDeadband;
            const double speedFraction = limit((dynPressure_PA - cfg::kRollRateQbarLow)
                / (cfg::kRollRateQbarHigh - cfg::kRollRateQbarLow), 0.0, 1.0);
            const double rollRateMax = cfg::kRollRateMaxLowSpeed
                + speedFraction * (cfg::kRollRateMaxHighSpeed - cfg::kRollRateMaxLowSpeed);
            const double parabolicGain = rollRateMax / (cfg::kRollUsableForce * cfg::kRollUsableForce);
            rollRateCommand = sign * parabolicGain * forceBeyondDeadband * forceBeyondDeadband;
            rollRateCommand = limit(rollRateCommand, -rollRateMax, rollRateMax);
        }
        return rollRateCommand;
    }

    // Flat-turn interconnect: with the rudder held and the stick near centre, aileron resists the
    // inside wing dropping. Fades out on the ground.
    double flat_turn_roll_command(const RollInputs& in, double effectiveLateralStick)
    {
        const double flatTurnPedalBlend = limit(std::abs(in.pedInput) / cfg::kFlatTurnPedalFull, 0.0, 1.0);
        const double flatTurnStickBlend = 1.0 - limit(std::abs(effectiveLateralStick) / cfg::kFlatTurnLatStickFade, 0.0, 1.0);
        const double flatTurnBlend = flatTurnPedalBlend * flatTurnStickBlend;
        return (1.0 - detail::ground_contact_fade) * flatTurnBlend * limit(
            in.beta_deg * cfg::kFlatTurnBetaGain - in.roll_angle_deg * cfg::kFlatTurnBankGain,
            -cfg::kFlatTurnCommandLimit,
            cfg::kFlatTurnCommandLimit);
    }

    double fcs_roll_controller_step(const RollInputs& in, bool resetFilters)
    {
        // Air refuelling: lateral stick gain reduced for control harmony with reduced pitch gain.
        const double effectiveLateralStick = in.airRefuelDoorOpen
            ? in.latStickInputCommand * cfg::kAARLateralStickGainScale
            : in.latStickInputCommand;

        const double latStickForceCmd = effectiveLateralStick * cfg::kLatStickForceScale;
        const double latStickForce = latStickForceFilter.step(latStickForceCmd, resetFilters);
        const double ayBiasBlend = limit(std::abs(effectiveLateralStick) / cfg::kAyBiasFullStick, 0.0, 1.0);
        const double latStickForceBiased = latStickForce - (in.ay * cfg::kAyBiasGain * ayBiasBlend);

        const double latStickForceFinal{latStickForceBiased * roll_feel_gain(in.longStickForceCommand)};
        const double rollRateCommand = parabolic_roll_rate_command(latStickForceFinal, in.dynPressure_PA);
        const double flatTurnRollCommand = flat_turn_roll_command(in, effectiveLateralStick);

        const double rollRateCommandTotal = rollRateCommand + flatTurnRollCommand;
        const double rollRateCommandFiltered = rollCommandFilter.step(rollRateCommandTotal, resetFilters);
        const double rollRateFiltered1 = rollRateFilter1.step(in.roll_rate, resetFilters);
        const double rollRateFiltered2 = rollRateFilter2.step(rollRateFiltered1, resetFilters);
        const double rollRateCommandWithTrim = rollRateCommandFiltered + in.roll_rate_trim;

        // Positive surface rolls left. Feedback sets how the rate error dies away; the feed-forward holds
        // the surface that balances roll damping at the commanded rate, so the rate doesn't droop.
        double rollLoopGain = (std::min)(cfg::kRollLoopBandwidth / (std::max)(in.rollAccelPerSurfaceDeg, 1.0), cfg::kRollLoopGainMax);

        // Gear down: increase roll rate feedback gain to reduce response to turbulence on approach.
        if (in.gearDown)
        {
            rollLoopGain *= cfg::kGearDownRateFeedbackGainFactor;
        }

        const double rollFeedForward = limit(in.surfacePerRollRate, 0.0, cfg::kRollFeedForwardMax);
        // On the ground the roll rate is ignored, which leaves the stick's path to the elevons unchanged.
        const double rollRateFeedback = rollRateFiltered2 * (1.0 - detail::ground_contact_fade);
        const double rollCommandGained = limit((rollRateFeedback - rollRateCommandWithTrim) * rollLoopGain - rollRateCommandWithTrim * rollFeedForward,
            -cfg::kRollCommandLimit, cfg::kRollCommandLimit);
        const double rollSurfaceCommand = rollActuatorDynamicsFilter.step(rollCommandGained, resetFilters);

        detail::RollSnapshot& snapshot = detail::debug_snapshot.roll;
        snapshot.stickInput = effectiveLateralStick;
        snapshot.stickForce = latStickForceFinal;
        snapshot.rateCommand = rollRateCommand;
        snapshot.flatTurnCommand = flatTurnRollCommand;
        snapshot.rateCommandFiltered = rollRateCommandFiltered;
        snapshot.commandGained = rollCommandGained;
        snapshot.surfaceCommand = rollSurfaceCommand;

        return rollSurfaceCommand;
    }
}

namespace detail {

void reset_roll_state()
{
    rollControllerInitialized = false;
    rollControlAccumulator = 0.0;
    rollHeldOutput = 0.0;
}

} // namespace detail

double fcs_roll_controller(double latStickInputCommand, double longStickForceCommand, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_PA, double rollAccelPerSurfaceDeg, double surfacePerRollRate, bool gearDown, bool airRefuelDoorOpen, double dt)
{
    const bool needsInit = !rollControllerInitialized;
    if (needsInit)
    {
        init_roll_filters(kFixedControlTimeStep);
    }

    detail::advance_control_accumulator(rollControlAccumulator, dt);
    if (needsInit && rollControlAccumulator < kFixedControlTimeStep)
    {
        rollControlAccumulator = kFixedControlTimeStep;
    }

    const RollInputs inputs = { latStickInputCommand, longStickForceCommand, ay, pedInput, beta_deg, roll_angle_deg,
                                roll_rate, roll_rate_trim, dynPressure_PA, rollAccelPerSurfaceDeg, surfacePerRollRate,
                                gearDown, airRefuelDoorOpen };
    bool resetFilters = needsInit;
    while (rollControlAccumulator >= kFixedControlTimeStep)
    {
        rollHeldOutput = fcs_roll_controller_step(inputs, resetFilters);
        resetFilters = false;
        rollControlAccumulator -= kFixedControlTimeStep;
    }

    return rollHeldOutput;
}

}}
