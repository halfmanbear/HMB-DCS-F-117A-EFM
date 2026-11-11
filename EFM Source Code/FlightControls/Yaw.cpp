// Directional axis: Beta-command augmentation (proportional only).
#include "stdafx.h"
#include "Internal.h"

#include <algorithm>
#include <cmath>

#include "TustinFilter.h"

namespace F117 { namespace FLIGHTCONTROLS {

namespace
{
    namespace cfg = YawControllerConfig;

    TustinFilter rudderCommandFilter;
    TustinFilter yawRateFilter;
    TustinFilter yawServoFilter;

    bool yawControllerInitialized = false;
    double yawControlAccumulator = 0.0;
    double yawHeldOutput = 0.0;
    double autoBetaTrimState = 0.0;      // automatic Beta trim accumulator (deg)
    double autoBetaTrimWait_S = 0.0;     // time left before the automatic Beta trim may act

    struct YawInputs
    {
        double pedInput;
        double pedTrim;
        double beta_deg;
        double yaw_rate;
        double turn_yaw_rate;
        double roll_rate;
        double pitch_rate;
        double aoa_filtered;
        double dynPressure_PA;
        bool gearDown;
    };

    void init_yaw_filters(double dt)
    {
        // Beta command low-pass filter
        double numerators[2] = { 0.0, cfg::kCommandFilterRate };
        double denominators[2] = { 1.0, cfg::kCommandFilterRate };
        rudderCommandFilter.configure(numerators, denominators, 1, dt);

        // Yaw rate lead-lag shaping
        double numerators2[2] = { cfg::kYawLeadLagNumerator0, cfg::kYawLeadLagPole };
        double denominators2[2] = { 1.0, cfg::kYawLeadLagPole };
        yawRateFilter.configure(numerators2, denominators2, 1, dt);

        // Fin servo dynamics
        const double servoOmegaSquared = std::pow(cfg::kServoNaturalFrequency, 2.0);
        double numerators3[3] = { 0.0, 0.0, servoOmegaSquared };
        double denominators3[3] = { 1.0, 2.0 * cfg::kServoDampingRatio * cfg::kServoNaturalFrequency, servoOmegaSquared };
        yawServoFilter.configure(numerators3, denominators3, 2, dt);

        yawControlAccumulator = 0.0;
        yawHeldOutput = 0.0;
        autoBetaTrimState = 0.0;
        autoBetaTrimWait_S = 0.0;
        detail::debug_snapshot.yaw = detail::YawSnapshot{};
        detail::debug_snapshot.aero = detail::AeroSnapshot{};
        yawControllerInitialized = true;
    }

    // Real F-117 directional axis: Beta-command augmentation (proportional only).
    // Pedal position commands a target sideslip angle (Beta). The fins are driven to
    // satisfy that command. Beta feedback gain is AoA-scheduled (doubles at high AoA to
    // resist the destabilising pitching moment from Beta). Automatic Beta trim bleeds
    // residual Beta when gear is up and pedal force is below threshold. Inertia coupling
    // (pitch_rate x roll_rate) prevents directional divergence during high-rate manoeuvres.
    // The ARI (aileron-rudder interconnect) present in early design was removed from the
    // real aircraft after evaluation - it is NOT implemented here.
    double fcs_yaw_controller_step(const YawInputs& in, double dt, bool resetFilters)
    {
        const double pedInput = in.pedInput;
        const double beta_deg = in.beta_deg;
        const double aoa_filtered = in.aoa_filtered;

        // 1. Pedal -> Beta command (with small deadband)
        double rawPedalCmd = 0.0;
        if (std::abs(pedInput) > cfg::kBetaPedalDeadband)
        {
            const double sign = (pedInput > 0.0) ? 1.0 : -1.0;
            rawPedalCmd = sign * (std::abs(pedInput) - cfg::kBetaPedalDeadband)
                          / (1.0 - cfg::kBetaPedalDeadband)
                          * cfg::kBetaCommandGain;
        }

        // Dynamic pressure schedule: limit Beta command at high speed
        // 0 at and below kBetaCmdQbarLow, 1 at and above kBetaCmdQbarHigh.
        const double qbarFraction = limit((in.dynPressure_PA - cfg::kBetaCmdQbarLow)
            / (cfg::kBetaCmdQbarHigh - cfg::kBetaCmdQbarLow), 0.0, 1.0);
        const double betaCmdLimit = cfg::kBetaCmdLimitHigh
            + qbarFraction * (cfg::kBetaCmdLimitLow - cfg::kBetaCmdLimitHigh);
        const double betaCmdLimited = limit(rawPedalCmd, -betaCmdLimit, betaCmdLimit);

        // Low-pass filter the Beta command to smooth pedal transients
        const double betaCmdFiltered = rudderCommandFilter.step(betaCmdLimited, resetFilters);

        // 2. Automatic Beta trim (series trim): active when gear is up and pedal near centre.
        // Slowly trims away residual Beta (e.g. from engine thrust asymmetry).
        const bool pedalCentred = std::abs(pedInput) < cfg::kAutoBetaTrimPedalThreshold;
        autoBetaTrimWait_S = pedalCentred ? (std::max)(autoBetaTrimWait_S - dt, 0.0) : cfg::kAutoBetaTrimDelay_S;
        if (!in.gearDown && pedalCentred && autoBetaTrimWait_S <= 0.0)
        {
            const double betaExcess = beta_deg - limit(beta_deg, -cfg::kAutoBetaTrimDeadband, cfg::kAutoBetaTrimDeadband);
            autoBetaTrimState -= cfg::kAutoBetaTrimRate * betaExcess * dt;
            autoBetaTrimState  = limit(autoBetaTrimState, -cfg::kAutoBetaTrimLimit, cfg::kAutoBetaTrimLimit);
        }

        const double betaCommandTotal = betaCmdFiltered + in.pedTrim + autoBetaTrimState;

        // 3. Beta feedback with AoA-scheduled gain.
        // Gain doubles linearly from base at 0 deg AoA to 2x base at kBetaGainAoADoubleAlpha. Negative AOA counts too:
        // the wing's dihedral effect follows its lift, so it reverses there and destabilises as much.
        const double aoaFraction = limit(std::abs(aoa_filtered) / cfg::kBetaGainAoADoubleAlpha, 0.0, 1.0);
        const double betaGain = cfg::kBetaFeedbackGainBase * (1.0 + aoaFraction);
        const double betaError = betaCommandTotal - beta_deg;
        const double betaFeedback = betaGain * betaError;

        // 4. Stability-axis yaw rate damping (stability axis correction: subtract roll x alpha coupling).
        // The yaw rate a steady banked turn needs is taken out first, so the damper never opposes a turn and
        // needs no washout. A washout leaves a tail after every yaw transient that pulls the sideslip back
        // towards zero, which overshot pedal steps by 20 to 50%.
        const double alphaRad = aoa_filtered * (kPi / 180.0);
        const double yawRateStabilityAxis = in.yaw_rate - in.roll_rate * alphaRad - in.turn_yaw_rate;
        const double yawRateSmoothed  = yawRateFilter.step(yawRateStabilityAxis, resetFilters);
        const double yawDampingGain = cfg::kYawDampingGainLowSpeed
            + qbarFraction * (cfg::kYawDampingGainHighSpeed - cfg::kYawDampingGainLowSpeed);
        const double yawDamping = yawDampingGain * yawRateSmoothed;

        // 5. Inertia coupling: pitch_rate x roll_rate to prevent inertia-coupled yaw divergence.
        const double pitch_rate_rad_s = in.pitch_rate * (kPi / 180.0);
        const double roll_rate_rad_s  = in.roll_rate  * (kPi / 180.0);
        const double inertiaCoupling  = cfg::kInertiaCouplingGain * pitch_rate_rad_s * roll_rate_rad_s;

        // 6. Combine and pass through fin servo dynamics
        const double combinedCommand    = betaFeedback + yawDamping + inertiaCoupling;
        const double yawSurfaceCommand  = yawServoFilter.step(combinedCommand, resetFilters);

        detail::YawSnapshot& snapshot = detail::debug_snapshot.yaw;
        snapshot.pedInput              = pedInput;
        snapshot.rateDegS              = in.yaw_rate;
        snapshot.rudderCommand         = rawPedalCmd;
        snapshot.rudderCommandFiltered = betaCmdFiltered;
        snapshot.pedalCommand          = betaCommandTotal;
        snapshot.damping               = yawDamping;
        snapshot.sideAccelFeedback     = inertiaCoupling;  // re-used log slot
        snapshot.ariCommand            = 0.0;              // ARI removed from real aircraft
        snapshot.combinedCommand       = combinedCommand;
        snapshot.surfaceCommand        = yawSurfaceCommand;

        return yawSurfaceCommand;
    }
}

namespace detail {

void reset_yaw_state()
{
    yawControllerInitialized = false;
    yawControlAccumulator = 0.0;
    yawHeldOutput = 0.0;
    autoBetaTrimState = 0.0;
    autoBetaTrimWait_S = 0.0;
}

} // namespace detail

double fcs_yaw_controller(double pedInput, double pedTrim, double beta_deg, double yaw_rate, double turn_yaw_rate, double roll_rate, double pitch_rate, double aoa_filtered, double dynPressure_PA, bool gearDown, double dt)
{
    const bool needsInit = !yawControllerInitialized;
    if (needsInit)
    {
        init_yaw_filters(kFixedControlTimeStep);
    }

    detail::advance_control_accumulator(yawControlAccumulator, dt);
    if (needsInit && yawControlAccumulator < kFixedControlTimeStep)
    {
        yawControlAccumulator = kFixedControlTimeStep;
    }

    const YawInputs inputs = { pedInput, pedTrim, beta_deg, yaw_rate, turn_yaw_rate, roll_rate, pitch_rate,
                               aoa_filtered, dynPressure_PA, gearDown };
    bool resetFilters = needsInit;
    while (yawControlAccumulator >= kFixedControlTimeStep)
    {
        yawHeldOutput = fcs_yaw_controller_step(inputs, kFixedControlTimeStep, resetFilters);
        resetFilters = false;
        yawControlAccumulator -= kFixedControlTimeStep;
    }

    return yawHeldOutput;
}

}}
