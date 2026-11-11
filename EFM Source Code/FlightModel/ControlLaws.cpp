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
constexpr double kPitchLawCmPerDeg = 0.0105;  // Elevon pitch power the pitch law's gains were tuned with.
constexpr double kPitchDamperCmq = -5.0;      // Pitch damping the flight controls add, per unit of q c / 2V.

// Pitch authority: -25 deg TEU (nose-up) to +37.5 deg TED (nose-down), per Loschke. The physical actuator
// stop is 45 deg; pitch authority does not reach it.
constexpr double kElevatorNoseUpLimit_DEG = -25.0;
constexpr double kElevatorNoseDownLimit_DEG = 37.5;
constexpr double kElevonPhysicalStop_DEG = 45.0;

constexpr double kRudderLimit_DEG = 25.0;  // Lets full pedal reach 8 deg of steady sideslip (TO 1F-117A-1 page 6-3).
constexpr double kMinControlSpeed_MPS = 30.0;    // Floor for speed in the damper and turn-rate terms.
constexpr double kDefaultRollInertia = 40000.0;  // kg m^2, when DCS has not yet reported one
constexpr double kMaxPitchLawElevonScale = 8.0;
constexpr double kPitchTrimStickScale = 15.0;  // pitch trim units that add 1.0 to the elevator actuator input

// Rudder limiter: the high-beta, high-yaw-rate rudder limit stays symmetric left vs right.
constexpr double kRudderLimiterBetaScale = 7500.0;
constexpr double kRudderLimiterYawRateScale_DEGPS = 90.0;

// Autopilot holds: cockpit integrity below which they disengage, and the hold gains.
constexpr double kAutopilotCockpitIntegrity = 0.7;
constexpr double kHorizHoldPitchGain = 2.0;
constexpr double kHorizHoldRollGain = 0.1;
constexpr double kVerticalSpeedHoldLimit_MPS = 10.0;

// The pitch law's gains are in degrees of an elevon giving kPitchLawCmPerDeg of pitching moment. This
// turns its command into the deflection that gives the same moment on this airframe at this AOA, as the
// roll law does with its rolling moment.
double pitch_law_elevon_scale()
{
    return limit(kPitchLawCmPerDeg / (std::max)(std::abs(AERO::CmPerElevatorDeg), 1e-4), 1.0, kMaxPitchLawElevonScale);
}

// Pitch damper. The tailless airframe has little natural pitch damping (Cmq about -2.2). The manual
// describes the flight controls as giving essentially dead-beat damping, so the elevons add more.
// Nose-up pitch rate gives trailing edge down. In a steady pull the pitch law's integrator takes up
// whatever the damper adds, so the elevon there is set by the airframe's own moments and the damper
// costs no lift.
double pitch_damper_elevator_DEG()
{
    const double speed = (std::max)(totalVelocity_MPS, kMinControlSpeed_MPS);
    const double addedCm = kPitchDamperCmq * meanChord_M / (2.0 * speed) * pitchRate_RPS;
    return addedCm / (std::min)(AERO::CmPerElevatorDeg, -1e-4);
}

void command_pitch_axis(double dt, double controlDegradation)
{
    FLIGHTCONTROLS::update_pitch_mode_auto_inputs(gearDown, airRefuelDoorOpen);
    // Suspension contact only, with the gear down. The strut draw args in weight_on_wheels aren't used here.
    FLIGHTCONTROLS::update_ground_contact(
        gearDown > 0.5 && (suspension_wow[0] || suspension_wow[1] || suspension_wow[2]), dt);

    elevator_DEG_commanded =
        -(FLIGHTCONTROLS::fcs_pitch_controller(
            FLIGHTCONTROLS::longStickInput * controlDegradation, 0.0, alpha_DEG, pitchRate_RPS * radiansToDegrees,
            (az / kStandardGravity_MPS2), 0.0, dynamicPressure_PA, dt, roll_angle, pitch_angle,
            rollRate_RPS * radiansToDegrees, totalVelocity_MPS, mach, thrust_N, AERO::Cx_total)) *
        pitch_law_elevon_scale();
    elevator_DEG_commanded += pitch_damper_elevator_DEG();
    elevator_DEG = limit(elevator_DEG_commanded + pitchTrim, kElevatorNoseUpLimit_DEG, kElevatorNoseDownLimit_DEG);
}

void command_roll_axis(double dt, double controlDegradation)
{
    // What the roll law needs to know about the airframe at this flight condition. It uses the inertia DCS is
    // flying, so changing moment_of_inertia in the mod's FM/config.lua doesn't change the loop's bandwidth.
    const double rollMomentPerSurfaceDeg = (std::max)(std::abs(AERO::ClPerAileronDeg), 0.0002);
    const double rollInertia = inertia.x > 1.0 ? inertia.x : kDefaultRollInertia;
    const double rollAccelPerSurfaceDeg =
        rollMomentPerSurfaceDeg * dynamicPressure_PA * wingArea_M2 * wingSpan_M / rollInertia * radiansToDegrees;
    const double surfacePerRollRate = std::abs(AERO::Clp) * wingSpan_M /
                                      (2.0 * (std::max)(totalVelocity_MPS, kMinControlSpeed_MPS)) / radiansToDegrees /
                                      rollMomentPerSurfaceDeg;

    aileron_DEG_commanded = FLIGHTCONTROLS::fcs_roll_controller(
        FLIGHTCONTROLS::latStickInput * controlDegradation, FLIGHTCONTROLS::longStickForce, ay / kStandardGravity_MPS2,
        pedInput, beta_DEG, roll_angle, rollRate_RPS * radiansToDegrees,
        g_paarsRollRateDegS,  // PAARS roll-rate command, added to the pilot's
        dynamicPressure_PA, rollAccelPerSurfaceDeg, surfacePerRollRate, gearDown > 0.5, airRefuelDoorOpen, dt);

    // Real F-117 pitch/roll mixer: pitch has priority.
    // Each elevon physical stop is 45 deg. Pitch occupies elevator_DEG of that travel.
    // Roll gets whatever remains: max roll deflection = 45 deg - |elevator_DEG|.
    // This means rolling pull-outs automatically have less roll authority at high pitch demand,
    // exactly as the real aircraft (and consistent with Loschke's mixer description).
    const double maxRollDEG = (std::max)(0.0, kElevonPhysicalStop_DEG - std::abs(elevator_DEG));
    aileron_DEG = limit(aileron_DEG_commanded + rollTrim, -maxRollDEG, maxRollDEG);
}

// Body yaw rate of a steady level turn at this bank and pitch: g sin(bank) cos(pitch) / V.
double level_turn_yaw_rate_DEGPS()
{
    return kStandardGravity_MPS2 * std::sin(roll_angle / radiansToDegrees) * std::cos(pitch_angle / radiansToDegrees) /
           (std::max)(totalVelocity_MPS, kMinControlSpeed_MPS) * radiansToDegrees;
}

void command_yaw_axis(double dt)
{
    // Keep the high-beta/high-yaw-rate rudder limiter symmetric left vs right.
    const double betaMagnitude_DEG = std::abs(beta_DEG);
    const double yawRateMagnitude_DEG_s = std::abs(yawRate_RPS * radiansToDegrees);
    const float rudderLimiterScale =
        static_cast<float>(1.0 + (betaMagnitude_DEG * betaMagnitude_DEG) / kRudderLimiterBetaScale *
                                     (1.0 + yawRateMagnitude_DEG_s / kRudderLimiterYawRateScale_DEGPS));

    rudder_DEG_commanded = FLIGHTCONTROLS::fcs_yaw_controller(
        pedInput, 0.0, beta_DEG, yawRate_RPS * radiansToDegrees, level_turn_yaw_rate_DEGPS(),
        rollRate_RPS * radiansToDegrees, pitchRate_RPS * radiansToDegrees, FLIGHTCONTROLS::alphaFiltered,
        dynamicPressure_PA, gearDown > 0.5, dt);
    rudder_DEG = yawTrim + rudder_DEG_commanded;
    rudder_DEG = limit(rudder_DEG, -kRudderLimit_DEG / rudderLimiterScale, kRudderLimit_DEG / rudderLimiterScale);
}
}  // namespace

void update_autopilot_hold_state()
{
    if (g_damage.cockpit <= kAutopilotCockpitIntegrity)
    {
        alt_hold = horiz_hold = altroll_hold = 0;
        return;
    }
    if (horiz_hold)
        pitchTrim = kHorizHoldPitchGain * pitch_angle;
    else if (alt_hold || altroll_hold)
        pitchTrim = limit(vspeed, -kVerticalSpeedHoldLimit_MPS, kVerticalSpeedHoldLimit_MPS);
    if (horiz_hold || altroll_hold)
        rollTrim = kHorizHoldRollGain * roll_angle;
}

void update_flight_control_commands(double dt, double controlDegradation)
{
    command_pitch_axis(dt, controlDegradation);
    command_roll_axis(dt, controlDegradation);

    dragchute = ACTUATORS::drive_drag_chute(dragchute_command, dt, totalVelocity_MPS, weight_on_wheels);

    command_yaw_axis(dt);
}

double update_airframe_actuators(double dt)
{
    elev_pos = static_cast<float>(
        ACTUATORS::drive(ACTUATORS::elevator, FLIGHTCONTROLS::longStickInput + (pitchTrim / kPitchTrimStickScale), dt));

    const double tailIntegrity = (g_damage.leftTail + g_damage.rightTail) / 2.0;
    rudder_pos = static_cast<float>(ACTUATORS::drive(ACTUATORS::rudder, pedInput, dt) * tailIntegrity);

    const double noseWheelRangeTarget = nwsAuthorityHeld ? kNoseWheelYawHigh_RAD : kNoseWheelYawLow_RAD;
    const double noseWheelRangeStep = kNoseWheelYawRangeRate_RADPS * dt;
    noseWheelYawRange_RAD +=
        limit(noseWheelRangeTarget - noseWheelYawRange_RAD, -noseWheelRangeStep, noseWheelRangeStep);

    GearCommand = clamp_gear_command_for_weight_on_wheels(GearCommand);
    gearDown = ACTUATORS::drive_gear(GearCommand, dt, gear_retraction_blocked());

    tailhook_pos = ACTUATORS::drive(ACTUATORS::tailhook, tailhook_command, dt);
    for (int bay = 0; bay < kWeaponBayCount; ++bay)
        bay_state[bay] = static_cast<float>(ACTUATORS::drive(ACTUATORS::weaponBay[bay], bay_cmd[bay], dt));

    return tailIntegrity;
}

}  // namespace F117
