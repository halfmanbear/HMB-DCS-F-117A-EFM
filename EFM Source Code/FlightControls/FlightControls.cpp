// Public state and shared services of the flight control laws. The laws themselves live in
// Yaw.cpp, Pitch.cpp and Roll.cpp; the pitch debug CSV in PitchLog.cpp.
#include "stdafx.h"
#include "FlightControls.h"
#include "Internal.h"

namespace F117
{
namespace FLIGHTCONTROLS
{

bool simInitialized{};
double longStickInput{};
double alphaFiltered{};
double longStickForce{};
double latStickInput{};

void reset_runtime_state()
{
    simInitialized = false;
    alphaFiltered = 0.0;
    longStickForce = 0.0;

    detail::debug_snapshot = detail::DebugSnapshot{};
    detail::reset_yaw_state();
    detail::reset_pitch_state();
    detail::reset_roll_state();
}

void update_ground_contact(bool wheelOnGround, double dt)
{
    const double change = kGroundContactFadeRate * dt;
    detail::ground_contact_fade = limit(detail::ground_contact_fade + (wheelOnGround ? change : -change), 0.0, 1.0);
}

void update_yaw_debug_snapshot(double betaDeg, double yawRateRps, double rudderDegCommanded, double rudderDeg,
                               double rudderPct)
{
    detail::YawSnapshot& yaw = detail::debug_snapshot.yaw;
    yaw.betaDeg = betaDeg;
    yaw.rateRps = yawRateRps;
    yaw.rudderDegCommanded = rudderDegCommanded;
    yaw.rudderDeg = rudderDeg;
    yaw.rudderPct = rudderPct;
}

void update_aero_debug_snapshot(double cyDeltaRudder, double cnDeltaRudder, double clDeltaRudder, double cnDeltaBeta,
                                double clDeltaBeta, double cyTotal, double cnTotal, double clTotal)
{
    detail::AeroSnapshot& aero = detail::debug_snapshot.aero;
    aero.cyDeltaRudder = cyDeltaRudder;
    aero.cnDeltaRudder = cnDeltaRudder;
    aero.clDeltaRudder = clDeltaRudder;
    aero.cnDeltaBeta = cnDeltaBeta;
    aero.clDeltaBeta = clDeltaBeta;
    aero.cyTotal = cyTotal;
    aero.cnTotal = cnTotal;
    aero.clTotal = clTotal;
}

}  // namespace FLIGHTCONTROLS
}  // namespace F117
