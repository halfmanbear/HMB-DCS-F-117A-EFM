#pragma once
// State and helpers shared by the yaw, pitch and roll control-law units (Yaw.cpp, Pitch.cpp,
// Roll.cpp, PitchLog.cpp, FlightControls.cpp). Not part of the public control-law interface.
#include "Config.h"
#include "FlightControls.h"

namespace F117
{
namespace FLIGHTCONTROLS
{
namespace detail
{

// Latest values of the roll and yaw loops and the aero totals, recorded for the pitch debug log.
struct RollSnapshot
{
    double stickInput{};
    double stickForce{};
    double rateCommand{};
    double flatTurnCommand{};
    double rateCommandFiltered{};
    double commandGained{};
    double surfaceCommand{};
};

struct YawSnapshot
{
    double pedInput{};
    double rateDegS{};
    double rudderCommand{};
    double rudderCommandFiltered{};
    double pedalCommand{};
    double damping{};
    double sideAccelFeedback{};
    double ariCommand{};
    double combinedCommand{};
    double surfaceCommand{};
    double betaDeg{};
    double rateRps{};
    double rudderDegCommanded{};
    double rudderDeg{};
    double rudderPct{};
};

struct AeroSnapshot
{
    double cyDeltaRudder{};
    double cnDeltaRudder{};
    double clDeltaRudder{};
    double cnDeltaBeta{};
    double clDeltaBeta{};
    double cyTotal{};
    double cnTotal{};
    double clTotal{};
};

struct DebugSnapshot
{
    RollSnapshot roll;
    YawSnapshot yaw;
    AeroSnapshot aero;
};

inline DebugSnapshot debug_snapshot;

// Ground-contact fade: 1 on the ground, 0 in the air (see kGroundContactFadeRate).
inline double ground_contact_fade = 0.0;

// Adds elapsed frame time to a control-loop accumulator, capped so a long frame cannot trigger an
// unbounded burst of fixed-step iterations.
inline void advance_control_accumulator(double& accumulator, double frameTime_SEC)
{
    if (frameTime_SEC <= 0.0)
    {
        return;
    }

    accumulator += frameTime_SEC;
    if (accumulator > kMaxAccumulatedControlTime)
    {
        accumulator = kMaxAccumulatedControlTime;
    }
}

// Per-axis state reset, called from reset_runtime_state().
void reset_yaw_state();
void reset_pitch_state();
void reset_roll_state();

}  // namespace detail
}  // namespace FLIGHTCONTROLS
}  // namespace F117
