#pragma once
// State and helpers shared by the yaw, pitch and roll control-law units (Yaw.cpp, Pitch.cpp,
// Roll.cpp, PitchLog.cpp, FlightControls.cpp). Not part of the public control-law interface.
#include "Config.h"
#include "FlightControls.h"

namespace F117
{
namespace Flight_controls
{
namespace Detail
{

// Latest values of the roll and yaw loops and the aero totals, recorded for the pitch debug log.
struct Roll_snapshot
{
    double stick_input{};
    double stick_force{};
    double rate_command{};
    double flat_turn_command{};
    double rate_command_filtered{};
    double command_gained{};
    double surface_command{};
};

struct Yaw_snapshot
{
    double ped_input{};
    double rate_deg_s{};
    double rudder_command{};
    double rudder_command_filtered{};
    double pedal_command{};
    double damping{};
    double side_accel_feedback{};
    double ari_command{};
    double combined_command{};
    double surface_command{};
    double beta_deg{};
    double rate_rps{};
    double rudder_deg_commanded{};
    double rudder_deg{};
    double rudder_pct{};
};

struct Aero_snapshot
{
    double cy_delta_rudder{};
    double cn_delta_rudder{};
    double cl_delta_rudder{};
    double cn_delta_beta{};
    double cl_delta_beta{};
    double cy_total{};
    double cn_total{};
    double cl_total{};
};

struct Debug_snapshot
{
    Roll_snapshot roll;
    Yaw_snapshot yaw;
    Aero_snapshot aero;
};

inline Debug_snapshot debug_snapshot;

// Ground-contact fade: 1 on the ground, 0 in the air (see kGroundContactFadeRate).
inline double ground_contact_fade = 0.0;

// Adds elapsed frame time to a control-loop accumulator, capped so a long frame cannot trigger an
// unbounded burst of fixed-step iterations.
inline void advance_control_accumulator(double& accumulator, double frame_time_sec)
{
    if (frame_time_sec <= 0.0)
    {
        return;
    }

    accumulator += frame_time_sec;
    if (accumulator > max_accumulated_control_time)
    {
        accumulator = max_accumulated_control_time;
    }
}

// Per-axis state reset, called from reset_runtime_state().
void reset_yaw_state();
void reset_pitch_state();
void reset_roll_state();

}  // namespace Detail
}  // namespace Flight_controls
}  // namespace F117
