// Public state and shared services of the flight control laws. The laws themselves live in
// Yaw.cpp, Pitch.cpp and Roll.cpp; the pitch debug CSV in PitchLog.cpp.
#include "stdafx.h"
#include "FlightControls.h"
#include "Internal.h"

namespace F117
{
namespace Flight_controls
{

bool sim_initialized{};
double long_stick_input{};
double alpha_filtered{};
double long_stick_force{};
double lat_stick_input{};

void reset_runtime_state()
{
    sim_initialized = false;
    alpha_filtered = 0.0;
    long_stick_force = 0.0;

    Detail::debug_snapshot = Detail::Debug_snapshot{};
    Detail::reset_yaw_state();
    Detail::reset_pitch_state();
    Detail::reset_roll_state();
}

void update_ground_contact(bool wheel_on_ground, double dt)
{
    const double change = ground_contact_fade_rate * dt;
    Detail::ground_contact_fade = limit(Detail::ground_contact_fade + (wheel_on_ground ? change : -change), 0.0, 1.0);
}

void update_yaw_debug_snapshot(double beta_deg, double yaw_rate_rps, double rudder_deg_commanded, double rudder_deg,
                               double rudder_pct)
{
    Detail::Yaw_snapshot& yaw = Detail::debug_snapshot.yaw;
    yaw.beta_deg = beta_deg;
    yaw.rate_rps = yaw_rate_rps;
    yaw.rudder_deg_commanded = rudder_deg_commanded;
    yaw.rudder_deg = rudder_deg;
    yaw.rudder_pct = rudder_pct;
}

void update_aero_debug_snapshot(const Aero_snapshot& aero)
{
    Detail::debug_snapshot.aero = aero;
}

}  // namespace Flight_controls
}  // namespace F117
