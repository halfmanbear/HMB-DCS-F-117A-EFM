#pragma once

// Per-frame control-law drive: pilot inputs through the pitch, roll and yaw laws to commanded
// surface deflections, and the airframe actuator update that follows.

namespace F117 {

// Autopilot attitude and altitude holds write the trims; they are disabled by cockpit damage.
void update_autopilot_hold_state();

// Runs the pitch, roll and yaw control laws and the pitch/roll mixer. `controlDegradation` is the
// 0..1 pilot-input scale from cockpit damage.
void update_flight_control_commands(double dt, double controlDegradation);

// Drives the surface, gear, hook and bay actuators. Returns the tail integrity (0..1), which the
// propulsion update also needs.
double update_airframe_actuators(double dt);

} // namespace F117
