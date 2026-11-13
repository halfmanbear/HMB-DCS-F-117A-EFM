#pragma once

// Pilot command handlers (flight controls, engines, actuators, autopilot, A/T and PAARS) and their per-frame updates.

inline constexpr double roll_trim_step = 0.02;
inline constexpr double pitch_trim_step = 0.05;
inline constexpr double yaw_trim_step = 0.05;
inline constexpr double throttle_step = 0.5;
// Fraction of wheel_brake_moment_max in FM/config.lua. Set by the landing distances in TO 1F-117A-1
// (Figure A8-2), checked with tools/fm_bench/dcs_check.py --landing in the mod repo.
inline constexpr double wheel_brake_on_friction = 0.32;
inline constexpr double wheel_brake_off_friction = 0.015;
inline constexpr double toggle_low_threshold = 0.25;
inline constexpr double toggle_high_threshold = 0.75;

bool handle_roll_command(int command, float value);
bool handle_pitch_command(int command, float value);
bool handle_yaw_command(int command, float value);
bool handle_engine_and_throttle_command(int command, float value);
// Sets both engine throttle inputs and the combined lever to one position (percent).
void set_throttle_levers(double percent);
bool handle_actuator_toggle_command(int command);
bool handle_autopilot_command(int command);
bool engines_running();
bool handle_autothrottle_command(int command);
void release_paars();
bool handle_paars_command(int command);
void update_paars();
void update_autothrottle(double dt);
