#pragma once

// Landing gear command rules and weight-on-wheels state.

inline constexpr double kGearCommandDown = 1.0;
inline constexpr double kGearCommandUp = 0.0;
inline constexpr double kGearToggleThreshold = 0.5;
inline constexpr double kWeightOnWheelsDrawArgThreshold = 0.5;

bool gear_retraction_blocked();
double clamp_gear_command_for_weight_on_wheels(double requestedCommand);
void set_gear_command(double requestedCommand);
void update_weight_on_wheels();
