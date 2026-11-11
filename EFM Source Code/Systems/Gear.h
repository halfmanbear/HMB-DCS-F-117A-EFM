#pragma once

// Landing gear command rules and weight-on-wheels state.

inline constexpr double gear_command_down = 1.0;
inline constexpr double gear_command_up = 0.0;
inline constexpr double gear_toggle_threshold = 0.5;
inline constexpr double weight_on_wheels_draw_arg_threshold = 0.5;

bool gear_retraction_blocked();
double clamp_gear_command_for_weight_on_wheels(double requested_command);
void set_gear_command(double requested_command);
void update_weight_on_wheels();
