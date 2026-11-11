#pragma once

// Aerodynamic coefficient assembly and the force and moment application for one simulation frame.

double induced_drag_ground_effect();
double lift_ground_effect_gain();
void update_aerodynamic_coefficients();
void apply_aerodynamic_forces_and_thrust();
