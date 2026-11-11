#pragma once

// Fuel burn, engine indications and propulsion effectiveness for one simulation frame.

void update_fuel_system(double dt);
void update_engine_indications(double dt);
void publish_engine_start_params();
void update_propulsion_and_control_effectiveness(double dt, double tailIntegrity);
