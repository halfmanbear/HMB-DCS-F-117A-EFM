#pragma once
// Mass-state log (F117_Mass_State.log in the DCS Logs folder): the mass, CG and inertia DCS reports, the fuel
// pieces the EFM hands it, and the callbacks around them, to check how DCS assembles the aircraft's mass.

// Logs the first state of each aircraft and every later change larger than fuel burn noise.
void mass_log_state(double mass_kg, double cg_x, double cg_y, double cg_z, double roll_inertia, double yaw_inertia,
                    double pitch_inertia);

// Logs a fuel piece of 20 kg or more reported to DCS; tank is its Fuel_tanks index.
void mass_log_piece(int tank, double mass_kg, double x, double y, double z, double roll_inertia, double yaw_inertia,
                    double pitch_inertia);

// One row for a callback the mass state depends on, such as a spawn or a fuel load.
void mass_log_event(const char* name, double value);

// Ends the aircraft: logs it, and makes the next aircraft log its first state even if it matches.
void mass_log_release(double fuel_kg);
