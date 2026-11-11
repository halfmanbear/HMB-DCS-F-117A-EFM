#pragma once

// Airframe damage state, the pending simulation-event queue and the damage debug log.

#include <queue>

#include "../F_117_NightHawk.h"
#include "../FlightModel/State.h"

inline F117::DamageState g_damage;
inline std::queue<ed_fm_simulation_event> g_simEvents;

// Queues a fire emitter at a body-space position unless that emitter is already burning.
void pushFireEvent(int handle, double x, double y, double z);

// Damage dealt by a Lua-reported hit, for aircraft that are not immortal.
void apply_lua_damage_hit();

// Refreshes the values derived from the component integrities. Called every frame.
void update_derived_damage_values();

// Flight-control authority left after cockpit damage, 1 intact down to 0.
double get_control_degradation();

// Puts out burning emitters, clears queued events and restores every component to intact.
void reset_damage_and_events();

// Marks a start or reset callback in the damage log.
void log_damage_lifecycle_event(const char* label);

// Periodic damage-state line in the damage log. acceleration_mps2 is the world acceleration magnitude.
void log_simulation_frame(double dt, double acceleration_mps2);
