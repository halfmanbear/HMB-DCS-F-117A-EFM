#pragma once
// ============================================================================
// Weapons_Interop.h — IRADS target hand-off to DCS weapons
// ============================================================================
// Store launch goes straight to the aircraft's DCS weapon manager with the
// IRADS designated point as the launch target (JDAM coordinates / LGB initial
// aim point), instead of avSimpleWeaponSystem's default "10 km ahead" point.
// The laser uses the same native path as the Litening pod's rangefinder, so
// laser-guided bombs track it and it is network replicated.

namespace Weapons_interop
{
static constexpr unsigned int default_laser_code = 1688;

// True once the DCS weapon manager is reachable from the cockpit context.
bool is_launch_available();

// Launch one store from the explicitly selected station at the given world point.
// Never falls through to another station. Returns true if the
// weapon manager accepted a launch.
bool launch_at_point(const double world_point[3], int station);

// Stock guided-bomb time-of-flight predictor: flight time in seconds of a laser-guided bomb released at the
// given altitude, speed and flight-path angle (radians, positive up) toward a target at the given altitude
// and ground range. Returns false when the predictor is unavailable; *out is -1 when it finds no impact.
bool stock_laser_bomb_tof(double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                          double ground_range_m, double* out);

// Stock GPS-bomb time-of-flight predictor, same inputs; type selects the weapon variant (0-4).
bool stock_jdam_tof(int type, double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                    double ground_range_m, double* out);

// Create / move / remove the laser spot. Call every frame.
void update_laser(bool want_on, const double world_point[3], unsigned int code);
bool is_laser_firing();

// ed_fm_release: drop pointers. The spot is parented to the aircraft and
// dies with it, so it is not killed here.
void reset();
}  // namespace Weapons_interop
