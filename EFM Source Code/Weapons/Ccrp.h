#pragma once

// Continuously computed release point (CCRP) model for the bay stores, and the cockpit parameters that
// publish its release cue and the IRADS lock.

#include <cstddef>

#include "../EfmMath.h"

namespace Weapons
{
// How a store decides it is in its release zone.
//  Jdam      GPS bomb, stock DCS min/max release range table
//  Lgb       laser-guided bomb, window around the ballistic impact, laser must fire
//  Unguided  free-fall bomb, released on the CCRP cue
//  Wcmd      wind-corrected dispenser, wide window around the ballistic impact
//  Glide     glide weapon (JSOW), stock DCS min/max release range table
//  Missile*  powered missile, slant range and seeker cone
enum class Store_kind
{
    jdam,
    lgb,
    unguided,
    wcmd,
    glide,
    missile_ir,
    missile_laser
};

struct Ccrp_store_model
{
    const char* name;
    double mass_kg;
    double drag_cd;
    double reference_area_m2;
    double release_window_meters;
    double min_speed_mps;
    double min_drop_meters;
    Store_kind kind;
    int envelope;                  // JdamEnvelope / GlideEnvelope table, or -1
    double min_range_meters;       // missiles only
    double max_range_meters;       // missiles only
    double seeker_half_angle_rad;  // missiles only
};

// Index = F117_CCRP_PROFILE, set by Cockpit/Scripts/Systems/Weapons.lua classify().
// Keep the two lists in the same order. Profiles 0-4 must never move.
// Stores 5-13 take mass and caliber from the stock DCS weapon tables; missile
// ranges and seeker cones come from the stock AGM-65 definitions. The MK-82 and
// MK-84 drag coefficients are fitted to impacts measured in DCS from 4500 m at
// 218 m/s (tools/missions/store_drop_test.py in the mod repo).
constexpr Ccrp_store_model ccrp_store_models[] = {
    {"GBU-31(V)1/B", 934.0, 0.00264, 0.45, 75.0, 30.0, 10.0, Store_kind::jdam, 0, 0.0, 0.0, 0.0},
    {"GBU-31(V)3/B", 981.0, 0.00170, 0.45, 90.0, 30.0, 10.0, Store_kind::jdam, 1, 0.0, 0.0, 0.0},
    {"GBU-32(V)2/B", 467.0, 0.00035, 0.30, 65.0, 30.0, 10.0, Store_kind::jdam, 2, 0.0, 0.0, 0.0},
    {"GBU-12", 277.0, 0.000413, 0.22, 55.0, 30.0, 10.0, Store_kind::lgb, -1, 0.0, 0.0, 0.0},
    {"GBU-27", 991.0, 0.00071, 0.45, 80.0, 30.0, 10.0, Store_kind::lgb, -1, 0.0, 0.0, 0.0},
    {"GBU-10", 959.0, 0.00071, 0.45, 80.0, 30.0, 10.0, Store_kind::lgb, -1, 0.0, 0.0, 0.0},
    {"MK-82", 228.0, 0.151, 0.0585, 15.0, 30.0, 10.0, Store_kind::unguided, -1, 0.0, 0.0, 0.0},
    {"MK-84", 908.0, 0.256, 0.164, 20.0, 30.0, 10.0, Store_kind::unguided, -1, 0.0, 0.0, 0.0},
    {"CBU-105", 420.0, 0.55, 0.1233, 150.0, 30.0, 300.0, Store_kind::wcmd, -1, 0.0, 0.0, 0.0},
    {"AGM-154A", 485.0, 0.0018, 0.1257, 0.0, 30.0, 300.0, Store_kind::glide, 0, 0.0, 0.0, 0.0},
    {"AGM-154B", 485.0, 0.0018, 0.1257, 0.0, 30.0, 300.0, Store_kind::glide, 1, 0.0, 0.0, 0.0},
    {"AGM-154C", 484.0, 0.0018, 0.1257, 0.0, 30.0, 300.0, Store_kind::glide, 0, 0.0, 0.0, 0.0},
    {"AGM-65D", 218.0, 0.0, 0.073, 0.0, 30.0, 10.0, Store_kind::missile_ir, -1, 500.0, 12964.0, 0.5},
    {"AGM-65E", 293.0, 0.0, 0.073, 0.0, 30.0, 10.0, Store_kind::missile_laser, -1, 1000.0, 11112.0, 0.5},
};
constexpr int ccrp_store_count = static_cast<int>(sizeof(ccrp_store_models) / sizeof(ccrp_store_models[0]));

constexpr bool store_needs_laser(Store_kind kind)
{
    return kind == Store_kind::lgb || kind == Store_kind::missile_laser;
}
constexpr bool store_is_missile(Store_kind kind)
{
    return kind == Store_kind::missile_ir || kind == Store_kind::missile_laser;
}
// Bombs whose impact point comes from the ballistic integration below.
constexpr bool store_falls_ballistic(Store_kind kind)
{
    return kind != Store_kind::glide && !store_is_missile(kind);
}

constexpr double gravity_mps2 = 9.81;
constexpr double release_cue_min_speed_mps = 30.0;
constexpr double release_cue_min_drop_meters = 10.0;
constexpr double atmosphere_scale_height_m = 8500.0;
constexpr double release_integration_step_seconds = 0.02;
constexpr double release_integration_max_seconds = 180.0;
constexpr double unguided_max_cross_track_meters = 30.0;  // farther off the line misses the target
// JSOW average ground speed over the flight, for the flight timer. Measured on AGM-154A/B drops from 5.8 km above the
// target at 23 km and 240 m/s (93-95 s); not checked at other ranges or heights.
constexpr double glide_weapon_speed_mps = 246.0;
constexpr double missile_average_speed_mps = 300.0;       // AGM-65 average speed, for the laser/flight timer

constexpr double weapon_bay_open_threshold = 0.995;
constexpr double weapon_bay_closed_threshold = 0.005;
constexpr double weapon_bay_hold_after_release_seconds = 1.0;
constexpr double weapon_release_gate_min_hold_seconds =
    0.25;                                      // Long enough for cockpit Lua update rates to sample the gate.
constexpr double laser_overrun_seconds = 4.0;  // keep lasing this long past predicted impact
constexpr char efm_launch_active_param_name[] = "F117_EFM_LAUNCH_ACTIVE";
constexpr char laser_armed_param_name[] = "F117_LASER_ARMED";
constexpr char laser_firing_param_name[] = "F117_LASER_FIRING";
constexpr char weapon_release_gate_param_name[] = "F117_WEAPON_RELEASE_GATE";
constexpr char irads_lock_valid_param_name[] = "F117_IRADS_LOCK_VALID";
constexpr char irads_lock_world_x_param_name[] = "F117_IRADS_LOCK_X";
constexpr char irads_lock_world_y_param_name[] = "F117_IRADS_LOCK_Y";
constexpr char irads_lock_world_z_param_name[] = "F117_IRADS_LOCK_Z";
constexpr char irads_lock_range_param_name[] = "F117_IRADS_LOCK_RANGE";
constexpr char irads_lock_azimuth_param_name[] = "F117_IRADS_LOCK_AZ";
constexpr char irads_lock_elevation_param_name[] = "F117_IRADS_LOCK_EL";
constexpr char release_cue_valid_param_name[] = "F117_RELEASE_CUE_VALID";
constexpr char release_cue_error_param_name[] = "F117_RELEASE_CUE_ERROR";
constexpr char release_cue_time_to_go_param_name[] = "F117_RELEASE_CUE_TTG";
constexpr char release_cue_range_param_name[] = "F117_RELEASE_CUE_RANGE";
constexpr char release_cue_in_zone_param_name[] = "F117_RELEASE_CUE_IN_ZONE";
constexpr char ccrp_profile_param_name[] = "F117_CCRP_PROFILE";

// Release alignment from the last solution, read by the release sequence.
extern bool g_release_alignment_valid;
extern double g_release_min_range, g_release_max_range;

// Release geometry from the last solution (height above the target, ground range, ground speed, climb rate).
struct Release_snapshot
{
    bool valid;
    double drop_meters;
    double horiz_range;
    double horiz_speed;
    double vertical_speed;
    double air_speed;
    double free_fall_tof;  // gravity and drag only
    double stock_tof;      // release-time estimate (stock JDAM, or LGB ballistic fall), 0 when not used
};
extern Release_snapshot g_release_snapshot;

// At the moment of release: replaces the flight time with the stock GPS-bomb estimate (JDAM), which takes longer to
// converge than a per-frame call allows, or with the char_time ballistic fall (laser-guided bombs). Leaves the time
// unchanged when the estimate is unavailable.
void refine_release_time_of_flight();

void publish_weapon_release_gate_param();
int get_ccrp_profile_index();
const Ccrp_store_model& get_ccrp_profile();
bool compute_release_solution(double* out_error_meters, double* out_time_to_go_seconds, double* out_range_meters,
                              bool* out_in_zone);
void publish_release_cue_params();
void publish_irads_lock_params();
}  // namespace Weapons
