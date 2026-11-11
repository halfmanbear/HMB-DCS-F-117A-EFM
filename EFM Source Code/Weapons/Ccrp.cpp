#include "stdafx.h"
#include "Ccrp.h"

#include <algorithm>
#include <cmath>

#include "../Avionics/CockpitBase_Interop.h"
#include "../Avionics/JdamEnvelope.h"
#include "../Avionics/Weapons_Interop.h"
#include "../Cockpit/ParamBridge.h"
#include "../FlightModel/State.h"

static void* g_weapon_release_gate_param = nullptr;
static bool g_weapon_release_gate_param_initialized = false;
static void* g_irads_lock_valid_param = nullptr;
static void* g_irads_lock_world_x_param = nullptr;
static void* g_irads_lock_world_y_param = nullptr;
static void* g_irads_lock_world_z_param = nullptr;
static void* g_irads_lock_range_param = nullptr;
static void* g_irads_lock_azimuth_param = nullptr;
static void* g_irads_lock_elevation_param = nullptr;
static bool g_irads_lock_params_initialized = false;
static void* g_release_cue_valid_param = nullptr;
static void* g_release_cue_error_param = nullptr;
static void* g_release_cue_time_to_go_param = nullptr;
static void* g_release_cue_range_param = nullptr;
static void* g_release_cue_in_zone_param = nullptr;
static bool g_release_cue_params_initialized = false;
static void* g_ccrp_profile_param = nullptr;
static bool g_ccrp_profile_param_initialized = false;

namespace Weapons
{
void initialize_weapon_release_gate_param()
{
    if (g_weapon_release_gate_param_initialized)
    {
        return;
    }

    F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
    if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
        F117::cockpit_api.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
    {
        return;
    }

    g_weapon_release_gate_param = F117::cockpit_api.getParamHandle(weapon_release_gate_param_name);
    g_weapon_release_gate_param_initialized = (g_weapon_release_gate_param != nullptr);
}

void publish_weapon_release_gate_param()
{
    initialize_weapon_release_gate_param();
    if (g_weapon_release_gate_param != nullptr)
    {
        F117::cockpit_api.setParamNumber(g_weapon_release_gate_param, F117::weapon_release_gate ? 1.0 : 0.0);
    }
}

void initialize_irads_lock_params()
{
    if (g_irads_lock_params_initialized)
    {
        return;
    }

    F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
    if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
        F117::cockpit_api.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
    {
        return;
    }

    g_irads_lock_valid_param = F117::cockpit_api.getParamHandle(irads_lock_valid_param_name);
    g_irads_lock_world_x_param = F117::cockpit_api.getParamHandle(irads_lock_world_x_param_name);
    g_irads_lock_world_y_param = F117::cockpit_api.getParamHandle(irads_lock_world_y_param_name);
    g_irads_lock_world_z_param = F117::cockpit_api.getParamHandle(irads_lock_world_z_param_name);
    g_irads_lock_range_param = F117::cockpit_api.getParamHandle(irads_lock_range_param_name);
    g_irads_lock_azimuth_param = F117::cockpit_api.getParamHandle(irads_lock_azimuth_param_name);
    g_irads_lock_elevation_param = F117::cockpit_api.getParamHandle(irads_lock_elevation_param_name);
    g_irads_lock_params_initialized =
        (g_irads_lock_valid_param != nullptr) && (g_irads_lock_world_x_param != nullptr) &&
        (g_irads_lock_world_y_param != nullptr) && (g_irads_lock_world_z_param != nullptr) &&
        (g_irads_lock_range_param != nullptr) && (g_irads_lock_azimuth_param != nullptr) &&
        (g_irads_lock_elevation_param != nullptr);
}

void initialize_release_cue_params()
{
    if (g_release_cue_params_initialized)
    {
        return;
    }

    F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
    if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
        F117::cockpit_api.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
    {
        return;
    }

    g_release_cue_valid_param = F117::cockpit_api.getParamHandle(release_cue_valid_param_name);
    g_release_cue_error_param = F117::cockpit_api.getParamHandle(release_cue_error_param_name);
    g_release_cue_time_to_go_param = F117::cockpit_api.getParamHandle(release_cue_time_to_go_param_name);
    g_release_cue_range_param = F117::cockpit_api.getParamHandle(release_cue_range_param_name);
    g_release_cue_in_zone_param = F117::cockpit_api.getParamHandle(release_cue_in_zone_param_name);
    g_release_cue_params_initialized =
        (g_release_cue_valid_param != nullptr) && (g_release_cue_error_param != nullptr) &&
        (g_release_cue_time_to_go_param != nullptr) && (g_release_cue_range_param != nullptr) &&
        (g_release_cue_in_zone_param != nullptr);
}

void initialize_ccrp_profile_param()
{
    if (g_ccrp_profile_param_initialized)
    {
        return;
    }

    F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
    if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
        F117::cockpit_api.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
    {
        return;
    }

    g_ccrp_profile_param = F117::cockpit_api.getParamHandle(ccrp_profile_param_name);
    g_ccrp_profile_param_initialized = (g_ccrp_profile_param != nullptr);
}

int get_ccrp_profile_index()
{
    initialize_ccrp_profile_param();
    if (!g_ccrp_profile_param_initialized)
    {
        return 0;
    }

    const double raw_index = F117::cockpit_api.getParamNumber(g_ccrp_profile_param);
    int profile_index = std::isfinite(raw_index) && raw_index >= 0.0 && raw_index < ccrp_store_count
                            ? static_cast<int>(std::floor(raw_index + 0.5))
                            : 0;
    const int profile_count = ccrp_store_count;
    if (profile_index < 0)
    {
        profile_index = 0;
    }
    if (profile_index >= profile_count)
    {
        profile_index = profile_count - 1;
    }

    return profile_index;
}

const Ccrp_store_model& get_ccrp_profile()
{
    return ccrp_store_models[get_ccrp_profile_index()];
}

Vec3 get_world_wind_vector()
{
    return wind_world_cs;
}

bool simulate_bomb_impact_point(const Ccrp_store_model& profile, const Vec3& release_pos, const Vec3& release_vel,
                                const Vec3& world_wind, double target_alt_meters, Vec3* out_impact_pos,
                                double* out_impact_time)
{
    if (out_impact_pos == nullptr || out_impact_time == nullptr || profile.mass_kg <= 0.0 ||
        profile.reference_area_m2 <= 0.0)
    {
        return false;
    }

    if (release_pos.y <= target_alt_meters)
    {
        return false;
    }

    const double altitude_reference = std::max(release_pos.y, 0.0);
    const double sea_level_density =
        std::max(F117::ambient_density_kg_per_m3, 0.05) * exp(altitude_reference / atmosphere_scale_height_m);

    Vec3 pos = release_pos;
    Vec3 vel = release_vel;
    Vec3 prev_pos = pos;
    double elapsed = 0.0;

    while (elapsed < release_integration_max_seconds)
    {
        prev_pos = pos;

        Vec3 rel_vel{};
        rel_vel.x = vel.x - world_wind.x;
        rel_vel.y = vel.y - world_wind.y;
        rel_vel.z = vel.z - world_wind.z;

        const double rel_speed = sqrt((rel_vel.x * rel_vel.x) + (rel_vel.y * rel_vel.y) + (rel_vel.z * rel_vel.z));
        Vec3 acc{};
        acc.x = 0.0;
        acc.y = -gravity_mps2;
        acc.z = 0.0;

        if (rel_speed > 0.25)
        {
            const double rho = sea_level_density * exp(-std::max(pos.y, 0.0) / atmosphere_scale_height_m);
            const double drag_scale = 0.5 * rho * profile.drag_cd * profile.reference_area_m2 / profile.mass_kg;
            const double drag_accel = drag_scale * rel_speed * rel_speed;
            const double inv_rel_speed = 1.0 / rel_speed;
            acc.x -= drag_accel * rel_vel.x * inv_rel_speed;
            acc.y -= drag_accel * rel_vel.y * inv_rel_speed;
            acc.z -= drag_accel * rel_vel.z * inv_rel_speed;
        }

        vel.x += acc.x * release_integration_step_seconds;
        vel.y += acc.y * release_integration_step_seconds;
        vel.z += acc.z * release_integration_step_seconds;

        pos.x += vel.x * release_integration_step_seconds;
        pos.y += vel.y * release_integration_step_seconds;
        pos.z += vel.z * release_integration_step_seconds;
        elapsed += release_integration_step_seconds;

        if (pos.y <= target_alt_meters)
        {
            const double denom = prev_pos.y - pos.y;
            const double fraction = (fabs(denom) > 1.0e-6) ? ((prev_pos.y - target_alt_meters) / denom) : 1.0;
            out_impact_pos->x = prev_pos.x + ((pos.x - prev_pos.x) * fraction);
            out_impact_pos->y = target_alt_meters;
            out_impact_pos->z = prev_pos.z + ((pos.z - prev_pos.z) * fraction);
            *out_impact_time = std::max(0.0, elapsed - release_integration_step_seconds +
                                                 (release_integration_step_seconds * fraction));
            return true;
        }
    }

    return false;
}

bool g_release_alignment_valid = false;
double g_release_min_range = 0.0;
double g_release_max_range = 0.0;

// Provisional guided-store envelope around the ballistic prediction, not a
// measured DCS DLZ. Keep the ballistic error/TTG as cues, not a pinpoint gate.
bool guided_release_in_zone(Store_kind kind, double ballistic_range, double along_track, double cross_track,
                            double minimum_window, bool* aligned)
{
    if (!aligned)
        return false;
    *aligned = false;
    if (!store_falls_ballistic(kind) || kind == Store_kind::unguided || !std::isfinite(ballistic_range) ||
        !std::isfinite(along_track) || !std::isfinite(cross_track) || !std::isfinite(minimum_window) ||
        ballistic_range <= 0.0 || along_track <= 0.0 || cross_track < 0.0 || minimum_window < 0.0)
        return false;
    const bool jdam = kind == Store_kind::jdam;
    const double lateral_ratio = jdam ? 0.577350269 : 0.267949192;  // 30 / 15 degrees
    *aligned = cross_track <= std::max(50.0, along_track * lateral_ratio);
    const double range_margin = std::max(minimum_window, ballistic_range * (jdam ? 0.35 : 0.20));
    const double horizontal_range = sqrt(along_track * along_track + cross_track * cross_track);
    return *aligned && along_track >= std::max(1.0, ballistic_range - range_margin) &&
           horizontal_range <= ballistic_range + range_margin;
}

bool compute_release_solution(double* out_error_meters, double* out_time_to_go_seconds, double* out_range_meters,
                              bool* out_in_zone)
{
    if (out_error_meters == nullptr || out_time_to_go_seconds == nullptr || out_range_meters == nullptr ||
        out_in_zone == nullptr)
    {
        return false;
    }

    g_release_alignment_valid = false;
    g_release_min_range = g_release_max_range = 0.0;
    double target_world[3] = {0.0, 0.0, 0.0};
    if (!Cockpit_interop::get_irads_lock_world_point(target_world))
    {
        return false;
    }

    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(target_world[i]))
            return false;
    if (!std::isfinite(vx_world) || !std::isfinite(vy_world) || !std::isfinite(vz_world) ||
        !std::isfinite(position_world_cs.x) || !std::isfinite(position_world_cs.y) ||
        !std::isfinite(position_world_cs.z))
        return false;
    const Ccrp_store_model& profile = get_ccrp_profile();
    const Vec3 release_vel{vx_world, vy_world, vz_world};
    const double dx = target_world[0] - position_world_cs.x;
    const double dz = target_world[2] - position_world_cs.z;
    const double horiz_speed = sqrt((release_vel.x * release_vel.x) + (release_vel.z * release_vel.z));
    const double horiz_range = sqrt((dx * dx) + (dz * dz));
    const double drop_meters = position_world_cs.y - target_world[1];

    if (horiz_speed < std::max(release_cue_min_speed_mps, profile.min_speed_mps) ||
        drop_meters < std::max(release_cue_min_drop_meters, profile.min_drop_meters))
    {
        return false;
    }

    const double vx = release_vel.x / horiz_speed;
    const double vz = release_vel.z / horiz_speed;
    const double along_track = (dx * vx) + (dz * vz);
    if (along_track <= 0.0)
    {
        return false;
    }

    const double cross_track_sq = (horiz_range * horiz_range) - (along_track * along_track);
    const double cross_track = sqrt((cross_track_sq > 0.0) ? cross_track_sq : 0.0);
    const Vec3 world_wind = get_world_wind_vector();
    const double air_x = release_vel.x - world_wind.x, air_y = release_vel.y - world_wind.y,
                 air_z = release_vel.z - world_wind.z;
    const double air_speed = std::sqrt(air_x * air_x + air_y * air_y + air_z * air_z);
    const Store_kind kind = profile.kind;
    double impact_along_track = 0.0;
    double error_meters = 0.0;
    double time_to_go = 0.0;
    if (store_falls_ballistic(kind))
    {
        Vec3 impact_pos{};
        double impact_time = 0.0;
        if (!simulate_bomb_impact_point(profile, position_world_cs, release_vel, world_wind, target_world[1],
                                        &impact_pos, &impact_time))
        {
            return false;
        }
        F117::ccrp_time_of_flight = impact_time;
        const Vec3 impact_delta{impact_pos.x - position_world_cs.x, impact_pos.y - position_world_cs.y,
                                impact_pos.z - position_world_cs.z};
        impact_along_track = (impact_delta.x * vx) + (impact_delta.z * vz);
        error_meters = along_track - impact_along_track;
        time_to_go = (error_meters > 0.0) ? (error_meters / std::max(horiz_speed, profile.min_speed_mps)) : 0.0;
    }
    bool in_zone = false;
    if (kind == Store_kind::jdam || kind == Store_kind::glide)
    {
        // Weapon-authored launch tables, interpolated in target-relative
        // height and true airspeed. No extrapolation outside the table domain.
        const bool tabled = kind == Store_kind::jdam
                                ? Jdam_envelope::lookup(profile.envelope, drop_meters, air_speed, &g_release_min_range,
                                                        &g_release_max_range)
                                : Glide_envelope::lookup(profile.envelope, drop_meters, air_speed, &g_release_min_range,
                                                         &g_release_max_range);
        if (!tabled)
            return false;
        g_release_alignment_valid = cross_track <= std::max(50.0, along_track * 0.577350269);
        in_zone = g_release_alignment_valid && horiz_range >= g_release_min_range && horiz_range <= g_release_max_range;
        if (kind == Store_kind::glide)
        {
            // A glide weapon has no ballistic impact: cue the distance still to close
            // to the maximum range, and time the flight at a typical glide speed.
            error_meters = std::max(0.0, horiz_range - g_release_max_range);
            time_to_go = error_meters / std::max(horiz_speed, profile.min_speed_mps);
            F117::ccrp_time_of_flight = horiz_range / glide_weapon_speed_mps;
        }
    }
    else if (kind == Store_kind::unguided)
    {
        // True CCRP: the release fires as the predicted impact crosses the target.
        g_release_alignment_valid = cross_track <= unguided_max_cross_track_meters;
        in_zone = g_release_alignment_valid && fabs(error_meters) <= profile.release_window_meters;
        g_release_min_range = std::max(1.0, impact_along_track - profile.release_window_meters);
        g_release_max_range = impact_along_track + profile.release_window_meters;
    }
    else if (store_is_missile(kind))
    {
        // Powered missile: target inside the seeker cone around the flight path
        // and slant range inside the stock launch limits.
        const double slant = sqrt(horiz_range * horiz_range + drop_meters * drop_meters);
        const double speed =
            sqrt(release_vel.x * release_vel.x + release_vel.y * release_vel.y + release_vel.z * release_vel.z);
        const double cosine =
            (slant > 1.0 && speed > 1.0)
                ? (dx * release_vel.x - drop_meters * release_vel.y + dz * release_vel.z) / (slant * speed)
                : -1.0;
        g_release_alignment_valid = cosine >= cos(profile.seeker_half_angle_rad);
        g_release_min_range = profile.min_range_meters;
        g_release_max_range = profile.max_range_meters;
        in_zone = g_release_alignment_valid && slant >= g_release_min_range && slant <= g_release_max_range;
        error_meters = std::max(0.0, slant - g_release_max_range);
        time_to_go = error_meters / std::max(horiz_speed, profile.min_speed_mps);
        F117::ccrp_time_of_flight = slant / missile_average_speed_mps;
    }
    else
    {
        in_zone = guided_release_in_zone(kind, impact_along_track, along_track, cross_track,
                                         profile.release_window_meters, &g_release_alignment_valid);
        g_release_min_range =
            std::max(1.0, impact_along_track - std::max(profile.release_window_meters, impact_along_track * 0.20));
        g_release_max_range = impact_along_track + std::max(profile.release_window_meters, impact_along_track * 0.20);
    }
    static bool logged = false;
    static bool last_zone = false;
    static bool last_aligned = false;
    static unsigned diagnostic_frames = 0;
    if (!logged || in_zone != last_zone || g_release_alignment_valid != last_aligned ||
        (F117::weapon_release_pickle_held && ++diagnostic_frames >= 600))
    {
        Cockpit_interop::log_format(
            "DELIVERY ZONE: profile=%d along=%.1f cross=%.1f ballistic=%.1f error=%.1f aligned=%d zone=%d "
            "min=%.1f max=%.1f drop=%.1f speed=%.1f",
            get_ccrp_profile_index(), along_track, cross_track, impact_along_track, error_meters,
            g_release_alignment_valid, in_zone, g_release_min_range, g_release_max_range, drop_meters, horiz_speed);
        logged = true;
        last_zone = in_zone;
        last_aligned = g_release_alignment_valid;
        diagnostic_frames = 0;
    }

    *out_error_meters = error_meters;
    *out_time_to_go_seconds = time_to_go;
    *out_range_meters = along_track;
    *out_in_zone = in_zone;
    return true;
}

void publish_release_cue_params()
{
    initialize_release_cue_params();
    if (!g_release_cue_params_initialized)
    {
        return;
    }

    double error_meters = 0.0;
    double time_to_go_seconds = 0.0;
    double range_meters = 0.0;
    bool in_zone = false;
    const bool valid = Cockpit_interop::has_irads_lock() &&
                       compute_release_solution(&error_meters, &time_to_go_seconds, &range_meters, &in_zone);

    if (valid)
    {
        F117::cockpit_api.setParamNumber(g_release_cue_valid_param, 1.0);
        F117::cockpit_api.setParamNumber(g_release_cue_error_param, error_meters);
        F117::cockpit_api.setParamNumber(g_release_cue_time_to_go_param, time_to_go_seconds);
        F117::cockpit_api.setParamNumber(g_release_cue_range_param, range_meters);
        F117::cockpit_api.setParamNumber(g_release_cue_in_zone_param, in_zone ? 1.0 : 0.0);
    }
    else
    {
        F117::cockpit_api.setParamNumber(g_release_cue_valid_param, 0.0);
        F117::cockpit_api.setParamNumber(g_release_cue_error_param, 0.0);
        F117::cockpit_api.setParamNumber(g_release_cue_time_to_go_param, 0.0);
        F117::cockpit_api.setParamNumber(g_release_cue_range_param, 0.0);
        F117::cockpit_api.setParamNumber(g_release_cue_in_zone_param, 0.0);
    }
}

void publish_irads_lock_params()
{
    initialize_irads_lock_params();
    if (!g_irads_lock_params_initialized)
    {
        return;
    }

    // Parameter payloads are published before their sequence/ack markers.
    // A pending ray is never a valid designation or release target.
    static void* request[7] = {};
    static void* result[5] = {};
    static const char* request_names[] = {"F117_TERRAIN_OX",     "F117_TERRAIN_OY", "F117_TERRAIN_OZ",
                                          "F117_TERRAIN_DX",     "F117_TERRAIN_DY", "F117_TERRAIN_DZ",
                                          "F117_TERRAIN_REQUEST"};
    static const char* result_names[] = {"F117_TERRAIN_X", "F117_TERRAIN_Y", "F117_TERRAIN_Z", "F117_TERRAIN_HIT",
                                         "F117_TERRAIN_ACK"};
    bool handles = true;
    for (int i = 0; i < 7; ++i)
    {
        if (!request[i])
            request[i] = F117::cockpit_api.getParamHandle(request_names[i]);
        handles = handles && request[i];
    }
    for (int i = 0; i < 5; ++i)
    {
        if (!result[i])
            result[i] = F117::cockpit_api.getParamHandle(result_names[i]);
        handles = handles && result[i];
    }
    if (handles)
    {
        unsigned sequence = 0;
        double ray[6] = {};
        bool pending = Cockpit_interop::get_irads_terrain_request(&sequence, ray);
        if (pending && F117::cockpit_api.getParamNumber(result[4]) == sequence)
        {
            double point[3];
            for (int i = 0; i < 3; ++i)
                point[i] = F117::cockpit_api.getParamNumber(result[i]);
            Cockpit_interop::complete_irads_terrain_request(sequence,
                                                            F117::cockpit_api.getParamNumber(result[3]) == 1.0, point);
            pending = false;
        }
        if (pending)
            for (int i = 0; i < 6; ++i)
                F117::cockpit_api.setParamNumber(request[i], ray[i]);
        F117::cockpit_api.setParamNumber(request[6], pending ? sequence : 0.0);
    }

    double world[3] = {0.0, 0.0, 0.0};
    double polar[3] = {0.0, 0.0, 0.0};
    const bool valid = Cockpit_interop::has_irads_lock() && Cockpit_interop::get_irads_lock_world_point(world) &&
                       Cockpit_interop::get_irads_lock_polar(polar);

    if (valid)
    {
        F117::cockpit_api.setParamNumber(g_irads_lock_valid_param, 1.0);
        F117::cockpit_api.setParamNumber(g_irads_lock_world_x_param, world[0]);
        F117::cockpit_api.setParamNumber(g_irads_lock_world_y_param, world[1]);
        F117::cockpit_api.setParamNumber(g_irads_lock_world_z_param, world[2]);
        F117::cockpit_api.setParamNumber(g_irads_lock_range_param, polar[2]);
        F117::cockpit_api.setParamNumber(g_irads_lock_azimuth_param, polar[0]);
        F117::cockpit_api.setParamNumber(g_irads_lock_elevation_param, polar[1]);
    }
    else
    {
        F117::cockpit_api.setParamNumber(g_irads_lock_valid_param, 0.0);
    }
}
}  // namespace Weapons
