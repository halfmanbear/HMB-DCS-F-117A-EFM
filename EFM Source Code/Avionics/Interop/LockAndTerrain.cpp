// IRADS ground designation: the rendered ray goes out as a terrain request (answered by the Lua
// terrain trace) and the reply becomes the world-space lock the weapon code reads.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>

namespace Cockpit_interop
{
// Terrain lock acceptance limits.
static constexpr double min_lock_range_m = 1.0;
static constexpr double max_lock_range_m = 80000.0;
static constexpr double min_ray_alignment = 0.99999999;  // cosine between request ray and lock
static constexpr unsigned terrain_sequence_wrap = 10000000;
static constexpr double min_ray_norm_sq = 0.5;

bool ground_stabilize_irads()
{
    if (!g_av_tv_sensor_instance || !s_irads_power || !s_irads_ray_valid)
        return false;
    double norm = 0;
    for (int i = 0; i < 6; ++i)
        if (!std::isfinite(s_irads_render_ray[i]))
            return false;
    for (int i = 3; i < 6; ++i)
        norm += s_irads_render_ray[i] * s_irads_render_ray[i];
    if (!std::isfinite(norm) || norm < min_ray_norm_sq)
        return false;
    norm = std::sqrt(norm);
    for (int i = 0; i < 3; ++i)
    {
        s_irads_request_ray[i] = s_irads_render_ray[i];
        s_irads_request_ray[i + 3] = s_irads_render_ray[i + 3] / norm;
    }
    s_irads_lock_valid = false;
    s_irads_terrain_pending = true;
    s_irads_terrain_wait = 0;
    if (++s_irads_terrain_sequence > terrain_sequence_wrap)
        s_irads_terrain_sequence = 1;
    log_format("IRADS terrain request: id=%u eye=(%.3f,%.3f,%.3f) direction=(%.8f,%.8f,%.8f)", s_irads_terrain_sequence,
               s_irads_request_ray[0], s_irads_request_ray[1], s_irads_request_ray[2], s_irads_request_ray[3],
               s_irads_request_ray[4], s_irads_request_ray[5]);
    return true;
}

void clear_irads_lock()
{
    s_irads_lock_valid = false;
    s_irads_terrain_pending = false;
    s_irads_lock_world[0] = s_irads_lock_world[1] = s_irads_lock_world[2] = 0.0;
}

bool get_irads_terrain_request(unsigned* sequence, double ray[6])
{
    if (!s_irads_terrain_pending || !sequence || !ray)
        return false;
    *sequence = s_irads_terrain_sequence;
    std::memcpy(ray, s_irads_request_ray, sizeof(s_irads_request_ray));
    return true;
}

bool complete_irads_terrain_request(unsigned sequence, bool hit, const double point[3])
{
    if (!s_irads_terrain_pending || sequence != s_irads_terrain_sequence)
        return false;
    s_irads_terrain_pending = false;
    if (!hit || !point || !s_irads_power || !g_av_tv_sensor_instance)
        return false;
    double distance_sq = 0;
    double dot = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(point[i]))
            return false;
        double delta = point[i] - s_irads_request_ray[i];
        distance_sq += delta * delta;
        dot += delta * s_irads_request_ray[i + 3];
    }
    const double range = std::sqrt(distance_sq);
    if (!std::isfinite(range) || range < min_lock_range_m || range >= max_lock_range_m ||
        dot / range < min_ray_alignment)
        return false;
    for (int i = 0; i < 3; ++i)
    {
        s_irads_lock_world[i] = point[i];
        as_sensor(g_av_tv_sensor_instance)->cached_tracked_point[i] = point[i];
    }
    s_irads_lock_valid = true;
    log_format("IRADS terrain lock accepted: id=%u world=(%.3f,%.3f,%.3f) range=%.2f", sequence, point[0], point[1],
               point[2], range);
    return true;
}

bool has_irads_lock()
{
    return s_irads_lock_valid;
}

bool get_irads_lock_world_point(double out_point[3])
{
    if (!s_irads_lock_valid || !out_point)
        return false;
    out_point[0] = s_irads_lock_world[0];
    out_point[1] = s_irads_lock_world[1];
    out_point[2] = s_irads_lock_world[2];
    return true;
}

bool get_irads_line_of_sight(double out_direction[3])
{
    if (!out_direction || !s_irads_power || !s_irads_ray_valid || s_irads_ray_age > irads_ray_max_age)
        return false;
    double norm = 0;
    for (int i = 0; i < 3; ++i)
        norm += s_irads_render_ray[i + 3] * s_irads_render_ray[i + 3];
    if (!std::isfinite(norm) || norm < min_ray_norm_sq)
        return false;
    norm = std::sqrt(norm);
    for (int i = 0; i < 3; ++i)
        out_direction[i] = s_irads_render_ray[i + 3] / norm;
    return true;
}

bool get_irads_lock_polar(double out_polar[3])
{
    if (!s_irads_lock_valid || !out_polar)
        return false;
    const double dx = s_irads_lock_world[0] - s_last_aircraft_pos[0];
    const double dy = s_irads_lock_world[1] - s_last_aircraft_pos[1];
    const double dz = s_irads_lock_world[2] - s_last_aircraft_pos[2];
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    out_polar[0] = std::atan2(dz, dx);
    out_polar[1] = range > 0 ? std::asin(dy / range) : 0.0;
    out_polar[2] = range;
    return true;
}
}  // namespace Cockpit_interop
