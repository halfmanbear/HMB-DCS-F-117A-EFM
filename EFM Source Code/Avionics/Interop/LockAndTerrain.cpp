// IRADS ground designation: the rendered ray goes out as a terrain request (answered by the Lua
// terrain trace) and the reply becomes the world-space lock the weapon code reads.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>

namespace CockpitInterop
{
// Terrain lock acceptance limits.
static constexpr double kMinLockRange_M = 1.0;
static constexpr double kMaxLockRange_M = 80000.0;
static constexpr double kMinRayAlignment = 0.99999999;  // cosine between request ray and lock
static constexpr unsigned kTerrainSequenceWrap = 10000000;
static constexpr double kMinRayNormSq = 0.5;

bool GroundStabilizeIRADS()
{
    if (!g_avTVSensor_instance || !s_irads_power || !s_irads_ray_valid)
        return false;
    double norm = 0;
    for (int i = 0; i < 6; ++i)
        if (!std::isfinite(s_irads_render_ray[i]))
            return false;
    for (int i = 3; i < 6; ++i)
        norm += s_irads_render_ray[i] * s_irads_render_ray[i];
    if (!std::isfinite(norm) || norm < kMinRayNormSq)
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
    if (++s_irads_terrain_sequence > kTerrainSequenceWrap)
        s_irads_terrain_sequence = 1;
    Log("IRADS terrain request: id=%u eye=(%.3f,%.3f,%.3f) direction=(%.8f,%.8f,%.8f)", s_irads_terrain_sequence,
        s_irads_request_ray[0], s_irads_request_ray[1], s_irads_request_ray[2], s_irads_request_ray[3],
        s_irads_request_ray[4], s_irads_request_ray[5]);
    return true;
}

void ClearIRADSLock()
{
    s_irads_lock_valid = false;
    s_irads_terrain_pending = false;
    s_irads_lock_world[0] = s_irads_lock_world[1] = s_irads_lock_world[2] = 0.0;
}

bool GetIRADSTerrainRequest(unsigned* sequence, double ray[6])
{
    if (!s_irads_terrain_pending || !sequence || !ray)
        return false;
    *sequence = s_irads_terrain_sequence;
    std::memcpy(ray, s_irads_request_ray, sizeof(s_irads_request_ray));
    return true;
}

bool CompleteIRADSTerrainRequest(unsigned sequence, bool hit, const double point[3])
{
    if (!s_irads_terrain_pending || sequence != s_irads_terrain_sequence)
        return false;
    s_irads_terrain_pending = false;
    if (!hit || !point || !s_irads_power || !g_avTVSensor_instance)
        return false;
    double distanceSq = 0;
    double dot = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(point[i]))
            return false;
        double delta = point[i] - s_irads_request_ray[i];
        distanceSq += delta * delta;
        dot += delta * s_irads_request_ray[i + 3];
    }
    const double range = std::sqrt(distanceSq);
    if (!std::isfinite(range) || range < kMinLockRange_M || range >= kMaxLockRange_M || dot / range < kMinRayAlignment)
        return false;
    for (int i = 0; i < 3; ++i)
    {
        s_irads_lock_world[i] = point[i];
        AsSensor(g_avTVSensor_instance)->cachedTrackedPoint[i] = point[i];
    }
    s_irads_lock_valid = true;
    Log("IRADS terrain lock accepted: id=%u world=(%.3f,%.3f,%.3f) range=%.2f", sequence, point[0], point[1], point[2],
        range);
    return true;
}

bool HasIRADSLock()
{
    return s_irads_lock_valid;
}

bool GetIRADSLockWorldPoint(double out_point[3])
{
    if (!s_irads_lock_valid || !out_point)
        return false;
    out_point[0] = s_irads_lock_world[0];
    out_point[1] = s_irads_lock_world[1];
    out_point[2] = s_irads_lock_world[2];
    return true;
}

bool GetIRADSLineOfSight(double out_direction[3])
{
    if (!out_direction || !s_irads_power || !s_irads_ray_valid || s_irads_ray_age > kIRADSRayMaxAge)
        return false;
    double norm = 0;
    for (int i = 0; i < 3; ++i)
        norm += s_irads_render_ray[i + 3] * s_irads_render_ray[i + 3];
    if (!std::isfinite(norm) || norm < kMinRayNormSq)
        return false;
    norm = std::sqrt(norm);
    for (int i = 0; i < 3; ++i)
        out_direction[i] = s_irads_render_ray[i + 3] / norm;
    return true;
}

bool GetIRADSLockPolar(double out_polar[3])
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
}  // namespace CockpitInterop
