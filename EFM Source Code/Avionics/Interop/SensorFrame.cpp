// Per-frame update of the injected sensor: world pose from the aircraft, native tick, locked aim.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>

namespace Cockpit_interop
{
static bool s_pose_seed_logged = false;
static bool s_carrier_link_disabled_logged = false;

static void build_world_pose_from_aircraft(double out_mat[16], double aircraft_x, double aircraft_y, double aircraft_z,
                                           double quat_x, double quat_y, double quat_z, double quat_w)
{
    const double norm = std::sqrt(quat_x * quat_x + quat_y * quat_y + quat_z * quat_z + quat_w * quat_w);
    const double qx = (norm > 1.0e-9) ? (quat_x / norm) : 0.0;
    const double qy = (norm > 1.0e-9) ? (quat_y / norm) : 0.0;
    const double qz = (norm > 1.0e-9) ? (quat_z / norm) : 0.0;
    const double qw = (norm > 1.0e-9) ? (quat_w / norm) : 1.0;
    const double xx = qx * qx;
    const double yy = qy * qy;
    const double zz = qz * qz;
    const double xy = qx * qy;
    const double xz = qx * qz;
    const double yz = qy * qz;
    const double wx = qw * qx;
    const double wy = qw * qy;
    const double wz = qw * qz;

    out_mat[0] = 1.0 - 2.0 * (yy + zz);
    out_mat[1] = 2.0 * (xy + wz);
    out_mat[2] = 2.0 * (xz - wy);
    out_mat[3] = 0.0;

    out_mat[4] = 2.0 * (xy - wz);
    out_mat[5] = 1.0 - 2.0 * (xx + zz);
    out_mat[6] = 2.0 * (yz + wx);
    out_mat[7] = 0.0;

    out_mat[8] = 2.0 * (xz + wy);
    out_mat[9] = 2.0 * (yz - wx);
    out_mat[10] = 1.0 - 2.0 * (xx + yy);
    out_mat[11] = 0.0;

    out_mat[12] = aircraft_x + irads_pos_x * out_mat[0] + irads_pos_y * out_mat[4] + irads_pos_z * out_mat[8];
    out_mat[13] = aircraft_y + irads_pos_x * out_mat[1] + irads_pos_y * out_mat[5] + irads_pos_z * out_mat[9];
    out_mat[14] = aircraft_z + irads_pos_x * out_mat[2] + irads_pos_y * out_mat[6] + irads_pos_z * out_mat[10];
    out_mat[15] = 1.0;
}

static void seed_injected_sensor_pose(double aircraft_x, double aircraft_y, double aircraft_z, double quat_x,
                                      double quat_y, double quat_z, double quat_w)
{
    if (!g_av_tv_sensor_instance)
        return;

    Av_tv_sensor_layout* sensor = as_sensor(g_av_tv_sensor_instance);
    Av_platform_layout* root = as_platform(sensor->platform_root ? sensor->platform_root : g_av_tv_sensor_instance);
    double world_pose[16] = {};

    build_world_pose_from_aircraft(world_pose, aircraft_x, aircraft_y, aircraft_z, quat_x, quat_y, quat_z, quat_w);

    std::memcpy(sensor->platform_world_pose.m, world_pose, sizeof(world_pose));
    std::memcpy(sensor->platform_initial_world_pose.m, world_pose, sizeof(world_pose));
    if (root)
    {
        std::memcpy(root->platform_world_pose.m, world_pose, sizeof(world_pose));
        std::memcpy(root->platform_initial_world_pose.m, world_pose, sizeof(world_pose));
    }

    s_last_aircraft_pos[0] = aircraft_x;
    s_last_aircraft_pos[1] = aircraft_y;
    s_last_aircraft_pos[2] = aircraft_z;

    if (!s_pose_seed_logged)
    {
        log_format("SeedInjectedSensorPose: aircraft=(%.2f, %.2f, %.2f) sensor=(%.2f, %.2f, %.2f)", aircraft_x,
                   aircraft_y, aircraft_z, world_pose[12], world_pose[13], world_pose[14]);
        s_pose_seed_logged = true;
    }
}

// DCS pose axes are forward X, up Y, right Z. Preserve the native camera's
// roll while making its final rendered LOS pass through the saved world point.
// Native slave_to_point is rate limited and otherwise leaves a visible lag.
bool aim_pose_at_world_point(double pose[16], const double target[3])
{
    double forward[3];
    double up[3];
    double right[3];
    double length_sq = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(target[i]) || !std::isfinite(pose[12 + i]) || !std::isfinite(pose[4 + i]))
            return false;
        forward[i] = target[i] - pose[12 + i];
        length_sq += forward[i] * forward[i];
    }
    if (!std::isfinite(length_sq) || length_sq < 1.0)
        return false;
    const double inv_length = 1.0 / std::sqrt(length_sq);
    for (int i = 0; i < 3; ++i)
        forward[i] *= inv_length;
    double dot = 0.0;
    for (int i = 0; i < 3; ++i)
        dot += pose[4 + i] * forward[i];
    length_sq = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        up[i] = pose[4 + i] - dot * forward[i];
        length_sq += up[i] * up[i];
    }
    if (length_sq < 1.0e-12)
    {
        // Near vertical LOS: choose a nonparallel reference without a singularity.
        const int axis = std::fabs(forward[1]) < 0.9 ? 1 : 2;
        length_sq = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            up[i] = (i == axis ? 1.0 : 0.0) - forward[axis] * forward[i];
            length_sq += up[i] * up[i];
        }
    }
    const double inv_up = 1.0 / std::sqrt(length_sq);
    for (int i = 0; i < 3; ++i)
        up[i] *= inv_up;
    right[0] = forward[1] * up[2] - forward[2] * up[1];
    right[1] = forward[2] * up[0] - forward[0] * up[2];
    right[2] = forward[0] * up[1] - forward[1] * up[0];
    for (int i = 0; i < 3; ++i)
    {
        pose[i] = forward[i];
        pose[4 + i] = up[i];
        pose[8 + i] = right[i];
    }
    return true;
}

void update_irads_sensor_frame(double dt, double aircraft_x, double aircraft_y, double aircraft_z, double quat_x,
                               double quat_y, double quat_z, double quat_w)
{
    if (std::isfinite(dt) && dt > 0)
        s_irads_ray_age += dt;
    if (!g_av_tv_sensor_instance || !g_av_tv_sensor_update)
        return;
    if (as_sensor(g_av_tv_sensor_instance)->active_flag == 0 || !s_irads_power)
        return;

    if (s_irads_terrain_pending)
    {
        s_irads_terrain_wait += dt;
        if (s_irads_terrain_wait > 1.0)
        {
            s_irads_terrain_pending = false;
            log_message("IRADS terrain request timed out; no lock");
        }
    }
    seed_injected_sensor_pose(aircraft_x, aircraft_y, aircraft_z, quat_x, quat_y, quat_z, quat_w);
    // Do not call set_carrier for the injected sensor: it leads to an unsafe
    // carrier chain during the sensor update and crashed DCS. Keeping
    // platformRoot self-referential preserves the native update path without
    // entering that chain.
    if (!s_carrier_link_disabled_logged)
    {
        log_message("UpdateIRADSSensorFrame: native carrier link disabled for injected sensor");
        s_carrier_link_disabled_logged = true;
    }

    // Keep the sensor uncaged every frame: some DCS mode transitions re-cage it, which
    // blocks all slew.
    uncage_sensor();

    // Tick the sensor: the update drives the gimbal motors from slew commands
    // and applies their angles to the world pose. No direct pose write needed.
    if (s_irads_lock_valid)
    {
        using Slave_to_point = void(__fastcall*)(void*, const double*);
        static auto slave = reinterpret_cast<Slave_to_point>(
            resolve_cockpit_base_export("?slave_to_point@avTVSensor@cockpit@@UEAAXAEBVVec3d@osg@@@Z"));
        if (slave)
            slave(g_av_tv_sensor_instance, s_irads_lock_world);
    }
    g_av_tv_sensor_update(g_av_tv_sensor_instance, dt);
    if (g_av_tv_sensor_update_frame)
        g_av_tv_sensor_update_frame(g_av_tv_sensor_instance);
    // Apply after all native motor/frame work. Do not change the initial mount,
    // target coordinate, optical zoom or free-slew state.
    if (s_irads_lock_valid)
        aim_pose_at_world_point(as_sensor(g_av_tv_sensor_instance)->platform_world_pose.m, s_irads_lock_world);

    static double diagnostic_time = 0;
    static unsigned int diagnostic_count = 0;
    diagnostic_time += dt;
    if (diagnostic_time >= 2.0 && diagnostic_count < 60)
    {
        diagnostic_time = 0;
        ++diagnostic_count;
        const auto* sensor = as_sensor(g_av_tv_sensor_instance);
        log_format("IRADS camera state: bridged=%d zoom=%.3f targetZoom=%.3f LOS=(%.4f,%.4f,%.4f) axes=(%.3f,%.3f)",
                   s_our_ind != nullptr, sensor->current_zoom, sensor->target_zoom, sensor->platform_world_pose.m[0],
                   sensor->platform_world_pose.m[1], sensor->platform_world_pose.m[2], s_irads_horizontal_axis,
                   s_irads_vertical_axis);
    }
}
}  // namespace Cockpit_interop
