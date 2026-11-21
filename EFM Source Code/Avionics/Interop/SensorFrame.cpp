// Per-frame update of the injected sensor: world pose from the aircraft, native tick, locked aim.
#include "stdafx.h"
#include "IradsInternal.h"
#include "../../Cockpit/ParamBridge.h"
#include <cmath>
#include <cstring>

namespace Cockpit_interop
{
// Projectors have a native collection power state, independent of EDM draw arguments.
// Drive it from the Lua switch/gear interlock instead of the FC3 near/far/off cycle.
static void update_gear_light_power()
{
    static void* ready_handle = nullptr;
    double ready = 0;
    if (!read_named_param(ready_handle, "F117_LIGHT_CONTROL_READY", ready) || ready < 0.5)
        return;
    using Get_lights = void*(__cdecl*)();
    using Get_child = void*(__cdecl*)(void*, unsigned);
    using Set_power = void(__cdecl*)(void*, double);
    using Get_power = double(__cdecl*)(void*);
    static auto get_lights = reinterpret_cast<Get_lights>(
        resolve_cockpit_base_export("ed_cockpit_external_lights_get_reference"));
    static auto get_child = reinterpret_cast<Get_child>(
        resolve_cockpit_base_export("ed_cockpit_external_lights_get_child"));
    static auto set_power = reinterpret_cast<Set_power>(
        resolve_cockpit_base_export("ed_cockpit_set_external_lights_power"));
    static auto get_power = reinterpret_cast<Get_power>(
        resolve_cockpit_base_export("ed_cockpit_get_external_lights_power"));
    if (!get_lights || !get_child || !set_power || !get_power)
        return;
    void* root = get_lights();
    if (!root)
        return;
    // Native child indices are zero-based; Lua WOLALIGHT indices are one-based.
    static void* handles[2] = {};
    static void* feedback[2] = {};
    static double last_request[2] = {-1, -1};
    const char* names[2] = {"F117_LIGHT_LANDING", "F117_LIGHT_TAXI"};
    const char* power_names[2] = {"F117_LIGHT_LANDING_POWER", "F117_LIGHT_TAXI_POWER"};
    const unsigned indices[2] = {1, 5};
    for (unsigned i = 0; i < 2; ++i)
    {
        double requested = 0;
        if (!read_named_param(handles[i], names[i], requested))
            continue;
        void* light = get_child(root, indices[i]);
        if (!light)
            continue;
        set_power(light, requested > 0.5 ? 1.0 : 0.0);
        const double actual = get_power(light);
        publish_named_param(feedback[i], power_names[i], actual);
        if (last_request[i] != requested)
        {
            log_format("Gear projector: %s child=%u requested=%.0f nativePower=%.2f",
                       names[i], indices[i], requested, actual);
            last_request[i] = requested;
        }
    }
}

// The authored 903 key rotates 129.44 degrees about the housing's lateral axis.
// Lens normals put argument 0 at about +44.2 degrees and argument 1 at -85.24.
static double irads_housing_target(const double los[16], const double mount[16])
{
    double forward = 0, up = 0, right = 0;
    for (int i = 0; i < 3; ++i)
    {
        forward += los[i] * mount[i];
        up += los[i] * mount[4 + i];
        right += los[i] * mount[8 + i];
    }
    if (!std::isfinite(forward) || !std::isfinite(up) || !std::isfinite(right))
        return 0;
    const double elevation = std::atan2(up, std::sqrt(forward * forward + right * right)) * rad_to_deg;
    const double argument = (44.2 - elevation) / 129.44;
    return argument < 0 ? 0 : (argument > 1 ? 1 : argument);
}

static void publish_irads_housing(bool powered)
{
    static void* handle = nullptr;
    const double argument = powered ? irads_housing_target(sensor_world_pose(), sensor_mount_pose()) : 0;
    publish_named_param(handle, "F117_IRADS_HOUSING_TARGET", argument);
}

static bool s_pose_seed_logged = false;
static bool s_carrier_link_disabled_logged = false;

static void build_world_pose_from_aircraft(double out_mat[16], const Aircraft_pose& pose)
{
    const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
    const double qx = (norm > 1.0e-9) ? (pose.qx / norm) : 0.0;
    const double qy = (norm > 1.0e-9) ? (pose.qy / norm) : 0.0;
    const double qz = (norm > 1.0e-9) ? (pose.qz / norm) : 0.0;
    const double qw = (norm > 1.0e-9) ? (pose.qw / norm) : 1.0;
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

    out_mat[12] = pose.x + irads_pos_x * out_mat[0] + irads_pos_y * out_mat[4] + irads_pos_z * out_mat[8];
    out_mat[13] = pose.y + irads_pos_x * out_mat[1] + irads_pos_y * out_mat[5] + irads_pos_z * out_mat[9];
    out_mat[14] = pose.z + irads_pos_x * out_mat[2] + irads_pos_y * out_mat[6] + irads_pos_z * out_mat[10];
    out_mat[15] = 1.0;
}

static void seed_injected_sensor_pose(const Aircraft_pose& pose)
{
    if (!g_av_tv_sensor_instance)
        return;

    double world_pose[16] = {};

    build_world_pose_from_aircraft(world_pose, pose);

    // The accessors resolve the root platform, which is the sensor itself.
    std::memcpy(sensor_world_pose(), world_pose, sizeof(world_pose));
    std::memcpy(sensor_mount_pose(), world_pose, sizeof(world_pose));

    s_last_aircraft_pos[0] = pose.x;
    s_last_aircraft_pos[1] = pose.y;
    s_last_aircraft_pos[2] = pose.z;

    if (!s_pose_seed_logged)
    {
        log_format("SeedInjectedSensorPose: aircraft=(%.2f, %.2f, %.2f) sensor=(%.2f, %.2f, %.2f)", pose.x, pose.y,
                   pose.z, world_pose[12], world_pose[13], world_pose[14]);
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

void update_irads_sensor_frame(double dt, const Aircraft_pose& pose)
{
    update_gear_light_power();
    if (std::isfinite(dt) && dt > 0)
        s_irads_ray_age += dt;
    if (!g_av_tv_sensor_instance || !g_av_tv_sensor_update)
    {
        publish_irads_housing(false);
        return;
    }
    if (!is_irads_powered())
    {
        publish_irads_housing(false);
        s_irads_ray_valid = false;
        s_irads_sensor_ray_valid = false;
        s_irads_homing = false;
        s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
        s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
        s_irads_horizontal_axis = s_irads_vertical_axis = 0.0f;
        clear_irads_lock();
        if (g_av_tv_sensor_slew_stop)
            g_av_tv_sensor_slew_stop(g_av_tv_sensor_instance);
        return;
    }
    if (!s_sensor_initialized)
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
    seed_injected_sensor_pose(pose);
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
    // The initial pose is the aircraft-fixed mount, re-seeded every frame above.
    if (s_irads_homing)
        service_irads_home(sensor_mount_pose());
    // Apply after all native motor/frame work. Do not change the initial mount,
    // target coordinate, optical zoom or free-slew state.
    if (s_irads_lock_valid)
        aim_pose_at_world_point(sensor_world_pose(), s_irads_lock_world);
    publish_irads_housing(true);
    // Sample the native optical pose every simulation frame, even when the cockpit is not
    // rendered. The raw platform pose omits camera/motor extrapolation and gives a different
    // designation in external-view track replay. Use the same calculation as fill_camera.
    using Get_camera_position = double*(__fastcall*)(const void*, double*);
    static auto get_camera_position = reinterpret_cast<Get_camera_position>(
        unique_export(GetModuleHandleA("CockpitBase.dll"), "?get_camera_position@avTVSensor@cockpit@@"));
    double optical_pose[16] = {};
    s_irads_sensor_ray_valid = get_camera_position != nullptr;
    if (get_camera_position)
        get_camera_position(g_av_tv_sensor_instance, optical_pose);
    for (int i = 0; i < 3; ++i)
    {
        s_irads_sensor_ray[i] = optical_pose[12 + i];
        s_irads_sensor_ray[i + 3] = optical_pose[i];
        if (!std::isfinite(optical_pose[12 + i]) || !std::isfinite(optical_pose[i]))
            s_irads_sensor_ray_valid = false;
    }

    static double diagnostic_time = 0;
    static unsigned int diagnostic_count = 0;
    diagnostic_time += dt;
    if (diagnostic_time >= 2.0 && diagnostic_count < 60)
    {
        diagnostic_time = 0;
        ++diagnostic_count;
        const double* los = sensor_world_pose();
        log_format("IRADS camera state: bridged=%d zoom=%.3f targetZoom=%.3f LOS=(%.4f,%.4f,%.4f) axes=(%.3f,%.3f)",
                   s_our_ind != nullptr, g_av_tv_sensor_zoom(g_av_tv_sensor_instance), optical_zoom_target(), los[0],
                   los[1], los[2],
                   s_irads_horizontal_axis,
                   s_irads_vertical_axis);
    }
}
}  // namespace Cockpit_interop
