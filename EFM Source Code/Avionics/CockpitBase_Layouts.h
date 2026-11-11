#pragma once

#include <cstddef>
#include <cstdint>

namespace Cockpit_interop
{
struct World_pose_matrix
{
    double m[16];
};

struct Sensor_frame_layout
{
    double half_azimuth;
    double half_elevation;
};
static_assert(sizeof(Sensor_frame_layout) == 0x10);

struct Search_item_layout
{
    std::uint32_t object_id;
    std::uint32_t pad_04;
    double relative_azimuth;
    double relative_elevation;
    double relative_range;
    double local_track_offset_x;
    double local_track_offset_y;
    double local_track_offset_z;
    Sensor_frame_layout candidate_frame;
    std::uint8_t has_los_intersection;
    std::uint8_t precise_track_candidate;
    std::uint8_t pad_4_a[6];
};
static_assert(sizeof(Search_item_layout) == 0x50);

struct Av_track_data_layout
{
    std::uint32_t object_id;
    std::uint32_t pad_04;
    double relative_azimuth;
    double relative_elevation;
    double relative_range;
    double world_transform[16];
    double velocity[3];
    std::uint8_t pad_0_b8[0xD0 - 0xB8];
    double last_update_model_time;
    std::uint8_t pad_0_d8[0xE8 - 0xD8];
    void* moving_object_or_type;
    double approx_size;
    double local_point_offset[3];
};
static_assert(offsetof(Av_track_data_layout, local_point_offset) == 0xF8);
static_assert(sizeof(Av_track_data_layout) == 0x110);

struct Pointer_vector_view
{
    void** begin;
    void** end;
    void** cap;
};
static_assert(sizeof(Pointer_vector_view) == 0x18);

struct Cc_cockpit_context_layout
{
    std::uint8_t pad_000[0x10];
    void* moving_object_raw;  // the aircraft object
    std::uint8_t pad_018[0x10];
    void* carrier_link_host_raw;  // carrier link host pointer
    std::uint8_t devices_keeper_storage[0x18];
    std::uint8_t pad_048[0xB0 - 0x48];
    Pointer_vector_view indicators;
    std::uint8_t pad_0_c8[0x130 - 0xC8];
    Pointer_vector_view child_contexts;
};
static_assert(offsetof(Cc_cockpit_context_layout, moving_object_raw) == 0x10);
static_assert(offsetof(Cc_cockpit_context_layout, carrier_link_host_raw) == 0x28);
static_assert(offsetof(Cc_cockpit_context_layout, devices_keeper_storage) == 0x30);
static_assert(offsetof(Cc_cockpit_context_layout, indicators) == 0xB0);
static_assert(offsetof(Cc_cockpit_context_layout, child_contexts) == 0x130);

struct Cc_indicator_layout
{
    std::uint8_t pad_000[0x38];
    void* controller;
    std::uint8_t pad_040[0x3A0 - 0x40];
    std::uint32_t screen_condition;
    std::uint8_t pad_3_a4[0x46C - 0x3A4];
    std::uint8_t indicator_type;
    std::uint8_t pad_46_d[0x470 - 0x46D];
    std::uint32_t render_target_id;
    std::uint8_t pad_474[0x48C - 0x474];
    std::uint32_t camera_condition;
};
static_assert(offsetof(Cc_indicator_layout, controller) == 0x38);
static_assert(offsetof(Cc_indicator_layout, screen_condition) == 0x3A0);
static_assert(offsetof(Cc_indicator_layout, indicator_type) == 0x46C);
static_assert(offsetof(Cc_indicator_layout, render_target_id) == 0x470);
static_assert(offsetof(Cc_indicator_layout, camera_condition) == 0x48C);

struct Av_tv_sensor_layout
{
    void* primary_vftable;
    void* platform_root;
    std::uint8_t platform_motors[0xA8 - 0x10];  // avPlatform motors; NEVER avDevice fields
    World_pose_matrix platform_world_pose;
    World_pose_matrix platform_initial_world_pose;
    World_pose_matrix platform_initial_local_pose;
    double platform_last_update_time;  // render extrapolation timestamp
    std::uint8_t pad_230[0x260 - 0x230];
    std::uint8_t active_flag;
    std::uint8_t pad_261[0x268 - 0x261];
    void* search_timer;
    std::uint8_t pad_270[0x328 - 0x270];
    double cached_tracked_point[3];
    std::uint8_t track_mode;
    std::uint8_t pad_341[0x348 - 0x341];
    double base_fov;
    std::uint8_t pad_350[0x358 - 0x350];
    double current_zoom;
    double target_zoom;  // zoom motor target
    std::uint8_t pad_368[0x4C0 - 0x368];
    double aspect_ratio;
    Sensor_frame_layout current_frame;
    std::uint8_t pad_4_d8[0x610 - 0x4D8];
    std::uint8_t hat_lock;
};
static_assert(offsetof(Av_tv_sensor_layout, platform_root) == 0x08);
static_assert(offsetof(Av_tv_sensor_layout, platform_world_pose) == 0xA8);
static_assert(offsetof(Av_tv_sensor_layout, platform_initial_world_pose) == 0x128);
static_assert(offsetof(Av_tv_sensor_layout, platform_initial_local_pose) == 0x1A8);
static_assert(offsetof(Av_tv_sensor_layout, platform_last_update_time) == 0x228);
static_assert(offsetof(Av_tv_sensor_layout, active_flag) == 0x260);
static_assert(offsetof(Av_tv_sensor_layout, search_timer) == 0x268);
static_assert(offsetof(Av_tv_sensor_layout, cached_tracked_point) == 0x328);
static_assert(offsetof(Av_tv_sensor_layout, track_mode) == 0x340);
static_assert(offsetof(Av_tv_sensor_layout, base_fov) == 0x348);
static_assert(offsetof(Av_tv_sensor_layout, current_zoom) == 0x358);
static_assert(offsetof(Av_tv_sensor_layout, target_zoom) == 0x360);
static_assert(offsetof(Av_tv_sensor_layout, aspect_ratio) == 0x4C0);
static_assert(offsetof(Av_tv_sensor_layout, current_frame) == 0x4C8);
static_assert(offsetof(Av_tv_sensor_layout, hat_lock) == 0x610);

struct Av_platform_layout
{
    void* primary_vftable;
    void* self_or_root;
    std::uint8_t pad_010[0xA8 - 0x10];
    World_pose_matrix platform_world_pose;
    World_pose_matrix platform_initial_world_pose;
    World_pose_matrix platform_initial_local_pose;
};
static_assert(offsetof(Av_platform_layout, self_or_root) == 0x08);
static_assert(offsetof(Av_platform_layout, platform_world_pose) == 0xA8);
static_assert(offsetof(Av_platform_layout, platform_initial_world_pose) == 0x128);
static_assert(offsetof(Av_platform_layout, platform_initial_local_pose) == 0x1A8);
}  // namespace Cockpit_interop
