#pragma once
// Shared declarations for the IRADS interop units (Loader, SensorInjection, SensorFrame,
// InputHandler, LockAndTerrain, CameraBridge, Log). Not part of the public interop interface.
#include "../CockpitBase_Interop.h"
#include "../CockpitBase_Layouts.h"

namespace Cockpit_interop
{
inline constexpr const char* irads_build_marker = "2026-11-02-thrust-table-v25";

// Seconds since the IRADS camera last rendered; its ray is stale past this age.
inline constexpr double irads_ray_max_age = 0.5;

inline constexpr double rad_to_deg = 57.295779513;

// -- Native entry points resolved from CockpitBase.dll by Initialize() ------
using Pfn_av_tv_sensor_ctor = void*(__cdecl*)(void* self);
using Pfn_av_tv_sensor_initialize = void(__fastcall*)(void* self);
using Pfn_av_tv_sensor_init_zoom = void(__fastcall*)(void* self);
using Pfn_av_tv_sensor_update = void(__fastcall*)(void* self, double dt);
using Pfn_av_tv_sensor_update_frame = void(__fastcall*)(void* self);
using Pfn_av_tv_sensor_slew_axis = void(__fastcall*)(void* self, double value);
using Pfn_av_tv_sensor_abs_axis = bool(__fastcall*)(void* self, float value, float deadzone, double scale);
using Pfn_av_tv_sensor_set_external_designation = void(__fastcall*)(void* self, char enabled, char from_external);
using Pfn_av_tv_sensor_stabilize_on_ground = void(__fastcall*)(void* self, char intersect_terrain);
using Pfn_av_tv_sensor_void_method = void(__fastcall*)(void* self);
using Pfn_binding_context = void* (*)();
// avPlatform pose accessors. The injected sensor is its own root platform, so each returns
// the sensor's 4x4 pose by reference: elements 0-2 forward axis, 12-14 position.
using Pfn_platform_pose = double*(__fastcall*)(void* self);
// Rebuilds the root pose from the mount and gimbal angles and stamps it with the current
// model time, so camera reads in the same model time use the stored pose.
using Pfn_platform_update = void(__fastcall*)(void* self);
using Pfn_sensor_flag = bool(__fastcall*)(const void* self);
using Pfn_sensor_value = double(__fastcall*)(const void* self);
// Returns the sensor's cached designated point by reference; read by the native tracked-point query.
using Pfn_sensor_point = double*(__fastcall*)(const void* self);
// avMotor accessors, used on the sensor's zoom motor.
using Pfn_motor_value = double(__fastcall*)(const void* motor);
using Pfn_motor_force = void(__fastcall*)(void* motor, double value);
using Pfn_motor_stop = void(__fastcall*)(void* motor);  // target = current position

extern Pfn_av_tv_sensor_ctor g_av_tv_sensor_ctor;
extern Pfn_av_tv_sensor_initialize g_av_tv_sensor_initialize;
extern Pfn_av_tv_sensor_init_zoom g_av_tv_sensor_init_zoom;
extern Pfn_av_tv_sensor_update g_av_tv_sensor_update;
extern Pfn_av_tv_sensor_update_frame g_av_tv_sensor_update_frame;
extern Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_left;
extern Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_right;
extern Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_up;
extern Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_down;
extern Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_horizontal_abs;
extern Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_vertical_abs;
extern Pfn_av_tv_sensor_set_external_designation g_av_tv_sensor_set_external_designation;
extern Pfn_av_tv_sensor_stabilize_on_ground g_av_tv_sensor_stabilize_on_ground;
extern Pfn_av_tv_sensor_void_method g_av_tv_sensor_search;
extern Pfn_av_tv_sensor_void_method g_av_tv_sensor_uncage;
extern Pfn_av_tv_sensor_void_method g_av_tv_sensor_slew_stop;
extern Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_in;
extern Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_out;
extern Pfn_binding_context g_binding_context;
extern Pfn_platform_pose g_platform_position;          // current line-of-sight pose
extern Pfn_platform_pose g_platform_initial_position;  // aircraft-fixed mount pose
extern Pfn_platform_update g_platform_update_position;
extern Pfn_sensor_flag g_av_tv_sensor_caged;
extern Pfn_sensor_value g_av_tv_sensor_zoom;         // current optical zoom factor
extern Pfn_sensor_value g_av_tv_sensor_default_fov;  // full field of view at zoom 1, radians
extern Pfn_sensor_value g_av_tv_sensor_view_angle;   // full field of view at the current zoom, radians
extern Pfn_sensor_point g_av_tv_sensor_reference_point;
extern Pfn_motor_value g_motor_get;
extern Pfn_motor_value g_motor_get_target;
extern Pfn_motor_force g_motor_force;
extern Pfn_motor_stop g_motor_stop;
extern void* g_av_tv_sensor_instance;  // our standalone sensor; null until injected

inline double* sensor_world_pose()
{
    return g_platform_position(g_av_tv_sensor_instance);
}
inline double* sensor_mount_pose()
{
    return g_platform_initial_position(g_av_tv_sensor_instance);
}

// -- Session state ---------------------------------------------------------
// Written by the input handler, the per-frame update and the camera callback; cleared by
// ResetIRADSSession.
inline bool s_added = false;              // sensor constructed and registered
inline bool s_sensor_initialized = false;  // native initialize() has run on our sensor
inline bool s_irads_power = false; // pilot selection; is_irads_powered also checks electrical/BIT readiness
inline int s_irads_zoom_level = 1;

// Slew demand. Keyboard input takes priority over a centred analog axis.
inline float s_irads_horizontal_axis = 0.0f;
inline float s_irads_vertical_axis = 0.0f;
inline float s_irads_analog_horizontal = 0.0f;
inline float s_irads_analog_vertical = 0.0f;
inline float s_irads_keyboard_horizontal = 0.0f;
inline float s_irads_keyboard_vertical = 0.0f;

// Terrain designation: the rendered ray is captured by the camera callback, sent to the
// Lua terrain trace as a request, and answered through CompleteIRADSTerrainRequest.
inline bool s_irads_ray_valid = false;
inline double s_irads_render_ray[6] = {};  // eye xyz, unit direction xyz
inline double s_irads_request_ray[6] = {};
inline double s_irads_ray_age = 1e9;
inline bool s_irads_terrain_pending = false;
inline unsigned s_irads_terrain_sequence = 0;
inline double s_irads_terrain_wait = 0.0;
inline bool s_irads_lock_valid = false;
inline bool s_irads_homing = false;  // Sensor Reset: slew the gimbal back to its mounted forward axis
inline double s_irads_lock_world[3] = {};
inline double s_last_aircraft_pos[3] = {};

inline void* s_our_ind = nullptr;  // cached DLIR indicator (owned by DCS)

// -- Cross-unit functions --------------------------------------------------
FARPROC resolve_cockpit_base_export(const char* name);
// Positions of members DCS exports no accessor for, read from the exported functions that use
// them. Zero if those functions disagree or do not match the expected form.
size_t screen_condition_offset();
size_t zoom_motor_offset();
void* get_global_context();
void* get_binding_context();
// Sets the optical zoom factor, current and target together. False if the zoom motor is unconfirmed.
bool set_optical_zoom(double zoom);
double optical_zoom_target();  // NaN if the zoom motor is unconfirmed
bool ground_stabilize_irads();
void clear_irads_lock();
void service_irads_home(const double mount[16]);  // mount: aircraft-fixed sensor pose, forward/up/right columns
bool aim_pose_at_world_point(double pose[16], const double target[3]);
void reset_camera_bridge();

// initialize() leaves the sensor caged and the slew functions do nothing while it is caged,
// so the cage flag is cleared at start-up and again every frame.
inline void uncage_sensor()
{
    g_av_tv_sensor_uncage(g_av_tv_sensor_instance);
}
}  // namespace Cockpit_interop
