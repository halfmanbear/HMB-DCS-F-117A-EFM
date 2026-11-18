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

// Byte offset of the sensor's caged flag, cleared every frame so the gimbal can slew.
inline constexpr size_t sensor_caged_byte_offset = 938;

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
extern void* g_av_tv_sensor_instance;  // our standalone sensor; null until injected

inline Av_tv_sensor_layout* as_sensor(void* sensor)
{
    return static_cast<Av_tv_sensor_layout*>(sensor);
}
inline Cc_cockpit_context_layout* as_context(void* context)
{
    return static_cast<Cc_cockpit_context_layout*>(context);
}
inline Cc_indicator_layout* as_indicator(void* indicator)
{
    return static_cast<Cc_indicator_layout*>(indicator);
}
inline Av_platform_layout* as_platform(void* platform)
{
    return static_cast<Av_platform_layout*>(platform);
}

// -- Session state ---------------------------------------------------------
// Written by the input handler, the per-frame update and the camera callback; cleared by
// ResetIRADSSession.
inline bool s_added = false;  // sensor constructed and registered
inline bool s_irads_power = true;
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
Cc_cockpit_context_layout* get_global_context();
Cc_cockpit_context_layout* get_binding_context();
bool ground_stabilize_irads();
void clear_irads_lock();
void service_irads_home(const double mount[16]);  // mount: aircraft-fixed sensor pose, forward/up/right columns
bool aim_pose_at_world_point(double pose[16], const double target[3]);
void reset_camera_bridge();

// initialize() leaves the sensor caged and the slew functions do nothing while it is caged,
// so the cage flag is cleared at start-up and again every frame.
inline void uncage_sensor()
{
    *(reinterpret_cast<uint8_t*>(g_av_tv_sensor_instance) + sensor_caged_byte_offset) = 0;
    if (g_av_tv_sensor_uncage)
        g_av_tv_sensor_uncage(g_av_tv_sensor_instance);
}
}  // namespace Cockpit_interop
