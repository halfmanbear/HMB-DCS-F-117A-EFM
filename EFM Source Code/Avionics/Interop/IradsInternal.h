#pragma once
// Shared declarations for the IRADS interop units (Loader, SensorInjection, SensorFrame,
// InputHandler, LockAndTerrain, CameraBridge, Log). Not part of the public interop interface.
#include "../CockpitBase_Interop.h"
#include "../CockpitBase_Layouts.h"

namespace CockpitInterop
{
inline constexpr const char* kIRADSBuildMarker = "2026-11-02-thrust-table-v25";

// Seconds since the IRADS camera last rendered; its ray is stale past this age.
inline constexpr double kIRADSRayMaxAge = 0.5;

inline constexpr double kRadToDeg = 57.295779513;

// Byte offset of the sensor's caged flag, cleared every frame so the gimbal can slew.
inline constexpr size_t kSensorCagedByteOffset = 938;

// -- Native entry points resolved from CockpitBase.dll by Initialize() ------
using PFN_avTVSensor_ctor = void*(__cdecl*)(void* self);
using PFN_avTVSensor_initialize = void(__fastcall*)(void* self);
using PFN_avTVSensor_initZoom = void(__fastcall*)(void* self);
using PFN_avTVSensor_update = void(__fastcall*)(void* self, double dt);
using PFN_avTVSensor_update_frame = void(__fastcall*)(void* self);
using PFN_avTVSensor_slew_axis = void(__fastcall*)(void* self, double value);
using PFN_avTVSensor_abs_axis = bool(__fastcall*)(void* self, float value, float deadzone, double scale);
using PFN_avTVSensor_set_external_designation = void(__fastcall*)(void* self, char enabled, char from_external);
using PFN_avTVSensor_stabilize_on_ground = void(__fastcall*)(void* self, char intersect_terrain);
using PFN_avTVSensor_void_method = void(__fastcall*)(void* self);
using PFN_binding_context = void* (*)();

extern PFN_avTVSensor_ctor g_avTVSensor_ctor;
extern PFN_avTVSensor_initialize g_avTVSensor_initialize;
extern PFN_avTVSensor_initZoom g_avTVSensor_initZoom;
extern PFN_avTVSensor_update g_avTVSensor_update;
extern PFN_avTVSensor_update_frame g_avTVSensor_update_frame;
extern PFN_avTVSensor_slew_axis g_avTVSensor_slew_left;
extern PFN_avTVSensor_slew_axis g_avTVSensor_slew_right;
extern PFN_avTVSensor_slew_axis g_avTVSensor_slew_up;
extern PFN_avTVSensor_slew_axis g_avTVSensor_slew_down;
extern PFN_avTVSensor_abs_axis g_avTVSensor_move_horizontal_abs;
extern PFN_avTVSensor_abs_axis g_avTVSensor_move_vertical_abs;
extern PFN_avTVSensor_set_external_designation g_avTVSensor_set_external_designation;
extern PFN_avTVSensor_stabilize_on_ground g_avTVSensor_stabilize_on_ground;
extern PFN_avTVSensor_void_method g_avTVSensor_search;
extern PFN_avTVSensor_void_method g_avTVSensor_uncage;
extern PFN_avTVSensor_void_method g_avTVSensor_slew_stop;
extern PFN_avTVSensor_void_method g_avTVSensor_zoom_in;
extern PFN_avTVSensor_void_method g_avTVSensor_zoom_out;
extern PFN_binding_context g_binding_context;
extern void* g_avTVSensor_instance;  // our standalone sensor; null until injected

inline AvTVSensorLayout* AsSensor(void* sensor)
{
    return static_cast<AvTVSensorLayout*>(sensor);
}
inline CcCockpitContextLayout* AsContext(void* context)
{
    return static_cast<CcCockpitContextLayout*>(context);
}
inline CcIndicatorLayout* AsIndicator(void* indicator)
{
    return static_cast<CcIndicatorLayout*>(indicator);
}
inline AvPlatformLayout* AsPlatform(void* platform)
{
    return static_cast<AvPlatformLayout*>(platform);
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
inline double s_irads_lock_world[3] = {};
inline double s_last_aircraft_pos[3] = {};

inline void* s_our_ind = nullptr;  // cached DLIR indicator (owned by DCS)

// -- Cross-unit functions --------------------------------------------------
void Log(const char* fmt, ...);
FARPROC ResolveCockpitBaseExport(const char* name);
CcCockpitContextLayout* GetGlobalContext();
CcCockpitContextLayout* GetBindingContext();
bool GroundStabilizeIRADS();
void ClearIRADSLock();
bool AimPoseAtWorldPoint(double pose[16], const double target[3]);
void ResetCameraBridge();

// initialize() leaves the sensor caged and the slew functions do nothing while it is caged,
// so the cage flag is cleared at start-up and again every frame.
inline void uncage_sensor()
{
    *(reinterpret_cast<uint8_t*>(g_avTVSensor_instance) + kSensorCagedByteOffset) = 0;
    if (g_avTVSensor_uncage)
        g_avTVSensor_uncage(g_avTVSensor_instance);
}
}  // namespace CockpitInterop
