#pragma once
// ============================================================================
// CockpitBase_Interop.h - avTVSensor injection into the DCS cockpit context
// Public interface used by the flight model; the implementation lives in Interop/.
// ============================================================================
#include <Windows.h>
#include <cstdarg>
#include <cstdint>

namespace Cockpit_interop
{
static constexpr int ir_screen_condition_flir = 11;
static constexpr int ir_camera_condition_on = 1;
static constexpr size_t avtvsensor_alloc_size = 0x800;

// F-117A DLIR belly position in model-local coordinates (metres).
// X = forward from CG, Y = right, Z = up.
// Tune these if the camera renders the wrong part of the scene.
static constexpr double irads_pos_x = 4.0;   // 4 m forward (same default as the native sensor)
static constexpr double irads_pos_y = 0.0;   // centreline
static constexpr double irads_pos_z = -0.3;  // 0.3 m below centreline (belly mount)

// Aircraft position and attitude quaternion in world axes, as DCS reports them each frame.
struct Aircraft_pose
{
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

extern HMODULE g_h_dll;             // this module; set by DllMain, used for the log path
extern void** g_contexts_ptr_addr;  // address of the cockpit context table

void log_message(const char* msg);
void log_vformat(const char* fmt, va_list args);
void log_format(const char* fmt, ...);
bool initialize();
bool inject_irads_sensor_early();  // Phase A: DllMain - ctor + add to keeper
bool inject_irads_sensor_late();   // Phase B: ed_fm_start - ensure init + camera pos + IR mode
bool handle_irads_command(int command, float value);
int get_irads_zoom_level();
bool is_irads_powered();
void update_irads_sensor_frame(double dt, const Aircraft_pose& pose);
void force_irads_indicator_active();  // Phase C: called every frame from ed_fm_simulate to keep
                                      // screenCondition/cameraCondition live
bool get_irads_terrain_request(unsigned* sequence, double ray[6]);
bool complete_irads_terrain_request(unsigned sequence, bool hit, const double point[3]);
bool has_irads_lock();
bool get_irads_lock_world_point(double out_point[3]);
// Unit world direction of the rendered IRADS view; false when off or stale.
bool get_irads_line_of_sight(double out_direction[3]);
bool get_irads_lock_polar(double out_polar[3]);
void reset_irads_session();  // ed_fm_release: drop pointers into the torn-down cockpit context

}  // namespace Cockpit_interop
