#pragma once
// ============================================================================
// CockpitBase_Interop.h - avTVSensor injection into the DCS cockpit context
// Public interface used by the flight model; the implementation lives in Interop/.
// ============================================================================
#include <Windows.h>
#include <cstdarg>
#include <cstdint>

namespace CockpitInterop
{
static constexpr int IR_SCREEN_CONDITION_FLIR = 11;
static constexpr int IR_CAMERA_CONDITION_ON = 1;
static constexpr size_t AVTVSENSOR_ALLOC_SIZE = 0x800;

// F-117A DLIR belly position in model-local coordinates (metres).
// X = forward from CG, Y = right, Z = up.
// Tune these if the camera renders the wrong part of the scene.
static constexpr double IRADS_POS_X = 4.0;   // 4 m forward (same default as the native sensor)
static constexpr double IRADS_POS_Y = 0.0;   // centreline
static constexpr double IRADS_POS_Z = -0.3;  // 0.3 m below centreline (belly mount)

extern HMODULE g_hDLL;              // this module; set by DllMain, used for the log path
extern void** g_contexts_ptr_addr;  // address of the cockpit context table

void Log_(const char* msg);
void LogV(const char* fmt, va_list args);
void Log(const char* fmt, ...);
bool Initialize();
bool InjectIRADSSensor_Early();  // Phase A: DllMain - ctor + add to keeper
bool InjectIRADSSensor_Late();   // Phase B: ed_fm_start - ensure init + camera pos + IR mode
bool HandleIRADSCommand(int command, float value);
int GetIRADSZoomLevel();
bool IsIRADSPowered();
void UpdateIRADSSensorFrame(double dt, double aircraft_x, double aircraft_y, double aircraft_z, double quat_x,
                            double quat_y, double quat_z, double quat_w);
void ForceIRADSIndicatorActive();  // Phase C: called every frame from ed_fm_simulate to keep
                                   // screenCondition/cameraCondition live
bool GetIRADSTerrainRequest(unsigned* sequence, double ray[6]);
bool CompleteIRADSTerrainRequest(unsigned sequence, bool hit, const double point[3]);
bool HasIRADSLock();
bool GetIRADSLockWorldPoint(double out_point[3]);
// Unit world direction of the rendered IRADS view; false when off or stale.
bool GetIRADSLineOfSight(double out_direction[3]);
bool GetIRADSLockPolar(double out_polar[3]);
void ResetIRADSSession();  // ed_fm_release: drop pointers into the torn-down cockpit context

}  // namespace CockpitInterop
