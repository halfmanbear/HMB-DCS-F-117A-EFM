#pragma once
// ============================================================================
// CockpitBase_Interop.h — avTVSensor injection into the DCS cockpit context
// ============================================================================
#include <Windows.h>
#include <cstdint>

namespace CockpitInterop
{
    static constexpr uint8_t IRADS_DEVICE_INDEX        = 9;
    static constexpr size_t  AVTVSENSOR_ALLOC_SIZE   = 0x800;
    static constexpr size_t  AVPLATFORM_ALLOC_SIZE   = 0x400;  // stub root platform (560 b used, 1024 allocated)
    static constexpr int     IR_SCREEN_CONDITION_FLIR  = 11;
    static constexpr int     IR_CAMERA_CONDITION_ON    = 1;

    // F-117A DLIR belly position in model-local coordinates (metres).
    // X = forward from CG, Y = right, Z = up.
    // Tune these if the camera renders the wrong part of the scene.
    static constexpr double IRADS_POS_X =  4.0;   // 4 m forward (matches avTVSensor::initialize() hardcode)
    static constexpr double IRADS_POS_Y =  0.0;   // centreline
    static constexpr double IRADS_POS_Z = -0.3;   // 0.3 m below centreline (belly mount)

    using PFN_avTVSensor_ctor         = void* (__cdecl*)(void* self);
    using PFN_avTVSensor_initialize   = void  (__fastcall*)(void* self);
    using PFN_avTVSensor_initZoom     = void  (__fastcall*)(void* self);
    using PFN_devices_keeper_add      = void  (__cdecl*)(void* keeper, void* device);
    using PFN_devices_keeper_get      = void* (__cdecl*)(void* keeper, uint8_t index);
    using PFN_SetFlirDefault          = void  (__cdecl*)(int condition);
    using PFN_avTVSensor_update      = void  (__fastcall*)(void* self, double dt);
    using PFN_avTVSensor_slew_axis   = void  (__fastcall*)(void* self, double value);
    using PFN_avTVSensor_abs_axis    = bool  (__fastcall*)(void* self, float value, float deadzone, double scale);
    using PFN_avTVSensor_set_external_designation = void (__fastcall*)(void* self, char enabled, char from_external);
    using PFN_avTVSensor_get_tracked_point = void* (__fastcall*)(void* self, double out_point[3]);
    using PFN_avTVSensor_get_polar_position = void* (__fastcall*)(void* self, double out_polar[3]);
    using PFN_avTVSensor_stabilize_on_ground = void (__fastcall*)(void* self, char intersect_terrain);
    using PFN_avTVSensor_update_frame = void  (__fastcall*)(void* self);
    using PFN_avTVSensor_void_method  = void  (__fastcall*)(void* self);
    using PFN_avDevice_start          = void  (__fastcall*)(void* self, unsigned int reason);
    using PFN_avPlatform_ctor         = void* (__fastcall*)(void* self);
    using PFN_avPlatform_set_carrier  = void  (__fastcall*)(void* self, void* movingObject);
    using PFN_avPlatform_update_pos   = void  (__fastcall*)(void* self);
    using PFN_binding_context         = void* (*)();

    extern HMODULE                   g_hDLL;
    extern PFN_avTVSensor_ctor       g_avTVSensor_ctor;
    extern PFN_avTVSensor_initialize g_avTVSensor_initialize;
    extern PFN_avTVSensor_initZoom   g_avTVSensor_initZoom;
    extern PFN_devices_keeper_add    g_devices_keeper_add;
    extern PFN_devices_keeper_get    g_devices_keeper_get;
    extern PFN_SetFlirDefault        g_setFlirDefault;
    extern PFN_avDevice_start        g_avDevice_start;
    extern PFN_avTVSensor_update_frame g_avTVSensor_update_frame;
    extern PFN_avTVSensor_update      g_avTVSensor_update;
    extern PFN_avTVSensor_slew_axis   g_avTVSensor_slew_left;
    extern PFN_avTVSensor_slew_axis   g_avTVSensor_slew_right;
    extern PFN_avTVSensor_slew_axis   g_avTVSensor_slew_up;
    extern PFN_avTVSensor_slew_axis   g_avTVSensor_slew_down;
    extern PFN_avTVSensor_abs_axis    g_avTVSensor_move_horizontal_abs;
    extern PFN_avTVSensor_abs_axis    g_avTVSensor_move_vertical_abs;
    extern PFN_avTVSensor_set_external_designation g_avTVSensor_set_external_designation;
    extern PFN_avTVSensor_get_tracked_point g_avTVSensor_get_tracked_point;
    extern PFN_avTVSensor_get_polar_position g_avTVSensor_get_polar_position;
    extern PFN_avTVSensor_stabilize_on_ground g_avTVSensor_stabilize_on_ground;
    extern PFN_avTVSensor_void_method g_avTVSensor_search;
    extern PFN_avTVSensor_void_method g_avTVSensor_uncage;
    extern void (__fastcall*          g_avTVSensor_slew_stop)(void* self);
    extern void (__fastcall*          g_avTVSensor_zoom_in)(void* self);
    extern void (__fastcall*          g_avTVSensor_zoom_out)(void* self);
    extern void**                    g_contexts_ptr_addr;
    extern void*                     g_avTVSensor_instance;
    extern PFN_avPlatform_ctor       g_avPlatform_ctor;
    extern PFN_avPlatform_set_carrier g_avPlatform_set_carrier;
    extern PFN_avPlatform_update_pos g_avPlatform_update_pos;
    extern PFN_binding_context       g_binding_context;
    extern void*                     g_stub_platform;

    void Log_(const char* msg);
    bool Initialize();
    bool InjectIRADSSensor_Early();   // Phase A: DllMain — ctor + add to keeper
    bool InjectIRADSSensor_Late();    // Phase B: ed_fm_start — ensure init + camera pos + IR mode
    bool HandleIRADSCommand(int command, float value);
    void UpdateIRADSSensorFrame(double dt,
        double aircraft_x, double aircraft_y, double aircraft_z,
        double quat_x, double quat_y, double quat_z, double quat_w);
    void ForceIRADSIndicatorActive();  // Phase C: called every frame from ed_fm_simulate to keep screenCondition/cameraCondition live
    bool CaptureIRADSLock();
    bool HasIRADSLock();
    bool GetIRADSLockWorldPoint(double out_point[3]);
    bool GetIRADSLockPolar(double out_polar[3]);

} // namespace CockpitInterop
