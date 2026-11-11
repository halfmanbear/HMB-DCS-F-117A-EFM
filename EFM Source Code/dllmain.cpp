// dllmain.cpp — DLL entry point
//
// Phase A (DLL_PROCESS_ATTACH): construct avTVSensor + devices_keeper::add()
//   Runs BEFORE device_init.lua so create_indicator() caches our avTVSensor*
//   when it binds the DLIR COMMON indicator to device slot 9.
//
// Phase B runs later from ed_fm_cold/hot_start (after cockpit is live).

#include "stdafx.h"
#include "Avionics/CockpitBase_Interop.h"

BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID)
{
    if (reason != DLL_PROCESS_ATTACH)
        return TRUE;
    OutputDebugStringA("F117: dllmain hit\n");
    Cockpit_interop::g_h_dll = instance;
    Cockpit_interop::log_message("DllMain: DLL_PROCESS_ATTACH");
    Cockpit_interop::inject_irads_sensor_early();
    return TRUE;
}
