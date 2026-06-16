// dllmain.cpp — DLL entry point
//
// Phase A (DLL_PROCESS_ATTACH): construct avTVSensor + devices_keeper::add()
//   Runs BEFORE device_init.lua so create_indicator() caches our avTVSensor*
//   when it binds the DLIR COMMON indicator to device slot 9.
//
// Phase B runs later from ed_fm_cold/hot_start (after cockpit is live).

#include "stdafx.h"
#include "Avionics/CockpitBase_Interop.h"

BOOL APIENTRY DllMain(HMODULE hModule,
                      DWORD   ul_reason_for_call,
                      LPVOID  lpReserved)
{
    if (ul_reason_for_call == DLL_PROCESS_ATTACH)
    {
        OutputDebugStringA("F117: dllmain hit\n");
        CockpitInterop::g_hDLL = hModule;
        CockpitInterop::Log_("DllMain: DLL_PROCESS_ATTACH");
        CockpitInterop::InjectIRADSSensor_Early();
    }
    return TRUE;
}
