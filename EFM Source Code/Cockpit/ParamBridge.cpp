#include "stdafx.h"
#include "ParamBridge.h"

#include "../FlightModel/State.h"

// Lazily resolve and publish a cockpit param by name.
void publish_named_param(void*& handle, const char* name, double value)
{
    if (handle == nullptr)
    {
        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }
        handle = F117::cockpitAPI.getParamHandle(name);
    }
    if (handle != nullptr)
    {
        F117::cockpitAPI.setParamNumber(handle, value);
    }
}

// Lazily resolve and read a cockpit param that the mod's Lua devices publish. False without the cockpit API.
bool read_named_param(void*& handle, const char* name, double& value)
{
    if (handle == nullptr)
    {
        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_parameter_value_to_number == nullptr)
        {
            return false;
        }
        handle = F117::cockpitAPI.getParamHandle(name);
    }
    if (handle == nullptr)
    {
        return false;
    }
    value = F117::cockpitAPI.getParamNumber(handle);
    return true;
}

// Start air for the air turbine starters from the APU or an external air cart. Crossbleed from a running
// engine is handled by ENGINE_START::Starts. The mod's Systems/Electrical.lua owns both: F117_APU_STATE 2 is
// the APU running at normal output pressure, and F117_GROUND_AIR 1 is an air cart connected.
bool external_start_air()
{
    static void* s_apuStateParam = nullptr;
    static void* s_groundAirParam = nullptr;
    double apu = 0.0;
    double cart = 0.0;
    const bool apuAir = read_named_param(s_apuStateParam, "F117_APU_STATE", apu) && apu > 1.5;
    const bool cartAir = read_named_param(s_groundAirParam, "F117_GROUND_AIR", cart) && cart > 0.5;
    return apuAir || cartAir;
}
