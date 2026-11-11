#include "stdafx.h"
#include "ParamBridge.h"

#include "../FlightModel/State.h"

// Lazily resolve and publish a cockpit param by name.
void publish_named_param(void*& handle, const char* name, double value)
{
    if (handle == nullptr)
    {
        F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpit_api.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }
        handle = F117::cockpit_api.getParamHandle(name);
    }
    if (handle != nullptr)
    {
        F117::cockpit_api.setParamNumber(handle, value);
    }
}

// Lazily resolve and read a cockpit param that the mod's Lua devices publish. False without the cockpit API.
bool read_named_param(void*& handle, const char* name, double& value)
{
    if (handle == nullptr)
    {
        F117::cockpit_api.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpit_api.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpit_api.ed_param_api.pfn_ed_cockpit_parameter_value_to_number == nullptr)
        {
            return false;
        }
        handle = F117::cockpit_api.getParamHandle(name);
    }
    if (handle == nullptr)
    {
        return false;
    }
    value = F117::cockpit_api.getParamNumber(handle);
    return true;
}

// Start air for the air turbine starters from the APU or an external air cart. Crossbleed from a running
// engine is handled by ENGINE_START::Starts. The mod's Systems/Electrical.lua owns both: F117_APU_STATE 2 is
// the APU running at normal output pressure, and F117_GROUND_AIR 1 is an air cart connected.
bool external_start_air()
{
    static void* s_apu_state_param = nullptr;
    static void* s_ground_air_param = nullptr;
    double apu = 0.0;
    double cart = 0.0;
    const bool apu_air = read_named_param(s_apu_state_param, "F117_APU_STATE", apu) && apu > 1.5;
    const bool cart_air = read_named_param(s_ground_air_param, "F117_GROUND_AIR", cart) && cart > 0.5;
    return apu_air || cart_air;
}
