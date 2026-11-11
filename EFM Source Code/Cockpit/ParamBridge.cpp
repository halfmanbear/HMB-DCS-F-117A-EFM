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
