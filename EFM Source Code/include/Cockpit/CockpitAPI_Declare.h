#pragma once
#include "ccParametersAPI.h" // ED's parameter API function table
#include <string>
#include <vector>

// A small checked adapter over the ED function pointers. The table can be
// refreshed by the caller once CockpitBase.dll becomes available.
struct EDPARAM final
{
    cockpit_param_api ed_param_api{ed_get_cockpit_param_api()};

    void* getParamHandle(const char* name) const
    {
        const auto lookup = ed_param_api.pfn_ed_cockpit_get_parameter_handle;
        return lookup && name ? lookup(name) : nullptr;
    }

    void setParamNumber(void* handle, double number) const
    {
        const auto update = ed_param_api.pfn_ed_cockpit_update_parameter_with_number;
        if (handle && update) update(handle, number);
    }

    void setParamString(void* handle, const char* text) const
    {
        const auto update = ed_param_api.pfn_ed_cockpit_update_parameter_with_string;
        if (handle && text && update) update(handle, text);
    }

    double getParamNumber(void* handle) const
    {
        double number{};
        const auto read = ed_param_api.pfn_ed_cockpit_parameter_value_to_number;
        if (!handle || !read || !read(handle, number, false)) return 0.0;
        return number;
    }

    // Return owned storage, rather than a pointer into a dead stack buffer.
    std::string getParamString(void* handle, unsigned capacity) const
    {
        const auto read = ed_param_api.pfn_ed_cockpit_parameter_value_to_string;
        if (!handle || !read || capacity == 0) return {};
        std::vector<char> text(capacity, '\0');
        if (!read(handle, text.data(), capacity)) return {};
        text.back() = '\0';
        return std::string(text.data());
    }
};
