// Resolves the avTVSensor entry points and the cockpit context table from CockpitBase.dll.
// avTVSensor is NOT an avDevice: never register/start it through devices_keeper.
#include "stdafx.h"
#include "IradsInternal.h"

namespace CockpitInterop
{
    HMODULE                    g_hDLL                           = nullptr;
    PFN_avTVSensor_ctor        g_avTVSensor_ctor                = nullptr;
    PFN_avTVSensor_initialize  g_avTVSensor_initialize          = nullptr;
    PFN_avTVSensor_initZoom    g_avTVSensor_initZoom            = nullptr;
    PFN_avTVSensor_update      g_avTVSensor_update              = nullptr;
    PFN_avTVSensor_update_frame g_avTVSensor_update_frame       = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_left           = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_right          = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_up             = nullptr;
    PFN_avTVSensor_slew_axis   g_avTVSensor_slew_down           = nullptr;
    PFN_avTVSensor_abs_axis    g_avTVSensor_move_horizontal_abs = nullptr;
    PFN_avTVSensor_abs_axis    g_avTVSensor_move_vertical_abs   = nullptr;
    PFN_avTVSensor_set_external_designation g_avTVSensor_set_external_designation = nullptr;
    PFN_avTVSensor_stabilize_on_ground g_avTVSensor_stabilize_on_ground = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_search              = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_uncage              = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_slew_stop           = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_zoom_in             = nullptr;
    PFN_avTVSensor_void_method g_avTVSensor_zoom_out            = nullptr;
    PFN_binding_context        g_binding_context                = nullptr;
    void**                     g_contexts_ptr_addr              = nullptr;
    void*                      g_avTVSensor_instance            = nullptr;

    static bool s_initialized = false;

    // g_hDLL is our EFM module (used for the log path), not CockpitBase.
    FARPROC ResolveCockpitBaseExport(const char* name)
    {
        const HMODULE module = GetModuleHandleA("CockpitBase.dll");
        return module ? GetProcAddress(module, name) : nullptr;
    }

    CcCockpitContextLayout* GetGlobalContext()
    {
        return (g_contexts_ptr_addr && *g_contexts_ptr_addr) ? AsContext(*g_contexts_ptr_addr) : nullptr;
    }

    CcCockpitContextLayout* GetBindingContext()
    {
        return g_binding_context ? AsContext(g_binding_context()) : nullptr;
    }

    template <typename Fn>
    static void load_export(HMODULE module, Fn& target, const char* name)
    {
        target = reinterpret_cast<Fn>(GetProcAddress(module, name));
        Log("Initialize: %-60s -> %p", name, reinterpret_cast<void*>(target));
    }

    // The exports the sensor bridge cannot work without.
    static bool required_exports_present()
    {
        return g_avTVSensor_ctor && g_avTVSensor_initialize && g_contexts_ptr_addr &&
               g_avTVSensor_update && g_avTVSensor_update_frame &&
               g_avTVSensor_zoom_in && g_avTVSensor_zoom_out;
    }

    static void load_sensor_exports(HMODULE module)
    {
        load_export(module, g_avTVSensor_ctor,         "??0avTVSensor@cockpit@@QEAA@XZ");
        load_export(module, g_avTVSensor_initialize,   "?initialize@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_avTVSensor_initZoom,     "?initZoom@avTVSensor@cockpit@@MEAAXXZ");
        load_export(module, g_avTVSensor_update,       "?update@avTVSensor@cockpit@@UEAAXN@Z");
        load_export(module, g_avTVSensor_update_frame, "?update_frame@avTVSensor@cockpit@@MEAAXXZ");
        load_export(module, g_avTVSensor_slew_left,    "?slew_left@avTVSensor@cockpit@@UEAAXN@Z");
        load_export(module, g_avTVSensor_slew_right,   "?slew_right@avTVSensor@cockpit@@UEAAXN@Z");
        load_export(module, g_avTVSensor_slew_up,      "?slew_up@avTVSensor@cockpit@@UEAAXN@Z");
        load_export(module, g_avTVSensor_slew_down,    "?slew_down@avTVSensor@cockpit@@UEAAXN@Z");
        load_export(module, g_avTVSensor_move_horizontal_abs,
                    "?on_TV_SENSOR_move_horizontal_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
        load_export(module, g_avTVSensor_move_vertical_abs,
                    "?on_TV_SENSOR_move_vertical_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
        load_export(module, g_avTVSensor_slew_stop,    "?slew_stop@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_avTVSensor_search,       "?search@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_avTVSensor_uncage,       "?uncage@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_avTVSensor_stabilize_on_ground, "?stabilizeOnGround@avTVSensor@cockpit@@UEAAX_N@Z");
        load_export(module, g_avTVSensor_zoom_in,      "?zoom_in@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_avTVSensor_zoom_out,     "?zoom_out@avTVSensor@cockpit@@UEAAXXZ");
        load_export(module, g_binding_context,         "?binding_context@cockpit@@YAPEAVccCockpitContext@1@XZ");
        g_contexts_ptr_addr = reinterpret_cast<void**>(
            GetProcAddress(module, "?contexts_ptr@ccCockpitContext@cockpit@@0PAPEAV12@A"));
        load_export(module, g_avTVSensor_set_external_designation,
                    "?set_external_designation@avTVSensor@cockpit@@UEAAX_N0@Z");
    }

    static void warn_about_missing_exports()
    {
        if (!g_avTVSensor_initialize)
            Log_("Initialize: WARNING - avTVSensor::initialize not exported; "
                 "standalone sensor cannot initialize");
        if (!g_avTVSensor_initZoom)
            Log_("Initialize: WARNING - avTVSensor::initZoom not exported; "
                 "will fall back to direct zoom reset");
        if (!g_avTVSensor_move_horizontal_abs || !g_avTVSensor_move_vertical_abs)
            Log_("Initialize: WARNING - avTVSensor absolute-axis exports missing; "
                 "will fall back to directional slew handlers");
        if (!g_avTVSensor_search)
            Log_("Initialize: WARNING - avTVSensor::search not exported; designation can only capture existing tracks");
        if (!g_avTVSensor_stabilize_on_ground)
            Log_("Initialize: WARNING - avTVSensor::stabilizeOnGround not exported; "
                 "ground designation will fall back to search");
    }

    bool Initialize()
    {
        if (s_initialized)
            return required_exports_present();

        HMODULE module = GetModuleHandleA("CockpitBase.dll");
        Log("Initialize: build marker %s", kIRADSBuildMarker);
        if (!module) { Log_("Initialize: CockpitBase.dll not yet loaded - will retry"); return false; }

        load_sensor_exports(module);
        Log("Initialize: contexts_ptr addr=%p  *value=%p  binding_context=%p",
            reinterpret_cast<void*>(g_contexts_ptr_addr),
            g_contexts_ptr_addr ? *g_contexts_ptr_addr : nullptr,
            g_binding_context ? g_binding_context() : nullptr);

        s_initialized = true;

        const bool ok = required_exports_present();
        Log(ok ? "Initialize: OK" : "Initialize: PARTIAL - some exports missing");
        warn_about_missing_exports();
        return ok;
    }
} // namespace CockpitInterop
