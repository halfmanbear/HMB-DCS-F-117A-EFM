// Resolves the avTVSensor entry points and the cockpit context table from CockpitBase.dll.
// avTVSensor is NOT an avDevice: never register/start it through devices_keeper.
#include "stdafx.h"
#include "IradsInternal.h"

namespace Cockpit_interop
{
HMODULE g_h_dll = nullptr;
Pfn_av_tv_sensor_ctor g_av_tv_sensor_ctor = nullptr;
Pfn_av_tv_sensor_initialize g_av_tv_sensor_initialize = nullptr;
Pfn_av_tv_sensor_init_zoom g_av_tv_sensor_init_zoom = nullptr;
Pfn_av_tv_sensor_update g_av_tv_sensor_update = nullptr;
Pfn_av_tv_sensor_update_frame g_av_tv_sensor_update_frame = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_left = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_right = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_up = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_down = nullptr;
Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_horizontal_abs = nullptr;
Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_vertical_abs = nullptr;
Pfn_av_tv_sensor_set_external_designation g_av_tv_sensor_set_external_designation = nullptr;
Pfn_av_tv_sensor_stabilize_on_ground g_av_tv_sensor_stabilize_on_ground = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_search = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_uncage = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_slew_stop = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_in = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_out = nullptr;
Pfn_binding_context g_binding_context = nullptr;
void** g_contexts_ptr_addr = nullptr;
void* g_av_tv_sensor_instance = nullptr;

static bool s_initialized = false;

// g_hDLL is our EFM module (used for the log path), not CockpitBase.
FARPROC resolve_cockpit_base_export(const char* name)
{
    const HMODULE module = GetModuleHandleA("CockpitBase.dll");
    return module ? GetProcAddress(module, name) : nullptr;
}

Cc_cockpit_context_layout* get_global_context()
{
    return (g_contexts_ptr_addr && *g_contexts_ptr_addr) ? as_context(*g_contexts_ptr_addr) : nullptr;
}

Cc_cockpit_context_layout* get_binding_context()
{
    return g_binding_context ? as_context(g_binding_context()) : nullptr;
}

template <typename Fn>
static void load_export(HMODULE module, Fn& target, const char* name)
{
    target = reinterpret_cast<Fn>(GetProcAddress(module, name));
    log_format("Initialize: %-60s -> %s", name, target ? "ok" : "missing");
}

// The exports the sensor bridge cannot work without.
static bool required_exports_present()
{
    return g_av_tv_sensor_ctor && g_av_tv_sensor_initialize && g_contexts_ptr_addr && g_av_tv_sensor_update &&
           g_av_tv_sensor_update_frame && g_av_tv_sensor_zoom_in && g_av_tv_sensor_zoom_out;
}

static void load_sensor_exports(HMODULE module)
{
    load_export(module, g_av_tv_sensor_ctor, "??0avTVSensor@cockpit@@QEAA@XZ");
    load_export(module, g_av_tv_sensor_initialize, "?initialize@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_init_zoom, "?initZoom@avTVSensor@cockpit@@MEAAXXZ");
    load_export(module, g_av_tv_sensor_update, "?update@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_update_frame, "?update_frame@avTVSensor@cockpit@@MEAAXXZ");
    load_export(module, g_av_tv_sensor_slew_left, "?slew_left@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_right, "?slew_right@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_up, "?slew_up@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_down, "?slew_down@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_move_horizontal_abs,
                "?on_TV_SENSOR_move_horizontal_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
    load_export(module, g_av_tv_sensor_move_vertical_abs,
                "?on_TV_SENSOR_move_vertical_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
    load_export(module, g_av_tv_sensor_slew_stop, "?slew_stop@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_search, "?search@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_uncage, "?uncage@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_stabilize_on_ground, "?stabilizeOnGround@avTVSensor@cockpit@@UEAAX_N@Z");
    load_export(module, g_av_tv_sensor_zoom_in, "?zoom_in@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_zoom_out, "?zoom_out@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_binding_context, "?binding_context@cockpit@@YAPEAVccCockpitContext@1@XZ");
    g_contexts_ptr_addr =
        reinterpret_cast<void**>(GetProcAddress(module, "?contexts_ptr@ccCockpitContext@cockpit@@0PAPEAV12@A"));
    load_export(module, g_av_tv_sensor_set_external_designation,
                "?set_external_designation@avTVSensor@cockpit@@UEAAX_N0@Z");
}

static void warn_about_missing_exports()
{
    if (!g_av_tv_sensor_initialize)
        log_message("Initialize: WARNING - avTVSensor::initialize not exported; "
                    "standalone sensor cannot initialize");
    if (!g_av_tv_sensor_init_zoom)
        log_message("Initialize: WARNING - avTVSensor::initZoom not exported; "
                    "will fall back to direct zoom reset");
    if (!g_av_tv_sensor_move_horizontal_abs || !g_av_tv_sensor_move_vertical_abs)
        log_message("Initialize: WARNING - avTVSensor absolute-axis exports missing; "
                    "will fall back to directional slew handlers");
    if (!g_av_tv_sensor_search)
        log_message(
            "Initialize: WARNING - avTVSensor::search not exported; designation can only capture existing tracks");
    if (!g_av_tv_sensor_stabilize_on_ground)
        log_message("Initialize: WARNING - avTVSensor::stabilizeOnGround not exported; "
                    "ground designation will fall back to search");
}

bool initialize()
{
    if (s_initialized)
        return required_exports_present();

    HMODULE module = GetModuleHandleA("CockpitBase.dll");
    log_format("Initialize: build marker %s", irads_build_marker);
    if (!module)
    {
        log_message("Initialize: CockpitBase.dll not yet loaded - will retry");
        return false;
    }

    load_sensor_exports(module);
    log_format("Initialize: contexts_ptr=%s  context=%s  binding_context=%s", g_contexts_ptr_addr ? "ok" : "missing",
               g_contexts_ptr_addr && *g_contexts_ptr_addr ? "ok" : "missing",
               g_binding_context && g_binding_context() ? "ok" : "missing");

    s_initialized = true;

    const bool ok = required_exports_present();
    log_format(ok ? "Initialize: OK" : "Initialize: PARTIAL - some exports missing");
    warn_about_missing_exports();
    return ok;
}
}  // namespace Cockpit_interop
