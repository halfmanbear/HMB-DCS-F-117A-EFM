// Bridges the DLIR indicator to the injected avTVSensor: finds the camera render-target indicator,
// keeps it in FLIR mode, and replaces its get_camera so the render uses our sensor's pose.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>

namespace Cockpit_interop
{
// The indicator's function table is copied whole, so its length is measured: entries are counted
// while they point into executable code. The type-information entry of the next table ends it.
static constexpr size_t max_vtable_entries = 2048;

// Render-aim diagnostics: log the first frame, then every kRenderLogInterval-th, up to a cap.
static constexpr unsigned render_log_limit = 120;
static constexpr unsigned render_log_interval = 120;

using Indicator_get_camera = void*(__fastcall*)(void*);
static Indicator_get_camera s_original_get_camera = nullptr;
using Sensor_fill_camera = void(__fastcall*)(void*, void*);
static Sensor_fill_camera s_sensor_fill_camera = nullptr;
// Returns the pose fill_camera renders from: elements 0-2 forward axis, 12-14 eye position.
using Sensor_get_camera_position = double*(__fastcall*)(const void*, double* pose16);
static Sensor_get_camera_position s_sensor_get_camera_position = nullptr;
static void* s_camera_bridge_indicator = nullptr;
static void** s_original_vtable = nullptr;
static void** s_vtable_copy = nullptr;  // allocation; the table itself starts one entry in

static bool is_executable(const void* address)
{
    MEMORY_BASIC_INFORMATION info;
    if (!VirtualQuery(address, &info, sizeof(info)) || info.State != MEM_COMMIT)
        return false;
    constexpr DWORD executable = PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    return (info.Protect & executable) != 0;
}

static size_t vtable_length(void* const* vtable)
{
    size_t count = 0;
    while (count < max_vtable_entries && vtable[count] && is_executable(vtable[count]))
        ++count;
    return count;
}

// Exported accessors used instead of fixed field offsets, so the search survives layout changes.
using Indicator_get_render_target = int(__fastcall*)(const void*);
static Indicator_get_render_target s_get_render_target = nullptr;
// DCS's list of render-target sources: entry N is the indicator that draws render target N.
static const Pointer_vector_view* s_render_targets = nullptr;
using Indicator_get_element = void*(__fastcall*)(void*, const char*);
static Indicator_get_element s_get_element = nullptr;
// Marks the indicator's render target as in use this frame; the next final draw pass clears it.
using Indicator_increase_render_target_counter = void(__fastcall*)(void* element, void* indicator, const void* params);
static Indicator_increase_render_target_counter s_increase_render_target_counter = nullptr;
using Indicator_get_screen_condition = int(__fastcall*)(void*);
static Indicator_get_screen_condition s_get_screen_condition = nullptr;

// Element on DLIR_Camera_page.lua that identifies the camera indicator.
static constexpr const char* camera_marker = "F117_IRADS_CAMERA_SOURCE";

static bool resolve_search_exports()
{
    if (s_get_render_target && s_render_targets && s_get_element && s_increase_render_target_counter &&
        s_get_screen_condition)
        return true;
    s_get_render_target = reinterpret_cast<Indicator_get_render_target>(
        resolve_cockpit_base_export("?get_render_target@ccIndicator@cockpit@@UEBA?BHXZ"));
    s_render_targets = reinterpret_cast<const Pointer_vector_view*>(
        unique_export(GetModuleHandleA("CockpitBase.dll"), "?render_targets@ccDrawable@cockpit@@"));
    s_get_element = reinterpret_cast<Indicator_get_element>(
        resolve_cockpit_base_export("?get_element_on_current_page@ccIndicator@cockpit@@UEAAPEAVElement@2@PEBD@Z"));
    s_increase_render_target_counter = reinterpret_cast<Indicator_increase_render_target_counter>(
        unique_export(GetModuleHandleA("CockpitBase.dll"), "?increase_render_target_counter@ccIndicator@cockpit@@"));
    s_get_screen_condition = reinterpret_cast<Indicator_get_screen_condition>(
        unique_export(GetModuleHandleA("CockpitBase.dll"), "?get_screen_condition@ccIndicator@cockpit@@"));
    return s_get_render_target && s_render_targets && s_get_element && s_increase_render_target_counter &&
           s_get_screen_condition;
}

// -- Indicator search ------------------------------------------------------

// The DLIR camera is the render target 0 source whose page carries the camera marker. Every
// indicator with a render target registers itself in DCS's render target list when its
// materials are set up, so no cockpit context walk is needed.
static void* find_camera_indicator()
{
    void** begin = s_render_targets->begin;
    void** end = s_render_targets->end;
    if (!begin || !end || end <= begin)
        return nullptr;
    for (void** p = begin; p < end; ++p)
    {
        void* indicator = *p;
        if (indicator && s_get_render_target(indicator) == 0 && s_get_element(indicator, camera_marker))
            return indicator;
    }
    return nullptr;
}

static void locate_camera_indicator()
{
    static bool s_indicator_search_logged = false;
    if (!resolve_search_exports())
    {
        if (!s_indicator_search_logged)
            log_message("ForceIRADSIndicatorActive: indicator search exports unavailable; disabled");
        s_indicator_search_logged = true;
        return;
    }
    s_our_ind = find_camera_indicator();
    if (s_our_ind)
    {
        log_message("ForceIRADSIndicatorActive: found camera render target source");
        return;
    }
    if (!s_indicator_search_logged)
    {
        const size_t count = s_render_targets->end > s_render_targets->begin
                                 ? static_cast<size_t>(s_render_targets->end - s_render_targets->begin)
                                 : 0u;
        log_format("ForceIRADSIndicatorActive: no camera indicator yet (%zu render targets)", count);
        s_indicator_search_logged = true;
    }
}

// -- Camera callback -------------------------------------------------------

static void* __fastcall irads_get_camera(void* indicator)
{
    void* camera = s_original_get_camera(indicator);
    if (!camera || !g_av_tv_sensor_instance)
        return camera;
    // The native camera extrapolates the pose from mount and motor history when
    // the platform was last updated at an earlier model time, which would
    // overwrite a locked aim. Bring the platform up to the current model time
    // first, then aim: the camera then uses the stored, aimed pose.
    bool locked = false;
    if (s_irads_lock_valid)
    {
        g_platform_update_position(g_av_tv_sensor_instance);
        locked = aim_pose_at_world_point(sensor_world_pose(), s_irads_lock_world);
    }
    s_sensor_fill_camera(g_av_tv_sensor_instance, camera);
    // Read the same pose the camera was just filled from.
    double pose[16];
    s_sensor_get_camera_position(g_av_tv_sensor_instance, pose);
    const double* forward = pose;
    const double* eye = pose + 12;
    // Snapshot the visible ray. Terrain tracing must use this optical eye,
    // not a native cached target or a newly recomputed aircraft-forward ray.
    if (is_irads_powered())
    {
        s_irads_ray_valid = true;
        for (int i = 0; i < 3; ++i)
        {
            s_irads_render_ray[i] = eye[i];
            s_irads_render_ray[i + 3] = forward[i];
            if (!std::isfinite(eye[i]) || !std::isfinite(forward[i]))
                s_irads_ray_valid = false;
        }
        if (s_irads_ray_valid)
            s_irads_ray_age = 0;
    }
    static unsigned render_samples = 0;
    static unsigned render_frames = 0;
    if (locked && render_samples < render_log_limit &&
        (++render_frames == 1 || render_frames % render_log_interval == 0))
    {
        // Observe the FINAL rendered ray, not just the cached pose.
        double dot = 0;
        double range_sq = 0;
        double axis_sq = 0;
        for (int i = 0; i < 3; ++i)
        {
            const double d = s_irads_lock_world[i] - eye[i];
            dot += d * forward[i];
            range_sq += d * d;
            axis_sq += forward[i] * forward[i];
        }
        if (range_sq > 1 && axis_sq > 0.5)
        {
            double cosine = dot / std::sqrt(range_sq * axis_sq);
            cosine = cosine > 1 ? 1 : (cosine < -1 ? -1 : cosine);
            log_format("IRADS render aim: errorDeg=%.6f range=%.1f eye=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f)",
                       std::acos(cosine) * rad_to_deg, std::sqrt(range_sq), eye[0], eye[1], eye[2],
                       s_irads_lock_world[0], s_irads_lock_world[1], s_irads_lock_world[2]);
            ++render_samples;
        }
    }
    return camera;
}

static bool install_irads_camera_bridge(void* indicator)
{
    if (s_camera_bridge_indicator == indicator)
        return true;
    auto native_get_camera = reinterpret_cast<Indicator_get_camera>(
        resolve_cockpit_base_export("?get_camera@ccIndicator@cockpit@@UEAAAEBVCamera@Graphics@@XZ"));
    auto** vtable = *reinterpret_cast<void***>(indicator);
    if (!native_get_camera || !vtable)
        return false;
    s_sensor_fill_camera = reinterpret_cast<Sensor_fill_camera>(
        resolve_cockpit_base_export("?fill_camera@avTVSensor@cockpit@@UEBAXAEAVCamera@Graphics@@@Z"));
    s_sensor_get_camera_position = reinterpret_cast<Sensor_get_camera_position>(
        unique_export(GetModuleHandleA("CockpitBase.dll"), "?get_camera_position@avTVSensor@cockpit@@"));
    if (!s_sensor_fill_camera || !s_sensor_get_camera_position)
    {
        log_message("IRADS camera bridge: native fill_camera/get_camera_position unavailable");
        return false;
    }
    const size_t length = vtable_length(vtable);
    size_t slot = length;
    for (size_t i = 0; i < length; ++i)
        if (vtable[i] == reinterpret_cast<void*>(native_get_camera))
        {
            slot = i;
            break;
        }
    if (slot == length || length == max_vtable_entries)
    {
        log_format("IRADS camera bridge: get_camera slot not found in %zu entries; disabled", length);
        return false;
    }
    // Keep the type information entry ahead of the copied table for native RTTI.
    auto** copy = new void*[length + 1];
    std::memcpy(copy, vtable - 1, sizeof(void*) * (length + 1));
    s_original_get_camera = native_get_camera;
    copy[slot + 1] = reinterpret_cast<void*>(&irads_get_camera);
    s_original_vtable = vtable;
    s_vtable_copy = copy;
    *reinterpret_cast<void***>(indicator) = copy + 1;
    s_camera_bridge_indicator = indicator;
    log_format("IRADS camera bridge installed: slot=%zu of %zu sensor=%s", slot, length,
               g_av_tv_sensor_instance ? "ok" : "missing");
    return true;
}

// The base indicator has no screen-condition setter, so the FLIR condition is written to
// its field and accepted only once the exported getter reads the same value back. If it
// does not, the original value is restored and the next frames retry; after the retry
// limit the camera stays off.
static constexpr int screen_condition_check_limit = 300;
static int s_screen_condition_checks = 0;
static bool s_screen_condition_verified = false;

static bool write_flir_screen_condition(void* indicator)
{
    const size_t offset = screen_condition_offset();
    if (!offset)
        return false;
    uint32_t& field = *reinterpret_cast<uint32_t*>(static_cast<char*>(indicator) + offset);
    if (s_screen_condition_verified)
    {
        field = static_cast<uint32_t>(ir_screen_condition_flir);
        return true;
    }
    if (s_screen_condition_checks >= screen_condition_check_limit)
        return false;
    ++s_screen_condition_checks;
    const uint32_t saved = field;
    s_increase_render_target_counter(nullptr, indicator, nullptr);  // the getter reports only targets in use
    field = static_cast<uint32_t>(ir_screen_condition_flir);
    if (s_get_screen_condition(indicator) == ir_screen_condition_flir)
    {
        s_screen_condition_verified = true;
        log_format("IRADS screen condition field verified after %d check(s)", s_screen_condition_checks);
        return true;
    }
    field = saved;
    if (s_screen_condition_checks == screen_condition_check_limit)
        log_message("IRADS screen condition field not confirmed; camera picture disabled");
    return false;
}

// Phase C: called every frame from ed_fm_simulate. Once indicators are populated (after
// device_init.lua) it locates the camera render-target indicator, installs the camera bridge,
// and every frame forces screenCondition = FLIR (11) and cameraCondition = ON (1) so DCS can't
// revert the indicator to a non-camera state.
void force_irads_indicator_active()
{
    if (!g_av_tv_sensor_instance)
        return;

    if (!s_our_ind)
        locate_camera_indicator();

    if (s_our_ind)
    {
        if (!install_irads_camera_bridge(s_our_ind))
            return;
        if (!write_flir_screen_condition(s_our_ind))
            return;
        // Powered: mark the camera target in use so the indicator reports FLIR this frame.
        // Unpowered: no mark, so the next final draw pass leaves it off.
        if (is_irads_powered())
            s_increase_render_target_counter(nullptr, s_our_ind, nullptr);
        static int s_logged_condition = -1;
        static unsigned s_condition_logs = 0;
        const int condition = s_get_screen_condition(s_our_ind);
        if (condition != s_logged_condition && s_condition_logs < 40)
        {
            log_format("IRADS screen condition: %d (power=%d)", condition, is_irads_powered() ? 1 : 0);
            s_logged_condition = condition;
            ++s_condition_logs;
        }
    }
}

// An indicator leaves the render target list when DCS releases it, so a listed indicator is
// still alive and gets its own table back. An unlisted one may still be mid-teardown and using
// the copy, so the copy is then left allocated.
static void restore_camera_vtable()
{
    void* indicator = s_camera_bridge_indicator;
    if (!indicator || !s_vtable_copy)
        return;
    bool listed = false;
    if (s_render_targets && s_render_targets->begin && s_render_targets->end > s_render_targets->begin)
        for (void** p = s_render_targets->begin; p < s_render_targets->end; ++p)
            if (*p == indicator)
                listed = true;
    void**& vptr = *reinterpret_cast<void***>(indicator);
    if (listed && vptr == s_vtable_copy + 1)
    {
        vptr = s_original_vtable;
        delete[] s_vtable_copy;
        log_message("IRADS camera bridge removed");
    }
    else
    {
        log_message("IRADS camera bridge: indicator already released; table copy kept");
    }
    s_vtable_copy = nullptr;
    s_original_vtable = nullptr;
}

void reset_camera_bridge()
{
    restore_camera_vtable();
    s_screen_condition_checks = 0;
    s_screen_condition_verified = false;
    s_camera_bridge_indicator = nullptr;
    s_our_ind = nullptr;
}
}  // namespace Cockpit_interop
