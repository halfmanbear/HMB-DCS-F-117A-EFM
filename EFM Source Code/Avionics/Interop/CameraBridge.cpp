// Bridges the DLIR indicator to the injected avTVSensor: finds the camera render-target indicator,
// keeps it in FLIR mode, and replaces its get_camera so the render uses our sensor's pose.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace Cockpit_interop
{
static constexpr int max_context_depth = 8;
static constexpr size_t vtable_slot_count = 128;

// Native camera object layout: eye position and forward vector (float triples).
static constexpr size_t camera_eye_offset = 0x260;
static constexpr size_t camera_forward_offset = 0x230;

// Render-aim diagnostics: log the first frame, then every kRenderLogInterval-th, up to a cap.
static constexpr unsigned render_log_limit = 120;
static constexpr unsigned render_log_interval = 120;

using Indicator_get_camera = void*(__fastcall*)(void*);
static Indicator_get_camera s_original_get_camera = nullptr;
using Sensor_fill_camera = void(__fastcall*)(void*, void*);
static Sensor_fill_camera s_sensor_fill_camera = nullptr;
static void* s_camera_bridge_indicator = nullptr;

// -- Indicator search ------------------------------------------------------

// The DLIR camera render target is the indicator of type 0 with render target 0.
static void* camera_indicator_in(const Cc_cockpit_context_layout* context)
{
    void** begin = context->indicators.begin;
    void** end = context->indicators.end;
    if (begin && end && end > begin)
    {
        for (void** p = begin; p < end; ++p)
        {
            void* indicator = *p;
            if (indicator)
            {
                const Cc_indicator_layout* view = as_indicator(indicator);
                if (view->indicator_type == 0 && static_cast<int32_t>(view->render_target_id) == 0)
                    return indicator;
            }
        }
    }
    return nullptr;
}

// Depth-first, pre-order walk of the context tree, bounded by kMaxContextDepth.
static void* find_camera_indicator(Cc_cockpit_context_layout* root, int* out_depth)
{
    struct Frame
    {
        void** next;
        void** end;
    };
    Frame stack[max_context_depth + 1] = {};
    int depth = -1;
    Cc_cockpit_context_layout* pending = root;
    for (;;)
    {
        if (pending)
        {
            const int pending_depth = depth + 1;
            if (pending_depth <= max_context_depth)
            {
                void* found = camera_indicator_in(pending);
                if (found)
                {
                    *out_depth = pending_depth;
                    return found;
                }
                void** begin = pending->child_contexts.begin;
                void** end = pending->child_contexts.end;
                const bool has_children = begin && end && end > begin;
                depth = pending_depth;
                stack[depth] = {has_children ? begin : nullptr, has_children ? end : nullptr};
            }
            pending = nullptr;
        }
        while (depth >= 0 && stack[depth].next == stack[depth].end)
            --depth;
        if (depth < 0)
            return nullptr;
        pending = as_context(*stack[depth].next);
        ++stack[depth].next;
    }
}

static void locate_camera_indicator()
{
    static bool s_indicator_search_logged = false;
    int found_depth = -1;
    Cc_cockpit_context_layout* global_ctx = get_global_context();
    Cc_cockpit_context_layout* binding_ctx = get_binding_context();

    s_our_ind = find_camera_indicator(global_ctx, &found_depth);
    if (s_our_ind)
    {
        log_format("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via global ctx", s_our_ind,
                   found_depth);
        return;
    }
    if (binding_ctx != global_ctx)
    {
        s_our_ind = find_camera_indicator(binding_ctx, &found_depth);
        if (s_our_ind)
        {
            log_format("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via binding ctx", s_our_ind,
                       found_depth);
            return;
        }
    }
    if (!s_indicator_search_logged)
    {
        log_format("ForceIRADSIndicatorActive: no indicator found yet (global=%p binding=%p)",
                   static_cast<void*>(global_ctx), static_cast<void*>(binding_ctx));
        s_indicator_search_logged = true;
    }
}

// -- Camera callback -------------------------------------------------------

static void* __fastcall irads_get_camera(void* indicator)
{
    void* camera = s_original_get_camera(indicator);
    if (!camera || !g_av_tv_sensor_instance)
        return camera;
    auto* sensor = as_sensor(g_av_tv_sensor_instance);
    const bool locked =
        s_irads_lock_valid && aim_pose_at_world_point(sensor->platform_world_pose.m, s_irads_lock_world);
    // The native camera normally extrapolates the pose from mount and motor
    // history between updates.
    // That overwrites our corrected pose. Suppress ONLY this render prediction
    // while building a locked camera, then restore the native update timestamp.
    const double saved_update_time = sensor->platform_last_update_time;
    if (locked)
        sensor->platform_last_update_time = (std::numeric_limits<double>::max)();
    if (s_sensor_fill_camera)
        s_sensor_fill_camera(g_av_tv_sensor_instance, camera);
    if (locked)
        sensor->platform_last_update_time = saved_update_time;
    // Snapshot the visible ray. Terrain tracing must use this optical eye,
    // not a native cached target or a newly recomputed aircraft-forward ray.
    if (s_irads_power)
    {
        float eye[3];
        float forward[3];
        std::memcpy(eye, static_cast<char*>(camera) + camera_eye_offset, sizeof(eye));
        std::memcpy(forward, static_cast<char*>(camera) + camera_forward_offset, sizeof(forward));
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
        float forward[3];
        float eye[3];
        std::memcpy(forward, static_cast<char*>(camera) + camera_forward_offset, sizeof(forward));
        std::memcpy(eye, static_cast<char*>(camera) + camera_eye_offset, sizeof(eye));
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
    if (!s_sensor_fill_camera)
    {
        log_message("IRADS camera bridge: native fill_camera unavailable");
        return false;
    }
    size_t slot = vtable_slot_count;
    for (size_t i = 0; i < vtable_slot_count; ++i)
        if (vtable[i] == reinterpret_cast<void*>(native_get_camera))
        {
            slot = i;
            break;
        }
    if (slot == vtable_slot_count)
    {
        log_message("IRADS camera bridge: get_camera slot not found; disabled");
        return false;
    }
    // Keep the type information entry ahead of the copied table for native RTTI.
    auto** copy = new void*[vtable_slot_count + 1];
    std::memcpy(copy, vtable - 1, sizeof(void*) * (vtable_slot_count + 1));
    s_original_get_camera = native_get_camera;
    copy[slot + 1] = reinterpret_cast<void*>(&irads_get_camera);
    *reinterpret_cast<void***>(indicator) = copy + 1;
    s_camera_bridge_indicator = indicator;
    log_format("IRADS camera bridge installed: indicator=%p slot=%zu sensor=%p", indicator, slot,
               g_av_tv_sensor_instance);
    return true;
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
        Cc_indicator_layout* indicator_view = as_indicator(s_our_ind);
        indicator_view->screen_condition = static_cast<uint32_t>(ir_screen_condition_flir);
        indicator_view->camera_condition = s_irads_power ? static_cast<uint32_t>(ir_camera_condition_on) : 0;
    }
}

void reset_camera_bridge()
{
    s_camera_bridge_indicator = nullptr;
    s_our_ind = nullptr;
}
}  // namespace Cockpit_interop
