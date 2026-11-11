// Bridges the DLIR indicator to the injected avTVSensor: finds the camera render-target indicator,
// keeps it in FLIR mode, and replaces its get_camera so the render uses our sensor's pose.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace CockpitInterop
{
static constexpr int kMaxContextDepth = 8;
static constexpr size_t kVtableSlotCount = 128;

// Native camera object layout: eye position and forward vector (float triples).
static constexpr size_t kCameraEyeOffset = 0x260;
static constexpr size_t kCameraForwardOffset = 0x230;

// Render-aim diagnostics: log the first frame, then every kRenderLogInterval-th, up to a cap.
static constexpr unsigned kRenderLogLimit = 120;
static constexpr unsigned kRenderLogInterval = 120;

using IndicatorGetCamera = void*(__fastcall*)(void*);
static IndicatorGetCamera s_original_get_camera = nullptr;
using SensorFillCamera = void(__fastcall*)(void*, void*);
static SensorFillCamera s_sensor_fill_camera = nullptr;
static void* s_camera_bridge_indicator = nullptr;

// -- Indicator search ------------------------------------------------------

// The DLIR camera render target is the indicator of type 0 with render target 0.
static void* camera_indicator_in(const CcCockpitContextLayout* context)
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
                const CcIndicatorLayout* view = AsIndicator(indicator);
                if (view->indicatorType == 0 && static_cast<int32_t>(view->renderTargetId) == 0)
                    return indicator;
            }
        }
    }
    return nullptr;
}

// Depth-first, pre-order walk of the context tree, bounded by kMaxContextDepth.
static void* find_camera_indicator(CcCockpitContextLayout* root, int* out_depth)
{
    struct Frame
    {
        void** next;
        void** end;
    };
    Frame stack[kMaxContextDepth + 1] = {};
    int depth = -1;
    CcCockpitContextLayout* pending = root;
    for (;;)
    {
        if (pending)
        {
            const int pendingDepth = depth + 1;
            if (pendingDepth <= kMaxContextDepth)
            {
                void* found = camera_indicator_in(pending);
                if (found)
                {
                    *out_depth = pendingDepth;
                    return found;
                }
                void** begin = pending->childContexts.begin;
                void** end = pending->childContexts.end;
                const bool hasChildren = begin && end && end > begin;
                depth = pendingDepth;
                stack[depth] = {hasChildren ? begin : nullptr, hasChildren ? end : nullptr};
            }
            pending = nullptr;
        }
        while (depth >= 0 && stack[depth].next == stack[depth].end)
            --depth;
        if (depth < 0)
            return nullptr;
        pending = AsContext(*stack[depth].next);
        ++stack[depth].next;
    }
}

static void locate_camera_indicator()
{
    static bool s_indicator_search_logged = false;
    int foundDepth = -1;
    CcCockpitContextLayout* globalCtx = GetGlobalContext();
    CcCockpitContextLayout* bindingCtx = GetBindingContext();

    s_our_ind = find_camera_indicator(globalCtx, &foundDepth);
    if (s_our_ind)
    {
        Log("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via global ctx", s_our_ind,
            foundDepth);
        return;
    }
    if (bindingCtx != globalCtx)
    {
        s_our_ind = find_camera_indicator(bindingCtx, &foundDepth);
        if (s_our_ind)
        {
            Log("ForceIRADSIndicatorActive: found camera render target @ %p depth=%d via binding ctx", s_our_ind,
                foundDepth);
            return;
        }
    }
    if (!s_indicator_search_logged)
    {
        Log("ForceIRADSIndicatorActive: no indicator found yet (global=%p binding=%p)", static_cast<void*>(globalCtx),
            static_cast<void*>(bindingCtx));
        s_indicator_search_logged = true;
    }
}

// -- Camera callback -------------------------------------------------------

static void* __fastcall IRADSGetCamera(void* indicator)
{
    void* camera = s_original_get_camera(indicator);
    if (!camera || !g_avTVSensor_instance)
        return camera;
    auto* sensor = AsSensor(g_avTVSensor_instance);
    const bool locked = s_irads_lock_valid && AimPoseAtWorldPoint(sensor->platformWorldPose.m, s_irads_lock_world);
    // The native camera normally extrapolates the pose from mount and motor
    // history between updates.
    // That overwrites our corrected pose. Suppress ONLY this render prediction
    // while building a locked camera, then restore the native update timestamp.
    const double savedUpdateTime = sensor->platformLastUpdateTime;
    if (locked)
        sensor->platformLastUpdateTime = (std::numeric_limits<double>::max)();
    if (s_sensor_fill_camera)
        s_sensor_fill_camera(g_avTVSensor_instance, camera);
    if (locked)
        sensor->platformLastUpdateTime = savedUpdateTime;
    // Snapshot the visible ray. Terrain tracing must use this optical eye,
    // not a native cached target or a newly recomputed aircraft-forward ray.
    if (s_irads_power)
    {
        float eye[3], forward[3];
        std::memcpy(eye, static_cast<char*>(camera) + kCameraEyeOffset, sizeof(eye));
        std::memcpy(forward, static_cast<char*>(camera) + kCameraForwardOffset, sizeof(forward));
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
    static unsigned renderSamples = 0, renderFrames = 0;
    if (locked && renderSamples < kRenderLogLimit && (++renderFrames == 1 || renderFrames % kRenderLogInterval == 0))
    {
        // Observe the FINAL rendered ray, not just the cached pose.
        float forward[3], eye[3];
        std::memcpy(forward, static_cast<char*>(camera) + kCameraForwardOffset, sizeof(forward));
        std::memcpy(eye, static_cast<char*>(camera) + kCameraEyeOffset, sizeof(eye));
        double dot = 0, rangeSq = 0, axisSq = 0;
        for (int i = 0; i < 3; ++i)
        {
            const double d = s_irads_lock_world[i] - eye[i];
            dot += d * forward[i];
            rangeSq += d * d;
            axisSq += forward[i] * forward[i];
        }
        if (rangeSq > 1 && axisSq > 0.5)
        {
            double cosine = dot / std::sqrt(rangeSq * axisSq);
            cosine = cosine > 1 ? 1 : (cosine < -1 ? -1 : cosine);
            Log("IRADS render aim: errorDeg=%.6f range=%.1f eye=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f)",
                std::acos(cosine) * kRadToDeg, std::sqrt(rangeSq), eye[0], eye[1], eye[2], s_irads_lock_world[0],
                s_irads_lock_world[1], s_irads_lock_world[2]);
            ++renderSamples;
        }
    }
    return camera;
}

static bool InstallIRADSCameraBridge(void* indicator)
{
    if (s_camera_bridge_indicator == indicator)
        return true;
    auto nativeGetCamera = reinterpret_cast<IndicatorGetCamera>(
        ResolveCockpitBaseExport("?get_camera@ccIndicator@cockpit@@UEAAAEBVCamera@Graphics@@XZ"));
    auto** vtable = *reinterpret_cast<void***>(indicator);
    if (!nativeGetCamera || !vtable)
        return false;
    s_sensor_fill_camera = reinterpret_cast<SensorFillCamera>(
        ResolveCockpitBaseExport("?fill_camera@avTVSensor@cockpit@@UEBAXAEAVCamera@Graphics@@@Z"));
    if (!s_sensor_fill_camera)
    {
        Log_("IRADS camera bridge: native fill_camera unavailable");
        return false;
    }
    size_t slot = kVtableSlotCount;
    for (size_t i = 0; i < kVtableSlotCount; ++i)
        if (vtable[i] == reinterpret_cast<void*>(nativeGetCamera))
        {
            slot = i;
            break;
        }
    if (slot == kVtableSlotCount)
    {
        Log_("IRADS camera bridge: get_camera slot not found; disabled");
        return false;
    }
    // Keep the type information entry ahead of the copied table for native RTTI.
    auto** copy = new void*[kVtableSlotCount + 1];
    std::memcpy(copy, vtable - 1, sizeof(void*) * (kVtableSlotCount + 1));
    s_original_get_camera = nativeGetCamera;
    copy[slot + 1] = reinterpret_cast<void*>(&IRADSGetCamera);
    *reinterpret_cast<void***>(indicator) = copy + 1;
    s_camera_bridge_indicator = indicator;
    Log("IRADS camera bridge installed: indicator=%p slot=%zu sensor=%p", indicator, slot, g_avTVSensor_instance);
    return true;
}

// Phase C: called every frame from ed_fm_simulate. Once indicators are populated (after
// device_init.lua) it locates the camera render-target indicator, installs the camera bridge,
// and every frame forces screenCondition = FLIR (11) and cameraCondition = ON (1) so DCS can't
// revert the indicator to a non-camera state.
void ForceIRADSIndicatorActive()
{
    if (!g_avTVSensor_instance)
        return;

    if (!s_our_ind)
        locate_camera_indicator();

    if (s_our_ind)
    {
        if (!InstallIRADSCameraBridge(s_our_ind))
            return;
        CcIndicatorLayout* indicatorView = AsIndicator(s_our_ind);
        indicatorView->screenCondition = static_cast<uint32_t>(IR_SCREEN_CONDITION_FLIR);
        indicatorView->cameraCondition = s_irads_power ? static_cast<uint32_t>(IR_CAMERA_CONDITION_ON) : 0;
    }
}

void ResetCameraBridge()
{
    s_camera_bridge_indicator = nullptr;
    s_our_ind = nullptr;
}
}  // namespace CockpitInterop
