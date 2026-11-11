#include "stdafx.h"
#include "F_117_NightHawk.h"

#include <algorithm>
#include <cmath>

#include "FlightModel/State.h"

// Leading-edge vortex and wing vapour. Shaped from two sources:
//
// The aircraft model. The planform is one straight 67.3 deg leading edge from
// the nose apex to the wingtip, at y = -0.28 m, so the local semispan is
// s(x) = 0.418 * (9.95 - x). The edge reaches the tip at x = -6.3 m (6.57 m
// out) and the tip trailing edge ends near x = -7.0 m. Upper-surface heights
// under the vortex (surface_90 / surface_80, at 80% and 90% of s) were sampled
// from the model.
//
// Vermeersch, "Investigation of the F117A Vortical Flow Characteristics"
// (MIT, 1993; docs/reference in the mod repo):
//   - the vortex starts at the nose apex and runs just inboard of the leading
//     edge all the way to the tip (Fig. 6.24);
//   - wind-tunnel smoke shows it clearly at 7 and 10 deg AoA, lying close to
//     the surface; it grows stronger and lifts away from 15 deg;
//   - breakdown sits just aft of the trailing edge at 15 deg and moves forward
//     with AoA: X/L = 0.775 at 20, 0.68 at 25, 0.555 at 30 deg (Fig. 6.25,
//     X measured from the nose, L the aircraft length);
//   - the breakdown point is unsteady; it is a bubble at 20 deg and a spiral
//     at 25 and 30 deg.
//
// Visible vapour also needs the pressure drop in the core to reach
// condensation, so opacity scales with dynamic pressure times AoA and fades
// with altitude.

namespace
{
// Geometry (body frame: x forward, y up, z right; metres).
constexpr float nose_x = 10.02F;           // nose tip
constexpr float length_m = 19.43F;         // reference length for breakdown X/L
constexpr float apex_x = 9.95F;            // leading edge line meets the centreline
constexpr float semispan_per_m = 0.418F;   // tan(90 - 67.3 deg)
constexpr float leading_edge_y = -0.28F;   // leading edge height
constexpr float tip_x = -6.30F;            // leading edge reaches the tip
constexpr float trailing_edge_x = -7.00F;  // tip trailing edge

constexpr int surface_points = 17;
constexpr float surface_x[surface_points] = {8.5F, 8.0F,  7.0F,  6.0F,  5.0F,  4.0F,  3.0F,  2.0F, 1.0F,
                                             0.0F, -1.0F, -2.0F, -3.0F, -4.0F, -5.0F, -6.0F, -6.3F};
constexpr float surface_90[surface_points] = {-0.22F, -0.18F, -0.16F, -0.10F, -0.04F, -0.02F, -0.17F, -0.15F, -0.14F,
                                              -0.12F, -0.11F, -0.09F, -0.10F, -0.12F, -0.15F, -0.17F, -0.18F};
constexpr float surface_80[surface_points] = {-0.22F, -0.09F, 0.01F,  0.09F,  0.17F,  0.22F,  0.11F,  -0.03F, 0.00F,
                                              0.03F,  0.03F,  -0.01F, -0.05F, -0.09F, -0.15F, -0.23F, -0.27F};

constexpr int path_points = 12;
constexpr float path_first_x = 8.5F;
constexpr float path_last_x = -9.0F;
constexpr float opacity_max = 0.20F;
constexpr float min_airspeed_mps = 35.0F;
constexpr float sea_level_density = 1.225F;
constexpr float degrees_to_radians_f = 0.0174533F;

// Clock shared by both sides: DCS calls once per side each frame, side 0 advances it.
float vortex_time = 0.0F;
LERX_vortex_spline_point vortex_buffers[2][path_points];

// Centreline path of one vortex before breakdown displacement.
struct VortexPath
{
    float x[path_points];
    float y[path_points];
    float z[path_points];
    float radius[path_points];
    float dist[path_points];
};

// Where along the path the vortex bursts, and how big it is there.
struct Breakdown
{
    float x;
    float distance;
    float radius;
    bool spiral;
};

float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float smoothstep(float e0, float e1, float v)
{
    const float t = clampf((v - e0) / (e1 - e0), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

// Upper-surface height at station x and fraction f of the local semispan.
float surface_y(float x, float f)
{
    if (x < trailing_edge_x)
    {
        return -1000.0F;  // behind the wing: nothing to clear
    }
    x = clampf(x, surface_x[surface_points - 1], surface_x[0]);
    int k = 0;
    while (k < surface_points - 2 && x < surface_x[k + 1])
    {
        ++k;
    }
    const float u = (surface_x[k] - x) / (surface_x[k] - surface_x[k + 1]);
    const float y90 = surface_90[k] + (surface_90[k + 1] - surface_90[k]) * u;
    const float y80 = surface_80[k] + (surface_80[k + 1] - surface_80[k]) * u;
    if (f >= 0.9F)
    {
        return y90 + (leading_edge_y - y90) * clampf((f - 0.9F) / 0.1F, 0.0F, 1.0F);
    }
    return y80 + (y90 - y80) * clampf((f - 0.8F) / 0.1F, 0.0F, 1.0F);
}

// Altitude-dependent condensation fade. The temperature/dew-point spread widens with
// altitude, so condensation gets harder. Ambient density stands in for both pressure
// and temperature; altitude gives a hard ceiling for non-standard days.
//   Sea level ~1.0, 10,000 ft ~0.85, 20,000 ft ~0.40, 30,000 ft ~0.10
float humidity_factor_for(float rho, float alt_m)
{
    const float rho_ratio = rho / sea_level_density;
    float humidity = powf(rho_ratio, 3.5F);
    if (rho_ratio > 0.95F)  // low-altitude moisture boost, below ~500 m
    {
        humidity += (rho_ratio - 0.95F) / 0.05F * 0.08F;
    }
    if (alt_m > 8000.0F)
    {
        humidity *= std::max(0.0F, 1.0F - (alt_m - 8000.0F) / 2000.0F);
    }
    return clampf(humidity, 0.0F, 1.0F);
}

// Overall vapour opacity for one side.
float vapour_opacity(float vortex_strength, float aoa, float vel, float rho, float alt_m, float side_bias)
{
    const float humidity_factor = humidity_factor_for(rho, alt_m);

    // Core suction scales with q * CL, and CL with AoA. A slow approach at
    // 10 deg shows little vapour; a fast, loaded turn shows a lot.
    const float q = 0.5F * rho * vel * vel;
    const float suction = q * clampf(aoa, 0.0F, 25.0F) / 12.0F;
    const float condensation = 0.25F + 0.75F * smoothstep(3000.0F, 12000.0F, suction);

    // Sideslip: the windward vortex is stronger and breaks down earlier.
    const float asym = clampf(1.0F + side_bias * 0.12F, 0.15F, 1.85F);

    // Photographs of high-g passes show one condensation cloud over the aft
    // body and no separate vortex tubes. The mod's overwing vapour effect
    // draws that cloud from about 8 deg AoA and at full strength from 11.5 deg
    // (OpacityOverPower in Effects/f-117-nighthawk_overwingVapor.lua), so the
    // vortices fade out over the same range.
    const float cloud_takeover = smoothstep(8.0F, 11.5F, aoa);

    return opacity_max * vortex_strength * condensation * humidity_factor * asym * (1.0F - cloud_takeover);
}

// Conical scaling with local semispan. At low AoA the core hugs the surface close to
// the edge; with AoA it moves inboard, grows and lifts. Path runs from the apex region
// to a short wake behind the tip.
VortexPath build_path(float aoa)
{
    const float a = clampf(aoa, 0.0F, 35.0F);
    const float f_core = 0.90F - 0.003F * a;                          // 0.87 s at 10 deg, 0.81 s at 30
    const float radius_k = 0.045F + 0.0025F * a;                      // vapour radius / s
    const float gap_k = 0.0025F * a;                                  // core clearance / s
    const float trail_slope = 0.5F * tanf(a * degrees_to_radians_f);  // wake rises toward the freestream

    VortexPath path{};
    for (int i = 0; i < path_points; ++i)
    {
        const float x =
            path_first_x + (path_last_x - path_first_x) * static_cast<float>(i) / static_cast<float>(path_points - 1);
        const float xs = std::max(x, tip_x);
        const float s = semispan_per_m * (apex_x - xs);
        const float r = radius_k * s;
        const float surf = std::max(surface_y(x, f_core), surface_y(x, f_core - radius_k));
        float y = surf + r + gap_k * s;
        if (x < tip_x)
        {
            const float y_tip = std::max(surface_y(tip_x, f_core), surface_y(tip_x, f_core - radius_k)) + r + gap_k * s;
            y = y_tip + (tip_x - x) * trail_slope;
        }
        path.x[i] = x;
        path.y[i] = y;
        path.z[i] = f_core * s;
        path.radius[i] = r;
        if (i == 0)
        {
            path.dist[i] = 0.0F;
        }
        else
        {
            path.dist[i] = path.dist[i - 1] + sqrtf((path.x[i] - path.x[i - 1]) * (path.x[i] - path.x[i - 1]) +
                                                    (path.y[i] - path.y[i - 1]) * (path.y[i] - path.y[i - 1]) +
                                                    (path.z[i] - path.z[i - 1]) * (path.z[i] - path.z[i - 1]));
        }
    }
    return path;
}

// Breakdown: X/L through Vermeersch Fig. 6.25 (20, 25, 30 deg) and the text's
// "just aft of the trailing edge" at 15 deg, extended linearly.
Breakdown find_breakdown(const VortexPath& path, float aoa, float side_bias, float time, unsigned idx)
{
    float burst_xl;
    if (aoa < 20.0F)
    {
        burst_xl = 0.775F - 0.022F * (aoa - 20.0F);
    }
    else if (aoa < 25.0F)
    {
        burst_xl = 0.775F - 0.019F * (aoa - 20.0F);
    }
    else
    {
        burst_xl = 0.680F - 0.025F * (aoa - 25.0F);
    }
    const float side = static_cast<float>(idx);
    burst_xl -= 0.006F * side_bias;
    burst_xl += 0.012F * sinf(time * 1.7F + side * 2.1F) + 0.006F * sinf(time * 4.3F + side * 0.7F);
    burst_xl = clampf(burst_xl, 0.40F, 1.40F);

    Breakdown burst{};
    burst.x = nose_x - burst_xl * length_m;
    burst.spiral = aoa > 22.5F;  // bubble at 20 deg, spiral at 25 and 30

    // Distance along the spline to the breakdown point.
    const int last = path_points - 1;
    if (burst.x >= path.x[0])
    {
        burst.distance = 0.0F;
        burst.radius = path.radius[0];
    }
    else if (burst.x <= path.x[last])
    {
        burst.distance = path.dist[last] + (path.x[last] - burst.x);
        burst.radius = path.radius[last];
    }
    else
    {
        int k = 0;
        while (path.x[k + 1] > burst.x)
        {
            ++k;
        }
        const float u = (path.x[k] - burst.x) / (path.x[k] - path.x[k + 1]);
        burst.distance = path.dist[k] + (path.dist[k + 1] - path.dist[k]) * u;
        burst.radius = path.radius[k] + (path.radius[k + 1] - path.radius[k]) * u;
    }
    return burst;
}

// Displaces the path into spline points: precession before breakdown, a spiral or a
// pulsing bubble after it, plus per-point opacity and the undisturbed tangent.
void fill_spline(LERX_vortex_spline_point* sp, const VortexPath& path, const Breakdown& burst, float time, unsigned idx)
{
    const float zs = (idx == 0) ? -1.0F : 1.0F;
    const int last = path_points - 1;

    for (int i = 0; i < path_points; ++i)
    {
        const float fi = static_cast<float>(i);
        const float t = fi / static_cast<float>(last);
        const float excess = std::max(0.0F, path.dist[i] - burst.distance);
        const float post = clampf(excess / 0.8F, 0.0F, 1.0F);  // blend over 0.8 m

        // Radius: conical before breakdown, rapid wake-type growth after.
        float radius = path.radius[i];
        float y = path.y[i];
        if (excess > 0.0F)
        {
            radius = std::min(1.8F, burst.radius + 0.35F * excess);
            y += 0.12F * excess;
        }

        // Motion: slight core precession before breakdown; after it, a
        // swirling spiral or a pulsing bubble.
        const float pre_amp = 0.04F * path.radius[i];
        float jy = (1.0F - post) * pre_amp * (sinf(time * 6.0F + fi * 1.9F) + 0.5F * sinf(time * 11.0F + fi * 3.1F));
        float jz = (1.0F - post) * pre_amp * 0.5F * sinf(time * 7.0F + fi * 2.3F);
        if (post > 0.0F)
        {
            const float amp = post * radius * (burst.spiral ? 0.30F : 0.12F);
            const float phase = time * (burst.spiral ? 9.0F : 5.0F) - excess * 2.5F + fi * 0.9F;
            jy += amp * sinf(phase) + 0.3F * amp * sinf(time * 23.0F + fi * 4.1F);
            jz += amp * cosf(phase) + 0.3F * amp * cosf(time * 17.0F + fi * 5.7F);
        }

        // Opacity shape, 0..1, scaled by out.opacity.
        //   - fades in aft of the nose as suction builds (photos show vapour
        //     from about the inlet back);
        //   - off the wing the pressure recovers and unburst vapour clears;
        //   - burst vapour turns to a thin haze.
        float shape = smoothstep(8.0F, 4.5F, path.x[i]);
        if (path.x[i] < trailing_edge_x && excess <= 0.0F)
        {
            shape *= expf(-(trailing_edge_x - path.x[i]) / 1.5F);
        }
        shape *= 1.0F - post * 0.75F * (1.0F - expf(-excess / 2.0F));

        const float noise1 = sinf(t * 4.5F + time * 1.3F) * sinf(t * 7.2F - time * 0.9F);
        const float noise2 = sinf(t * 13.0F + time * 3.7F + fi * 0.8F) * cosf(t * 9.1F + time * 2.1F);
        const float noise3 = sinf(t * 29.0F + time * 11.0F + fi * 2.4F);
        const float density = clampf(0.80F + 0.12F * noise1 + 0.06F * noise2 + 0.02F * noise3 +
                                         post * 0.15F * sinf(time * 19.0F + fi * 3.3F),
                                     0.35F, 1.0F);

        sp[i].radius = radius * (1.0F + 0.10F * noise1 + 0.05F * noise2);
        sp[i].opacity = clampf(shape * density, 0.0F, 1.0F);
        sp[i].pos[0] = path.x[i];
        sp[i].pos[1] = y + jy;
        sp[i].pos[2] = (path.z[i] + jz) * zs;

        // Unit tangent along the undisturbed path.
        const int i0 = (i == 0) ? 0 : i - 1;
        const int i1 = (i == last) ? last : i + 1;
        const float dx = path.x[i1] - path.x[i0];
        const float dy = path.y[i1] - path.y[i0];
        const float dz = path.z[i1] - path.z[i0];
        float mag = sqrtf(dx * dx + dy * dy + dz * dz);
        if (mag < 1e-6F)
        {
            mag = 1.0F;
        }
        sp[i].vel[0] = dx / mag;
        sp[i].vel[1] = dy / mag;
        sp[i].vel[2] = dz / mag * zs;
    }
}

bool detach(LERX_vortex& out)
{
    out.spline = nullptr;
    out.spline_points_count = 0;
    return true;
}
}  // namespace

bool ed_fm_LERX_vortex_update(unsigned idx, LERX_vortex& out)
{
    if (idx > 1)
    {
        return false;
    }

    out.version = 1;  // per-point opacity, scaled by out.opacity
    out.spline_point_size_in_bytes = sizeof(LERX_vortex_spline_point);

    // DCS calls this once per side each frame; advance the shared clock once.
    if (idx == 0)
    {
        vortex_time += static_cast<float>(F117::DeltaTime);
    }
    const float time = vortex_time;

    if (F117::weight_on_wheels || F117::totalVelocity_MPS < min_airspeed_mps)
    {
        return detach(out);
    }

    const float aoa = static_cast<float>(F117::alpha_DEG);
    const float beta = static_cast<float>(F117::beta_DEG);
    const float vel = static_cast<float>(F117::totalVelocity_MPS);
    float rho = static_cast<float>(F117::ambientDensity_KgPerM3);
    if (rho < 0.1F)
    {
        rho = sea_level_density;
    }

    // Vortex forms from about 5 deg and is well established by 8-10 deg.
    const float vortex_strength = 1.0F / (1.0F + expf(-(aoa - 6.0F) * 0.9F));
    if (vortex_strength < 0.02F)
    {
        return detach(out);
    }

    const float side_bias = (idx == 1) ? beta : -beta;
    out.opacity = vapour_opacity(vortex_strength, aoa, vel, rho, static_cast<float>(F117::altitude_m), side_bias);
    if (out.opacity < 0.004F)
    {
        return detach(out);
    }

    const VortexPath path = build_path(aoa);
    const Breakdown burst = find_breakdown(path, aoa, side_bias, time, idx);
    out.explosion_start = std::max(0.5F, burst.distance);

    LERX_vortex_spline_point* sp = vortex_buffers[idx];
    fill_spline(sp, path, burst, time, idx);

    out.spline = sp;
    out.spline_points_count = path_points;
    return true;
}
