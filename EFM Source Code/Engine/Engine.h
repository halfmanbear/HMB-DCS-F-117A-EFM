#pragma once
#include "../EfmMath.h"
#include <algorithm>
#include <cmath>

namespace F117
{
namespace Engine
{
// One engine's state. It persists across FM ticks and is reset by the startup hooks.
struct Engine_state
{
    double n2 = 0.0;  // core rpm, %
    double thrust_fraction_of_max = 0.0;
};
// The thrust constants and table below are for both engines together.
constexpr double engine_count = 2.0;

constexpr double throttle_min = 0.0;
constexpr double throttle_max = 100.0;
constexpr double mach_min = 0.0;
constexpr double mach_max = 0.92;
constexpr double altitude_min_m = 0.0;
constexpr double altitude_max_m = 13716.0;  // 45000 ft
constexpr double ground_idle_n2 = 60.0;
constexpr double flight_idle_n2 = 65.0;
constexpr double max_n2 = 100.0;
constexpr double spool_up_rate = 0.65;
constexpr double spool_down_rate = 0.45;
constexpr double windmill_decay = 0.10;
constexpr double shutdown_n2_threshold = 0.5;
constexpr double ground_idle_thrust_n = 3000.0;
// Net thrust at flight idle, fitted to the idle deceleration in TO 1F-117A-1 Figure A9-11. That data stops at 30,000
// ft.
constexpr double flight_idle_thrust_sea_level_n = 935.3;
constexpr double flight_idle_thrust_per_10_km_n = 4777.0;
constexpr double flight_idle_thrust_altitude_cap_m = 9144.0;
constexpr double max_thrust_limit_n = 94000.0;  // Two F404s at about 47 kN each.
// Installed maximum thrust of both engines, fitted by tools/fm_bench/fit_performance.py in the mod repository
// to the acceleration, climb, ceiling, top speed and sustained turn data in TO 1F-117A-1.
constexpr int thrust_table_altitudes = 5;
constexpr int thrust_table_machs = 5;
constexpr double thrust_table_altitude_m[thrust_table_altitudes] = {0.0, 3048.0, 6096.0, 9144.0, 12192.0};
constexpr double thrust_table_mach[thrust_table_machs] = {0.30, 0.50, 0.70, 0.80, 0.90};
constexpr double thrust_table_n[thrust_table_altitudes][thrust_table_machs] = {
    {80268.0, 82454.0, 82689.0, 81440.0, 80000.0},
    {59398.0, 60031.0, 62792.0, 62681.0, 62365.0},
    {39344.0, 39434.0, 43246.0, 44643.0, 45700.0},
    {25533.0, 25580.0, 27304.0, 28407.0, 29522.0},
    {16563.0, 16576.0, 16845.0, 17096.0, 17504.0}};
// Below the table's lowest Mach, thrust rises towards this much extra at a standstill, up to the engines' rating.
// The manual's in-flight data doesn't reach those speeds, so this comes from the takeoff example on TO 1F-117A-1
// page A3-4 (4,700 ft ground run at 46,000 lb).
constexpr double static_thrust_rise = 0.18;
constexpr double min_thrust_idle_margin = 1.1;
constexpr double idle_thrust_cutoff_factor = 0.9;

// Bilinear in altitude and Mach. Above the top row it carries on along the same slope, up to ALTITUDE_MAX_M.
inline double max_thrust_from_table(double altitude, double mach)
{
    int i = 0;
    while (i < thrust_table_altitudes - 2 && altitude >= thrust_table_altitude_m[i + 1])
        ++i;
    int j = 0;
    while (j < thrust_table_machs - 2 && mach >= thrust_table_mach[j + 1])
        ++j;
    const double a =
        (altitude - thrust_table_altitude_m[i]) / (thrust_table_altitude_m[i + 1] - thrust_table_altitude_m[i]);
    const double m = limit(mach, thrust_table_mach[0], thrust_table_mach[thrust_table_machs - 1]);
    const double b = (m - thrust_table_mach[j]) / (thrust_table_mach[j + 1] - thrust_table_mach[j]);
    return (1.0 - a) * ((1.0 - b) * thrust_table_n[i][j] + b * thrust_table_n[i][j + 1]) +
           a * ((1.0 - b) * thrust_table_n[i + 1][j] + b * thrust_table_n[i + 1][j + 1]);
}

// Advances one engine and returns its thrust (N).
inline double engine_dynamics(Engine_state& engine, double throttle_input, double mach, double alt, double frame_time,
                              bool engine_running = true, bool weight_on_wheels = false)
{
    // Clamp the incoming conditions to the model's intended operating range.
    double throttle = limit(throttle_input, throttle_min, throttle_max);
    double mach_limited = limit(mach, mach_min, mach_max);
    double altitude = limit(alt, altitude_min_m, altitude_max_m);

    // Ground idle remains lower than flight idle.
    double idle_n2 = weight_on_wheels ? ground_idle_n2 : flight_idle_n2;

    if (engine_running)
    {
        // FADEC-style schedule: throttle commands a target N2.
        double target_n2 = idle_n2 + (throttle / throttle_max) * (max_n2 - idle_n2);

        double rate = (target_n2 > engine.n2) ? spool_up_rate : spool_down_rate;

        engine.n2 += (target_n2 - engine.n2) * rate * frame_time;
        engine.n2 = limit(engine.n2, 0.0, max_n2);
    }
    else
    {
        // Shutdown or flameout: allow a gradual windmill decay.
        engine.n2 += (0.0 - engine.n2) * windmill_decay * frame_time;
        if (engine.n2 < shutdown_n2_threshold)
        {
            engine.n2 = 0.0;
        }
    }

    // Below this threshold the engine is treated as producing no useful thrust.
    if (engine.n2 < idle_n2 * idle_thrust_cutoff_factor)
    {
        engine.thrust_fraction_of_max = 0.0;
        return 0.0;
    }

    // Ground idle thrust is held lower than airborne idle so the aircraft
    // does not start creeping on the runway at zero throttle.
    double tidle =
        weight_on_wheels
            ? ground_idle_thrust_n * max_thrust_from_table(altitude, thrust_table_mach[0]) / thrust_table_n[0][0]
            : flight_idle_thrust_sea_level_n +
                  flight_idle_thrust_per_10_km_n * (std::min)(altitude, flight_idle_thrust_altitude_cap_m) / 10000.0;
    double tmax = max_thrust_from_table(altitude, mach_limited);
    if (mach_limited < thrust_table_mach[0])
    {
        tmax *= 1.0 + static_thrust_rise * (1.0 - mach_limited / thrust_table_mach[0]);
    }

    if (tmax < tidle)
    {
        tmax = tidle * min_thrust_idle_margin;
    }

    // Normalize N2 above idle before shaping the thrust response.
    double norm_n2 = (engine.n2 - idle_n2) / (max_n2 - idle_n2);

    norm_n2 = limit(norm_n2, 0.0, 1.0);

    // Preserve the existing non-linear thrust curve.
    double thrust_fraction = pow(norm_n2, 2.5);

    double thrust = tidle + thrust_fraction * (tmax - tidle);

    thrust = limit(thrust, 0.0, max_thrust_limit_n);
    engine.thrust_fraction_of_max = tmax > 0.0 ? limit(thrust / tmax, 0.0, 1.0) : 0.0;

    return thrust / engine_count;
}
}  // namespace Engine
}  // namespace F117
