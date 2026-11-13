#pragma once

// Time of flight for laser-guided bombs, worked out once at release and then counted down, as the stock CCRP
// displays do. A point mass falls from the release state to the target height under gravity and the store's DCS
// drag curve (fm.cx_coeff). The drag is scaled so that a fall from 2000 m at 40 m/s in the standard atmosphere
// takes the store's characteristic time (targeting_data.char_time). Guidance makes the real flight longer: a
// per-store share of the fall time is added, and a release beyond the ballistic range adds the extra distance at a
// per-store glide speed. Both are fitted to drops measured in DCS.

#include <algorithm>
#include <cmath>
#include <cstring>

namespace Weapons
{
namespace Lgb_ballistics
{
struct Store
{
    const char* name;  // CCRP profile name
    double mass_kg;
    double caliber_m;
    double cx_coeff[5];  // k0, k1, m1, cx_sub, m2
    double char_time_s;
    double guided_time_share;  // added share of the ballistic fall time
    double glide_speed_mps;    // speed for the distance released beyond the ballistic range; 0 adds nothing
};

// Stock DCS weapon data: mass, caliber, fm.cx_coeff and targeting_data.char_time. The guidance terms are fitted to
// level drops 1280-7300 m above the target at 160-260 m/s, released from 300 m short of to 2.1 km beyond the
// ballistic range (tools/missions/store_drop_test.py in the mod repo): within 1.7 s of every measured flight.
constexpr Store stores[] = {
    {"GBU-12", 277.0, 0.279, {1.037, 0.74, 0.6, 0.382, 1.34}, 20.38, 0.0694, 299.0},
    {"GBU-27", 991.0, 0.457, {1.037, 0.91, 0.9, 0.382, 1.34}, 20.33, 0.3336, 0.0},
    {"GBU-10", 959.0, 0.457, {1.037, 0.91, 0.6, 0.382, 1.34}, 20.34, 0.1749, 540.0},
};
constexpr int store_count = static_cast<int>(sizeof(stores) / sizeof(stores[0]));

inline int find_store(const char* name)
{
    for (int i = 0; name != nullptr && i < store_count; ++i)
        if (std::strcmp(stores[i].name, name) == 0)
            return i;
    return -1;
}

// Drag coefficient against Mach: cx_sub up to Mach m1, k1 / sqrt(M^2 - 1) from Mach m2, and between them a cubic
// that leaves cx_sub flat and meets the supersonic curve with the same value and slope (slope scaled by k0).
inline double drag_coefficient(const double k[5], double mach)
{
    const double k0 = k[0], k1 = k[1], m1 = k[2], cx_sub = k[3], m2 = k[4];
    if (mach <= m1)
        return cx_sub;
    if (mach >= m2)
        return k1 / std::sqrt(mach * mach - 1.0);
    const double root = std::sqrt(m2 * m2 - 1.0);
    const double f2 = k1 / root;
    const double s2 = -k1 * m2 / (root * root * root) * k0;
    const double h = m2 - m1, t = (mach - m1) / h, t2 = t * t, t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * cx_sub + (-2.0 * t3 + 3.0 * t2) * f2 + (t3 - t2) * h * s2;
}

// Air that cools at 6.5 K per km from a reference height, with density following the standard-atmosphere law.
struct Atmosphere
{
    double reference_alt_m;
    double temperature_k;
    double density_kg_m3;
};
constexpr Atmosphere standard_atmosphere{0.0, 288.15, 1.225};

inline void air_at(const Atmosphere& air, double alt_m, double* density, double* sound_speed)
{
    const double t = std::max(150.0, air.temperature_k - 0.0065 * (alt_m - air.reference_alt_m));
    *density = air.density_kg_m3 * std::pow(t / air.temperature_k, 4.25588);
    *sound_speed = std::sqrt(1.4 * 287.05 * t);
}

constexpr double gravity_mps2 = 9.81;
constexpr double step_seconds = 0.02;
constexpr double max_seconds = 180.0;

// Seconds to fall from release_alt_m to target_alt_m. Speeds are relative to the air (vertical positive up);
// a steady wind carries the bomb but does not change its fall time. Returns -1 when it does not get there.
// `distance` (optional) receives the horizontal distance covered through the air.
inline double fall_time(const Store& store, double drag_scale, const Atmosphere& air, double release_alt_m,
                        double target_alt_m, double horizontal_speed, double vertical_speed,
                        double* distance = nullptr)
{
    if (!(release_alt_m > target_alt_m) || store.mass_kg <= 0.0)
        return -1.0;
    const double area_per_mass = 0.25 * 3.14159265358979 * store.caliber_m * store.caliber_m / store.mass_kg;
    auto accel = [&](double y, double vx, double vy, double* ax, double* ay) {
        double rho = 0.0;
        double sound = 1.0;
        air_at(air, y, &rho, &sound);
        const double v = std::sqrt(vx * vx + vy * vy);
        const double d = v > 0.01 ? 0.5 * rho * v * drag_scale * drag_coefficient(store.cx_coeff, v / sound) *
                                        area_per_mass
                                  : 0.0;
        *ax = -d * vx;
        *ay = -gravity_mps2 - d * vy;
    };
    double x = 0.0;
    double y = release_alt_m;
    double vx = horizontal_speed;
    double vy = vertical_speed;
    double t = 0.0;
    const double h = step_seconds;
    while (t < max_seconds)
    {
        double ax1 = 0.0;
        double ay1 = 0.0;
        double ax2 = 0.0;
        double ay2 = 0.0;
        double ax3 = 0.0;
        double ay3 = 0.0;
        double ax4 = 0.0;
        double ay4 = 0.0;
        accel(y, vx, vy, &ax1, &ay1);
        accel(y + 0.5 * h * vy, vx + 0.5 * h * ax1, vy + 0.5 * h * ay1, &ax2, &ay2);
        accel(y + 0.5 * h * (vy + 0.5 * h * ay1), vx + 0.5 * h * ax2, vy + 0.5 * h * ay2, &ax3, &ay3);
        accel(y + h * (vy + 0.5 * h * ay2), vx + h * ax3, vy + h * ay3, &ax4, &ay4);
        // RK4 position update for a velocity-dependent force
        const double x_next = x + h * vx + h * h / 6.0 * (ax1 + ax2 + ax3);
        const double y_next = y + h * vy + h * h / 6.0 * (ay1 + ay2 + ay3);
        vx += h / 6.0 * (ax1 + 2.0 * ax2 + 2.0 * ax3 + ax4);
        vy += h / 6.0 * (ay1 + 2.0 * ay2 + 2.0 * ay3 + ay4);
        if (y_next <= target_alt_m)
        {
            const double fraction = (y - target_alt_m) / (y - y_next);
            if (distance != nullptr)
                *distance = x + fraction * (x_next - x);
            return t + h * fraction;
        }
        x = x_next;
        y = y_next;
        t += h;
    }
    return -1.0;
}

// Drag scale that makes a fall from 2000 m at 40 m/s in the standard atmosphere take the store's char_time.
inline double calibrate_drag_scale(const Store& store)
{
    double low = 0.0;
    double high = 4.0;
    for (int i = 0; i < 40; ++i)
    {
        const double mid = 0.5 * (low + high);
        if (fall_time(store, mid, standard_atmosphere, 2000.0, 0.0, 40.0, 0.0) < store.char_time_s)
            low = mid;
        else
            high = mid;
    }
    return 0.5 * (low + high);
}

// Flight time for store `index` (see find_store), or -1. The drag calibration is worked out on first use.
// target_range_m is the ground distance to the target at release and tailwind_mps the wind along the track. The
// ballistic fall is lengthened by the store's guided share, and a release beyond the ballistic range adds the extra
// distance at the store's glide speed.
inline double time_of_flight(int index, const Atmosphere& air, double release_alt_m, double target_alt_m,
                             double horizontal_speed, double vertical_speed, double target_range_m,
                             double tailwind_mps)
{
    if (index < 0 || index >= store_count)
        return -1.0;
    static double drag_scale[store_count] = {};
    if (drag_scale[index] <= 0.0)
        drag_scale[index] = calibrate_drag_scale(stores[index]);
    double distance = 0.0;
    const double t = fall_time(stores[index], drag_scale[index], air, release_alt_m, target_alt_m,
                               horizontal_speed, vertical_speed, &distance);
    if (t < 0.0)
        return t;
    const Store& store = stores[index];
    const double beyond = target_range_m - (distance + tailwind_mps * t);
    const double glide = beyond > 0.0 && store.glide_speed_mps > 0.0 ? beyond / store.glide_speed_mps : 0.0;
    return t * (1.0 + store.guided_time_share) + glide;
}
}  // namespace Lgb_ballistics
}  // namespace Weapons
