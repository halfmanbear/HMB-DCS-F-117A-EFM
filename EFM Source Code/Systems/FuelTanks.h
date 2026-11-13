#pragma once
// Internal fuel tanks: where the fuel is for any total quantity, so DCS gets the right mass, CG and inertia.
//
// Capacities and usage order: TO 1F-117A-1 pages 1-10 and 1-11, Figure 1-7 (18,200 lb with wing fuel).
// Positions: the manual places the fuselage fuel above and aft of the weapons bay, left and right of centre,
// forward transfer, then feed, then aft transfer from front to back, and the wing fuel between the beams
// outboard to the elevon actuator bay. The stations below are estimated from the exterior model in DCS axes
// (x forward, y up, z right, metres from the model origin), about +/-0.3 m. Each tank is a uniform box for its
// own inertia. With the empty aircraft at x = 0, the normal usage order keeps the CG between 44.4 and 45.5%
// MAC without payload; the manual's aft case (no payload, forward transfer empty) comes out at 47.7%, past the
// 46.5% limit, and its forward failure case (payload in, forward transfer and feed full) at 42.1%.
//
// The EFM tracks only the total; the contents of each tank are a function of it. Fuel burns in the manual's
// order and a partial load fills in the reverse order, feed tanks first.

namespace F117
{
namespace Fuel_tanks
{
constexpr double kg_per_lb = 0.45359237;

enum Tank_index
{
    left_forward,
    right_forward,
    left_feed,
    right_feed,
    left_aft,
    right_aft,
    left_wing,
    right_wing,
    tank_count
};

struct Tank
{
    double capacity_kg;
    double x, y, z;     // centroid, DCS axes
    double dx, dy, dz;  // box size, for the tank's own inertia
};

constexpr Tank tanks[tank_count] = {
    {2700 * kg_per_lb, 1.85, 0.80, -0.60, 1.7, 1.0, 0.8},    // left forward transfer
    {2700 * kg_per_lb, 1.85, 0.80, 0.60, 1.7, 1.0, 0.8},     // right forward transfer
    {2400 * kg_per_lb, 0.10, 0.75, -0.60, 1.8, 0.95, 0.8},   // left feed
    {2400 * kg_per_lb, 0.10, 0.75, 0.60, 1.8, 0.95, 0.8},    // right feed
    {1900 * kg_per_lb, -1.80, 0.10, -0.45, 2.0, 0.95, 0.6},  // left aft transfer
    {1900 * kg_per_lb, -1.80, 0.10, 0.45, 2.0, 0.95, 0.6},   // right aft transfer
    {2100 * kg_per_lb, -1.20, -0.20, -3.30, 3.0, 0.3, 1.8},  // left wing
    {2100 * kg_per_lb, -1.20, -0.20, 3.30, 3.0, 0.3, 1.8},   // right wing
};

constexpr double total_capacity_kg = 18200 * kg_per_lb;

// Usage order (page 1-11). Each stage drains its two tanks at the same rate until one of them is empty.
constexpr int burn_stages[][2] = {
    {left_wing, right_wing},        // both wing tanks
    {left_forward, right_aft},      // until the right aft tank empties
    {left_forward, right_forward},  // until the left forward tank empties
    {right_forward, left_aft},      // until both empty
    {left_feed, right_feed},        // to engine shutdown
};

// Contents of each tank, in kg, for a total quantity.
inline void distribute(double total_kg, double (&contents)[tank_count])
{
    for (int i = 0; i < tank_count; ++i)
        contents[i] = tanks[i].capacity_kg;
    double burned = total_capacity_kg - total_kg;
    if (!(burned == burned))  // NaN: report the tanks full rather than poison the mass
        burned = 0.0;
    if (burned < 0.0)
    {
        // More than the tanks hold (DCS rounding): keep the mass, in the feed tanks.
        contents[left_feed] -= burned / 2.0;
        contents[right_feed] -= burned / 2.0;
        return;
    }
    for (const auto& stage : burn_stages)
    {
        const double a = contents[stage[0]];
        const double b = contents[stage[1]];
        const double each = (burned / 2.0 < (a < b ? a : b)) ? burned / 2.0 : (a < b ? a : b);
        contents[stage[0]] -= each;
        contents[stage[1]] -= each;
        burned -= 2.0 * each;
    }
    // Anything left over means the total was negative: the tanks are empty.
}

// A tank's own inertia per kg of fuel about its centroid: roll (x), yaw (y), pitch (z), as DCS orders them.
inline double own_roll_inertia_per_kg(const Tank& t)
{
    return (t.dy * t.dy + t.dz * t.dz) / 12.0;
}
inline double own_yaw_inertia_per_kg(const Tank& t)
{
    return (t.dx * t.dx + t.dz * t.dz) / 12.0;
}
inline double own_pitch_inertia_per_kg(const Tank& t)
{
    return (t.dx * t.dx + t.dy * t.dy) / 12.0;
}
}  // namespace Fuel_tanks
}  // namespace F117
