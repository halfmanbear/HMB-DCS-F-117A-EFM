#pragma once
// Internal fuel tanks: where the fuel is for any total quantity, so DCS gets the right mass, CG and inertia.
//
// Tanks: the TO 00-105E-9 fuel diagram. Nine fuselage tanks in one block on the centreline, front to back:
// forward transfer 1 (397 gal), 2 and 3 (198.5 each), feed 4A to 4D (176.5 each), aft transfer 5 (279.4) and
// 6 (297.4). Each has a baffle on the centreline, so it is modelled as a left and a right half. The two wing
// tanks (308.8 gal each) run along the leading edge to about 85% of the semispan, cells A to E, larger outboard.
// The listed tanks hold 2,694.4 gal; the diagram's 2,812 gal total includes 117.6 gal not in any listed tank,
// left out here. Capacities are the diagram's gallons scaled to the 18,200 lb of TO 1F-117A-1 Figure 1-7 (and
// M_fuel_max), 6.755 lb/gal; per side that is 2,682 / 2,384 / 1,948 / 2,086 lb, which the manual rounds to
// 2,700 / 2,400 / 1,900 / 2,100.
//
// Usage order: TO 1F-117A-1 page 1-11, by group (left/right forward transfer, feed, aft transfer, wing). The
// tanks in a group drain together in proportion to their capacity, so each group's centroid stays fixed.
//
// Positions in DCS axes (x forward, y up, z right, metres from the model origin), estimated from the exterior
// model, about +/-0.3 m. Each group is a box split front to back by volume; the halves meet at the baffle. Each
// piece is a uniform box for its own inertia. With the empty aircraft at x = 0, the normal usage order keeps the
// CG between 44.5 and 45.8% MAC without payload; the manual's aft case (no payload, forward transfer empty) comes
// out at 48.0%, past the 46.5% limit, and its forward failure case (payload in, forward transfer and feed full)
// at about 42%.
//
// The EFM tracks only the total; the contents of each tank are a function of it. Fuel burns in the manual's
// order and a partial load fills in the reverse order, feed tanks first.

namespace F117
{
namespace Fuel_tanks
{
constexpr double kg_per_lb = 0.45359237;
constexpr double total_capacity_kg = 18200 * kg_per_lb;
constexpr double kg_per_gal = total_capacity_kg / 2694.4;

// The groups the usage order names.
enum Group
{
    left_forward,
    right_forward,
    left_feed,
    right_feed,
    left_aft,
    right_aft,
    left_wing,
    right_wing,
    group_count
};

struct Tank
{
    Group group;
    double capacity_kg;
    double x, y, z;     // centroid, DCS axes
    double dx, dy, dz;  // box size, for the tank's own inertia
};

// Forward transfer x 1.00 to 2.70, feed -0.80 to 1.00, aft transfer -2.80 to -0.80.
constexpr Tank tanks[] = {
    {left_forward, 198.5 * kg_per_gal, 2.2750, 0.80, -0.40, 0.850, 1.0, 0.8},    // 1 left
    {right_forward, 198.5 * kg_per_gal, 2.2750, 0.80, 0.40, 0.850, 1.0, 0.8},    // 1 right
    {left_forward, 99.25 * kg_per_gal, 1.6375, 0.80, -0.40, 0.425, 1.0, 0.8},    // 2 left
    {right_forward, 99.25 * kg_per_gal, 1.6375, 0.80, 0.40, 0.425, 1.0, 0.8},    // 2 right
    {left_forward, 99.25 * kg_per_gal, 1.2125, 0.80, -0.40, 0.425, 1.0, 0.8},    // 3 left
    {right_forward, 99.25 * kg_per_gal, 1.2125, 0.80, 0.40, 0.425, 1.0, 0.8},    // 3 right
    {left_feed, 88.25 * kg_per_gal, 0.775, 0.75, -0.40, 0.45, 0.95, 0.8},        // 4A left
    {right_feed, 88.25 * kg_per_gal, 0.775, 0.75, 0.40, 0.45, 0.95, 0.8},        // 4A right
    {left_feed, 88.25 * kg_per_gal, 0.325, 0.75, -0.40, 0.45, 0.95, 0.8},        // 4B left
    {right_feed, 88.25 * kg_per_gal, 0.325, 0.75, 0.40, 0.45, 0.95, 0.8},        // 4B right
    {left_feed, 88.25 * kg_per_gal, -0.125, 0.75, -0.40, 0.45, 0.95, 0.8},       // 4C left
    {right_feed, 88.25 * kg_per_gal, -0.125, 0.75, 0.40, 0.45, 0.95, 0.8},       // 4C right
    {left_feed, 88.25 * kg_per_gal, -0.575, 0.75, -0.40, 0.45, 0.95, 0.8},       // 4D left
    {right_feed, 88.25 * kg_per_gal, -0.575, 0.75, 0.40, 0.45, 0.95, 0.8},       // 4D right
    {left_aft, 139.7 * kg_per_gal, -1.284, 0.10, -0.30, 0.969, 0.95, 0.6},       // 5 left
    {right_aft, 139.7 * kg_per_gal, -1.284, 0.10, 0.30, 0.969, 0.95, 0.6},       // 5 right
    {left_aft, 148.7 * kg_per_gal, -2.284, 0.10, -0.30, 1.031, 0.95, 0.6},       // 6 left
    {right_aft, 148.7 * kg_per_gal, -2.284, 0.10, 0.30, 1.031, 0.95, 0.6},       // 6 right
    {left_wing, 308.8 * kg_per_gal, -1.50, -0.20, -4.30, 3.0, 0.3, 2.4},         // left wing
    {right_wing, 308.8 * kg_per_gal, -1.50, -0.20, 4.30, 3.0, 0.3, 2.4},         // right wing
};
constexpr int tank_count = sizeof(tanks) / sizeof(tanks[0]);

constexpr double group_capacity_kg(int group)
{
    double sum = 0.0;
    for (const Tank& t : tanks)
        sum += t.group == group ? t.capacity_kg : 0.0;
    return sum;
}

// Usage order (page 1-11). Each stage drains its two groups at the same rate until one of them is empty.
constexpr int burn_stages[][2] = {
    {left_wing, right_wing},        // both wing tanks
    {left_forward, right_aft},      // until the right aft tanks empty
    {left_forward, right_forward},  // until the left forward tanks empty
    {right_forward, left_aft},      // until both empty
    {left_feed, right_feed},        // to engine shutdown
};

// Contents of each group, in kg, for a total quantity.
inline void distribute_groups(double total_kg, double (&contents)[group_count])
{
    for (int g = 0; g < group_count; ++g)
        contents[g] = group_capacity_kg(g);
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

// Contents of each tank, in kg, for a total quantity: its group's share in proportion to capacity.
inline void distribute(double total_kg, double (&contents)[tank_count])
{
    double groups[group_count];
    distribute_groups(total_kg, groups);
    for (int i = 0; i < tank_count; ++i)
        contents[i] = groups[tanks[i].group] * tanks[i].capacity_kg / group_capacity_kg(tanks[i].group);
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
