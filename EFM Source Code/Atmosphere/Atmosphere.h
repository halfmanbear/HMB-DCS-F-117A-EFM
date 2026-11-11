#pragma once
#include <cmath>

namespace F117
{
namespace Atmos
{
// SI air data: dynamic pressure (Pa), followed by Mach.
inline void atmos(double kelvin, double kg_per_cubic_metre, double metres_per_second, double* result)
{
    constexpr double heat_capacity_ratio = 1.4;
    constexpr double gas_constant = 287.058;
    const double sound_speed = std::sqrt(heat_capacity_ratio * gas_constant * kelvin);
    result[1] = metres_per_second / sound_speed;
    result[0] = (0.5 * kg_per_cubic_metre) * metres_per_second * metres_per_second;
}
}  // namespace Atmos
}  // namespace F117
