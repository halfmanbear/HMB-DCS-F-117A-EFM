#pragma once
#include <cmath>

namespace F117
{
namespace ATMOS
{
// SI air data: dynamic pressure (Pa), followed by Mach.
inline void atmos(double kelvin, double kgPerCubicMetre, double metresPerSecond, double* result)
{
    constexpr double heatCapacityRatio = 1.4;
    constexpr double gasConstant = 287.058;
    const double soundSpeed = std::sqrt(heatCapacityRatio * gasConstant * kelvin);
    result[1] = metresPerSecond / soundSpeed;
    result[0] = (0.5 * kgPerCubicMetre) * metresPerSecond * metresPerSecond;
}
}  // namespace ATMOS
}  // namespace F117
