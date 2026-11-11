#pragma once

// Small math helpers shared by the flight model.

// Standard gravity, m/s^2.
inline constexpr double kStandardGravity_MPS2 = 9.81;

struct Vec3
{
    Vec3(double xIn = 0.0, double yIn = 0.0, double zIn = 0.0) : x(xIn), y(yIn), z(zIn) {}

    double x;
    double y;
    double z;
};

// Clamp value into [lower, upper]. The upper bound wins if the bounds cross.
inline double limit(double value, double lower, double upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}
