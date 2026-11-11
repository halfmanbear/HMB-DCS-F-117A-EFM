#pragma once

// Small math helpers shared by the flight model.

// Standard gravity, m/s^2.
inline constexpr double standard_gravity_mps2 = 9.81;

struct Vec3
{
    Vec3(double x_in = 0.0, double y_in = 0.0, double z_in = 0.0) : x(x_in), y(y_in), z(z_in)
    {
    }

    double x;
    double y;
    double z;
};

// Clamp value into [lower, upper]. The upper bound wins if the bounds cross.
inline double limit(double value, double lower, double upper)
{
    if (value > upper)
        return upper;
    if (value < lower)
        return lower;
    return value;
}
