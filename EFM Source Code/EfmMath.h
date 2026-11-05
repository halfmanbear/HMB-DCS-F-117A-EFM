#pragma once

// Small math helpers shared by the flight model.

struct Vec3
{
    Vec3(double xIn = 0.0, double yIn = 0.0, double zIn = 0.0) : x(xIn), y(yIn), z(zIn) {}

    double x;
    double y;
    double z;
};

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}

// Clamp value into [lower, upper]. The upper bound wins if the bounds cross.
inline double limit(double value, double lower, double upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}
