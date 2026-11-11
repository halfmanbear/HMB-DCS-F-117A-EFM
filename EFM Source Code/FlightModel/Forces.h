#pragma once

#include "../EfmMath.h"

// Accumulate a body-axis force (N) or moment (N m) for the current frame.
void add_local_force(const Vec3& force);
void add_local_moment(const Vec3& moment);
