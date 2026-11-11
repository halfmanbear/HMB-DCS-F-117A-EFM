#pragma once

namespace F117
{

// Updates the air-relative speed (from body velocity and the current wind estimate), dynamic
// pressure and Mach number for this frame.
void update_air_data();

}  // namespace F117
