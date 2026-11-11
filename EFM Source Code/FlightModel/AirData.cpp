#include "stdafx.h"
#include "AirData.h"

#include <cmath>

#include "../Atmosphere/Atmosphere.h"
#include "State.h"

namespace F117
{

namespace
{
constexpr double min_airspeed_mps = 0.003;  // ~0.01 ft/s equivalent minimum

void update_total_velocity_from_airmass()
{
    Vec3 airspeed;
    airspeed.x = velocity_world_cs.x - wind.x;
    airspeed.y = velocity_world_cs.y - wind.y;
    airspeed.z = velocity_world_cs.z - wind.z;

    total_velocity_mps = std::sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);
    if (total_velocity_mps < min_airspeed_mps)
    {
        total_velocity_mps = min_airspeed_mps;
    }
}
}  // namespace

void update_air_data()
{
    update_total_velocity_from_airmass();

    double air_data[2]{};
    Atmos::atmos(ambient_temperature_deg_k, ambient_density_kg_per_m3, total_velocity_mps, air_data);
    dynamic_pressure_pa = air_data[0];
    mach = air_data[1];
}

}  // namespace F117
