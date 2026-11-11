#include "stdafx.h"
#include "AirData.h"

#include <cmath>

#include "../Atmosphere/Atmosphere.h"
#include "State.h"

namespace F117
{

namespace
{
constexpr double kMinAirspeed_MPS = 0.003;  // ~0.01 ft/s equivalent minimum

void update_total_velocity_from_airmass()
{
    Vec3 airspeed;
    airspeed.x = velocity_world_cs.x - wind.x;
    airspeed.y = velocity_world_cs.y - wind.y;
    airspeed.z = velocity_world_cs.z - wind.z;

    totalVelocity_MPS = std::sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);
    if (totalVelocity_MPS < kMinAirspeed_MPS)
    {
        totalVelocity_MPS = kMinAirspeed_MPS;
    }
}
}  // namespace

void update_air_data()
{
    update_total_velocity_from_airmass();

    double airData[2]{};
    ATMOS::atmos(ambientTemperature_DegK, ambientDensity_KgPerM3, totalVelocity_MPS, airData);
    dynamicPressure_PA = airData[0];
    mach = airData[1];
}

}  // namespace F117
