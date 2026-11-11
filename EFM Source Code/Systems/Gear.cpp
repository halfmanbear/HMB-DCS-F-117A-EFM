#include "stdafx.h"
#include "Gear.h"

#include "../FlightModel/State.h"

bool gear_retraction_blocked()
{
    return F117::weight_on_wheels || F117::wow_from_draw_args;
}

double clamp_gear_command_for_weight_on_wheels(double requested_command)
{
    if (gear_retraction_blocked() && requested_command < gear_command_down)
    {
        return gear_command_down;
    }

    return requested_command;
}

void set_gear_command(double requested_command)
{
    F117::gear_command = clamp_gear_command_for_weight_on_wheels(requested_command);
}

// Weight on wheels is wheel contact only: a compressed strut reported by DCS, or the strut draw args. Lift below
// weight is not a test for it, because that is also true in the air with the gear down, through the whole climb
// after takeoff, and it held the gear handle down there.
void update_weight_on_wheels()
{
    const bool suspension_weight_on_wheels =
        F117::suspension_wow[0] || F117::suspension_wow[1] || F117::suspension_wow[2];
    F117::weight_on_wheels = F117::wow_from_draw_args || suspension_weight_on_wheels;
    if (F117::weight_on_wheels)
    {
        F117::gear_command = gear_command_down;
    }
}
