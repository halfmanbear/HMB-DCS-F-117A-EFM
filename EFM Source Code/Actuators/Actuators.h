#pragma once

#include <initializer_list>
#include "../EfmMath.h"

namespace F117
{
namespace Actuators
{
// A surface or mechanism that moves toward its command at a speed proportional to the
// remaining travel (gain, 1/s), capped by a speed limit and held inside its travel stops.
struct Servo
{
    Servo(double gain_in, double min_rate_in, double max_rate_in, double min_position_in, double max_position_in)
        : gain(gain_in), min_rate(min_rate_in), max_rate(max_rate_in), min_position(min_position_in),
          max_position(max_position_in)
    {
    }

    double gain;
    double min_rate;
    double max_rate;
    double min_position;
    double max_position;
    double position = 0.0;
    double rate = 0.0;

    double move(double command, double dt)
    {
        rate = limit(gain * (command - position), min_rate, max_rate);
        position = limit(position + rate * dt, min_position, max_position);
        return position;
    }

    void set(double value)
    {
        position = value;
        rate = 0.0;
    }
};

// Cleared at the start of a session. Until the first frame has run, every actuator
// takes its command directly, so the aircraft starts in its mission configuration.
inline bool initialized = false;

//                 gain   rate limits    travel
inline Servo elevator{20.2, -4.0, 4.0, -1.0, 1.0};  // stick-to-elevon path, normalised
inline Servo rudder{20.2, -3.0, 3.0, -1.0, 1.0};    // pedal-to-fin path, normalised
inline Servo throttle{20.2, -25.0, 20.0, 0.0, 100.0};
inline Servo gear{20.2, -0.5, 0.5, 0.0, 1.0};  // 0 up, 1 down
inline Servo tailhook{20.2, -0.75, 0.75, 0.0, 1.0};
// Left and right bay: 0 closed, 1 door open and trapeze down. Two seconds for the full travel.
inline Servo weapon_bay[2] = {Servo(20.2, -0.5, 0.5, 0.0, 1.0), Servo(20.2, -0.5, 0.5, 0.0, 1.0)};
inline Servo drag_chute{20.2, 0.0, 0.75, 0.0, 1.0};  // deploys only; stowing is immediate

inline double drive(Servo& servo, double command, double dt)
{
    if (!initialized)
    {
        servo.position = command;
        return command;
    }
    return servo.move(command, dt);
}

// While the gear can't retract (weight on the wheels), any up command is treated as down.
inline double drive_gear(double command, double dt, bool retraction_blocked)
{
    if (retraction_blocked && command < 1.0)
        command = 1.0;
    return drive(gear, command, dt);
}

// The chute can be streamed only on the ground, from about 60 to 160 kt. Once out it stays
// out until it is released, the jet slows below about 18 kt, or it fails above about 160 kt.
inline double drive_drag_chute(double command, double dt, double ground_speed_mps, bool on_wheels)
{
    if (!initialized)
    {
        drag_chute.position = command;
        return command;
    }

    constexpr double stream_min_mps = 30.87;
    constexpr double fail_speed_mps = 82.31;
    constexpr double collapse_mps = 9.14;
    constexpr double out_threshold = 0.1;

    const bool already_out = drag_chute.position >= out_threshold;
    const bool in_stream_window = ground_speed_mps >= stream_min_mps && ground_speed_mps <= fail_speed_mps;
    const bool speed_kills_chute = ground_speed_mps < collapse_mps || ground_speed_mps > fail_speed_mps;

    double target = command;
    if (command > out_threshold && (!on_wheels || (!already_out && !in_stream_window)))
        target = 0.0;
    if (speed_kills_chute)
        target = 0.0;

    if (target < out_threshold)
    {
        drag_chute.set(0.0);
        return 0.0;
    }
    return drag_chute.move(target, dt);
}

inline void reset_all()
{
    for (Servo* servo : {&elevator, &rudder, &throttle, &gear, &tailhook, &weapon_bay[0], &weapon_bay[1], &drag_chute})
        servo->set(0.0);
}
}  // namespace Actuators
}  // namespace F117
