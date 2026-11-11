#pragma once

#include <initializer_list>
#include "../EfmMath.h"

namespace F117 { namespace ACTUATORS {
        // A surface or mechanism that moves toward its command at a speed proportional to the
        // remaining travel (gain, 1/s), capped by a speed limit and held inside its travel stops.
        struct Servo
        {
            Servo(double gainIn, double minRateIn, double maxRateIn, double minPositionIn, double maxPositionIn)
                : gain(gainIn), minRate(minRateIn), maxRate(maxRateIn),
                  minPosition(minPositionIn), maxPosition(maxPositionIn) {}

            double gain;
            double minRate;
            double maxRate;
            double minPosition;
            double maxPosition;
            double position = 0.0;
            double rate = 0.0;

            double move(double command, double dt)
            {
                rate = limit(gain * (command - position), minRate, maxRate);
                position = limit(position + rate * dt, minPosition, maxPosition);
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
        inline Servo elevator   { 20.2,  -4.0,  4.0,   -1.0,   1.0 };   // stick-to-elevon path, normalised
        inline Servo rudder     { 20.2,  -3.0,  3.0,   -1.0,   1.0 };   // pedal-to-fin path, normalised
        inline Servo throttle   { 20.2, -25.0, 20.0,    0.0, 100.0 };
        inline Servo gear       { 20.2,  -0.5,  0.5,    0.0,   1.0 };   // 0 up, 1 down
        inline Servo tailhook   { 20.2, -0.75, 0.75,    0.0,   1.0 };
        // Left and right bay: 0 closed, 1 door open and trapeze down. Two seconds for the full travel.
        inline Servo weaponBay[2] = { Servo(20.2, -0.5, 0.5, 0.0, 1.0), Servo(20.2, -0.5, 0.5, 0.0, 1.0) };
        inline Servo dragChute  { 20.2,   0.0, 0.75,    0.0,   1.0 };   // deploys only; stowing is immediate

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
        inline double drive_gear(double command, double dt, bool retractionBlocked)
        {
            if (retractionBlocked && command < 1.0)
                command = 1.0;
            return drive(gear, command, dt);
        }

        // The chute can be streamed only on the ground, from about 60 to 160 kt. Once out it stays
        // out until it is released, the jet slows below about 18 kt, or it fails above about 160 kt.
        inline double drive_drag_chute(double command, double dt, double groundSpeed_MPS, bool weightOnWheels)
        {
            if (!initialized)
            {
                dragChute.position = command;
                return command;
            }

            constexpr double kStreamMin_MPS = 30.87;
            constexpr double kFailSpeed_MPS = 82.31;
            constexpr double kCollapse_MPS = 9.14;
            constexpr double kOutThreshold = 0.1;

            const bool alreadyOut = dragChute.position >= kOutThreshold;
            const bool inStreamWindow = groundSpeed_MPS >= kStreamMin_MPS && groundSpeed_MPS <= kFailSpeed_MPS;
            const bool speedKillsChute = groundSpeed_MPS < kCollapse_MPS || groundSpeed_MPS > kFailSpeed_MPS;

            double target = command;
            if (command > kOutThreshold && (!weightOnWheels || (!alreadyOut && !inStreamWindow)))
                target = 0.0;
            if (speedKillsChute)
                target = 0.0;

            if (target < kOutThreshold)
            {
                dragChute.set(0.0);
                return 0.0;
            }
            return dragChute.move(target, dt);
        }

        inline void reset_all()
        {
            for (Servo* servo : { &elevator, &rudder, &throttle, &gear, &tailhook, &weaponBay[0], &weaponBay[1], &dragChute })
                servo->set(0.0);
        }
}}
