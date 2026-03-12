#include "../stdafx.h"
#include <cmath>

namespace F117
{
    namespace ENGINE
    {
        // Engine state persists across FM ticks and is reset by the startup hooks.
        static double N2 = 0.0;

        constexpr double THROTTLE_MIN = 0.0;
        constexpr double THROTTLE_MAX = 100.0;
        constexpr double MACH_MIN = 0.0;
        constexpr double MACH_MAX = 0.92;
        constexpr double ALTITUDE_MIN_FT = 0.0;
        constexpr double ALTITUDE_MAX_FT = 45000.0;
        constexpr double GROUND_IDLE_N2 = 60.0;
        constexpr double FLIGHT_IDLE_N2 = 65.0;
        constexpr double MAX_N2 = 100.0;
        constexpr double SPOOL_UP_RATE = 0.65;
        constexpr double SPOOL_DOWN_RATE = 0.45;
        constexpr double WINDMILL_DECAY = 0.10;
        constexpr double SHUTDOWN_N2_THRESHOLD = 0.5;
        constexpr double GROUND_IDLE_THRUST_N = 3000.0;
        constexpr double FLIGHT_IDLE_THRUST_N = 8000.0;
        constexpr double MAX_THRUST_SEA_LEVEL_N = 80400.0;
        constexpr double MAX_THRUST_LIMIT_N = 96000.0;
        constexpr double ALTITUDE_THRUST_LOSS_FACTOR = 0.7;
        constexpr double MACH_THRUST_GAIN = 0.15;
        constexpr double MIN_THRUST_IDLE_MARGIN = 1.1;
        constexpr double IDLE_THRUST_CUTOFF_FACTOR = 0.9;

        double engine_dynamics(double throttleInput,
            double mach,
            double alt,
            double frameTime,
            bool engineRunning = true,
            bool weightOnWheels = false)
        {
            // Clamp the incoming conditions to the model's intended operating range.
            double throttle = limit(throttleInput, THROTTLE_MIN, THROTTLE_MAX);
            double machLimited = limit(mach, MACH_MIN, MACH_MAX);
            double altitude = limit(alt, ALTITUDE_MIN_FT, ALTITUDE_MAX_FT);

            // Ground idle remains lower than flight idle.
            double idleN2 = weightOnWheels ? GROUND_IDLE_N2 : FLIGHT_IDLE_N2;

            if (engineRunning)
            {
                // FADEC-style schedule: throttle commands a target N2.
                double targetN2 =
                    idleN2 + (throttle / THROTTLE_MAX) * (MAX_N2 - idleN2);

                double rate = (targetN2 > N2) ? SPOOL_UP_RATE : SPOOL_DOWN_RATE;

                N2 += (targetN2 - N2) * rate * frameTime;
                N2 = limit(N2, 0.0, MAX_N2);
            }
            else
            {
                // Shutdown or flameout: allow a gradual windmill decay.
                N2 += (0.0 - N2) * WINDMILL_DECAY * frameTime;
                if (N2 < SHUTDOWN_N2_THRESHOLD)
                {
                    N2 = 0.0;
                }
            }

            // Below this threshold the engine is treated as producing no useful thrust.
            if (N2 < idleN2 * IDLE_THRUST_CUTOFF_FACTOR)
            {
                return 0.0;
            }

            double altitude_factor =
                1.0 - (altitude / ALTITUDE_MAX_FT) * ALTITUDE_THRUST_LOSS_FACTOR;
            const double idleThrustBaselineN =
                weightOnWheels ? GROUND_IDLE_THRUST_N : FLIGHT_IDLE_THRUST_N;

            // Ground idle thrust is held lower than airborne idle so the aircraft
            // does not start creeping on the runway at zero throttle.
            double Tidle = idleThrustBaselineN * altitude_factor;
            double mach_factor = 1.0 + MACH_THRUST_GAIN * machLimited;

            double Tmax = MAX_THRUST_SEA_LEVEL_N * altitude_factor * mach_factor;

            if (Tmax < Tidle)
            {
                Tmax = Tidle * MIN_THRUST_IDLE_MARGIN;
            }

            // Normalize N2 above idle before shaping the thrust response.
            double normN2 =
                (N2 - idleN2) / (MAX_N2 - idleN2);

            normN2 = limit(normN2, 0.0, 1.0);

            // Preserve the existing non-linear thrust curve.
            double thrustFraction = pow(normN2, 2.5);

            double thrust =
                Tidle + thrustFraction * (Tmax - Tidle);

            thrust = limit(thrust, 0.0, MAX_THRUST_LIMIT_N);

            return thrust;
        }
    }
}


