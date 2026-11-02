#include "../stdafx.h"
#include <cmath>

namespace F117
{
    namespace ENGINE
    {
        // Engine state persists across FM ticks and is reset by the startup hooks.
        static double N2 = 0.0;
        static double thrustFractionOfMax = 0.0;

        constexpr double THROTTLE_MIN = 0.0;
        constexpr double THROTTLE_MAX = 100.0;
        constexpr double MACH_MIN = 0.0;
        constexpr double MACH_MAX = 0.92;
        constexpr double ALTITUDE_MIN_M = 0.0;
        constexpr double ALTITUDE_MAX_M = 13716.0; // 45000 ft
        constexpr double GROUND_IDLE_N2 = 60.0;
        constexpr double FLIGHT_IDLE_N2 = 65.0;
        constexpr double MAX_N2 = 100.0;
        constexpr double SPOOL_UP_RATE = 0.65;
        constexpr double SPOOL_DOWN_RATE = 0.45;
        constexpr double WINDMILL_DECAY = 0.10;
        constexpr double SHUTDOWN_N2_THRESHOLD = 0.5;
        constexpr double GROUND_IDLE_THRUST_N = 3000.0;
        // Net thrust at flight idle, fitted to the idle deceleration in TO 1F-117A-1 Figure A9-11. That data stops at 30,000 ft.
        constexpr double FLIGHT_IDLE_THRUST_SEA_LEVEL_N = 5172.0;
        constexpr double FLIGHT_IDLE_THRUST_PER_10KM_N = 6066.0;
        constexpr double FLIGHT_IDLE_THRUST_ALTITUDE_CAP_M = 9144.0;
        // Installed maximum thrust, fitted by tools/fm_bench/fit_performance.py in the mod repository to the
        // acceleration, climb, ceiling and sustained turn data in TO 1F-117A-1.
        constexpr double MAX_THRUST_SEA_LEVEL_N = 78374.0; // Before the Mach terms below, so not a static thrust figure.
        constexpr double MAX_THRUST_LIMIT_N = 94000.0; // Two F404s at about 47 kN each.
        constexpr double DPDH_M_SCALE_HEIGHT = 13951.0;
        constexpr double DPDH_CURVE_PER_10KM_SQ = 0.3;
        constexpr double MACH_THRUST_FIT_FLOOR = 0.30; // The manual's in-flight data doesn't constrain thrust below about here.
        // Below that Mach, thrust rises towards this much extra at a standstill. The takeoff example on TO 1F-117A-1
        // page A3-4 (4,700 ft ground run at 46,000 lb) needs about 92 kN at low speed.
        constexpr double STATIC_THRUST_RISE = 0.135;
        constexpr double MACH_THRUST = 0.0094636;
        constexpr double MACH_THRUST_SQ = 0.34578;
        constexpr double MACH_THRUST_SQ_PER_10KM = -0.30334;
        constexpr double HIGH_Q_THRUST_LIMIT_PA = 18383.0;
        constexpr double HIGH_Q_THRUST_LOSS = 0.07614;
        constexpr double HIGH_Q_THRUST_MIN_FRACTION = 0.3;
        constexpr double MIN_THRUST_IDLE_MARGIN = 1.1;
        constexpr double IDLE_THRUST_CUTOFF_FACTOR = 0.9;

        double engine_dynamics(double throttleInput,
            double mach,
            double alt,
            double dynamicPressure_PA,
            double frameTime,
            bool engineRunning = true,
            bool weightOnWheels = false)
        {
            // Clamp the incoming conditions to the model's intended operating range.
            double throttle = limit(throttleInput, THROTTLE_MIN, THROTTLE_MAX);
            double machLimited = limit(mach, MACH_MIN, MACH_MAX);
            double altitude = limit(alt, ALTITUDE_MIN_M, ALTITUDE_MAX_M);

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
                thrustFractionOfMax = 0.0;
                return 0.0;
            }

            const double altitude_10km = altitude / 10000.0;
            double altitude_factor = exp(-altitude / DPDH_M_SCALE_HEIGHT - DPDH_CURVE_PER_10KM_SQ * altitude_10km * altitude_10km);

            // Ground idle thrust is held lower than airborne idle so the aircraft
            // does not start creeping on the runway at zero throttle.
            double Tidle = weightOnWheels
                ? GROUND_IDLE_THRUST_N * altitude_factor
                : FLIGHT_IDLE_THRUST_SEA_LEVEL_N + FLIGHT_IDLE_THRUST_PER_10KM_N * (std::min)(altitude, FLIGHT_IDLE_THRUST_ALTITUDE_CAP_M) / 10000.0;
            double M = limit(machLimited, MACH_THRUST_FIT_FLOOR, MACH_MAX);
            double mach_factor = 1.0 + MACH_THRUST * M + (MACH_THRUST_SQ + MACH_THRUST_SQ_PER_10KM * altitude_10km) * M * M;
            if (machLimited < MACH_THRUST_FIT_FLOOR)
            {
                mach_factor *= 1.0 + STATIC_THRUST_RISE * (1.0 - machLimited / MACH_THRUST_FIT_FLOOR);
            }
            // Thrust falls away at high dynamic pressure, seen at low altitude near the Mach limit.
            double q_factor = limit(1.0 - HIGH_Q_THRUST_LOSS * (dynamicPressure_PA - HIGH_Q_THRUST_LIMIT_PA) / HIGH_Q_THRUST_LIMIT_PA,
                HIGH_Q_THRUST_MIN_FRACTION, 1.0);

            double Tmax = MAX_THRUST_SEA_LEVEL_N * altitude_factor * mach_factor * q_factor;

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
            thrustFractionOfMax = Tmax > 0.0 ? limit(thrust / Tmax, 0.0, 1.0) : 0.0;

            return thrust;
        }
    }
}


