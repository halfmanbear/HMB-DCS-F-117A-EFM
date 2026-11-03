#include "../stdafx.h"
#include <cmath>

namespace F117
{
    namespace ENGINE
    {
        // One engine's state. It persists across FM ticks and is reset by the startup hooks.
        struct EngineState
        {
            double N2 = 0.0;                  // core rpm, %
            double thrustFractionOfMax = 0.0;
        };
        // The thrust constants and table below are for both engines together.
        constexpr double ENGINE_COUNT = 2.0;

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
        constexpr double FLIGHT_IDLE_THRUST_SEA_LEVEL_N = 4008.4;
        constexpr double FLIGHT_IDLE_THRUST_PER_10KM_N = 5369.5;
        constexpr double FLIGHT_IDLE_THRUST_ALTITUDE_CAP_M = 9144.0;
        constexpr double MAX_THRUST_LIMIT_N = 94000.0; // Two F404s at about 47 kN each.
        // Installed maximum thrust of both engines, fitted by tools/fm_bench/fit_performance.py in the mod repository
        // to the acceleration, climb, ceiling, top speed and sustained turn data in TO 1F-117A-1.
        constexpr int THRUST_TABLE_ALTITUDES = 5;
        constexpr int THRUST_TABLE_MACHS = 5;
        constexpr double THRUST_TABLE_ALTITUDE_M[THRUST_TABLE_ALTITUDES] = { 0.0, 3048.0, 6096.0, 9144.0, 12192.0 };
        constexpr double THRUST_TABLE_MACH[THRUST_TABLE_MACHS] = { 0.30, 0.50, 0.70, 0.80, 0.90 };
        constexpr double THRUST_TABLE_N[THRUST_TABLE_ALTITUDES][THRUST_TABLE_MACHS] = {
            { 79559.0, 80476.0, 83670.0, 84521.0, 84736.0 },
            { 62262.0, 63631.0, 65313.0, 65328.0, 65472.0 },
            { 45132.0, 45181.0, 46839.0, 48095.0, 49141.0 },
            { 30068.0, 29405.0, 30417.0, 31485.0, 32098.0 },
            { 19766.0, 19336.0, 19196.0, 19278.0, 19520.0 }
        };
        // Below the table's lowest Mach, thrust rises towards this much extra at a standstill, up to the engines' rating.
        // The manual's in-flight data doesn't reach those speeds, so this comes from the takeoff example on TO 1F-117A-1
        // page A3-4 (4,700 ft ground run at 46,000 lb).
        constexpr double STATIC_THRUST_RISE = 0.18;
        constexpr double MIN_THRUST_IDLE_MARGIN = 1.1;
        constexpr double IDLE_THRUST_CUTOFF_FACTOR = 0.9;

        // Bilinear in altitude and Mach. Above the top row it carries on along the same slope, up to ALTITUDE_MAX_M.
        inline double max_thrust_from_table(double altitude, double mach)
        {
            int i = 0;
            while (i < THRUST_TABLE_ALTITUDES - 2 && altitude >= THRUST_TABLE_ALTITUDE_M[i + 1]) ++i;
            int j = 0;
            while (j < THRUST_TABLE_MACHS - 2 && mach >= THRUST_TABLE_MACH[j + 1]) ++j;
            const double a = (altitude - THRUST_TABLE_ALTITUDE_M[i]) / (THRUST_TABLE_ALTITUDE_M[i + 1] - THRUST_TABLE_ALTITUDE_M[i]);
            const double m = limit(mach, THRUST_TABLE_MACH[0], THRUST_TABLE_MACH[THRUST_TABLE_MACHS - 1]);
            const double b = (m - THRUST_TABLE_MACH[j]) / (THRUST_TABLE_MACH[j + 1] - THRUST_TABLE_MACH[j]);
            return (1.0 - a) * ((1.0 - b) * THRUST_TABLE_N[i][j] + b * THRUST_TABLE_N[i][j + 1])
                 + a * ((1.0 - b) * THRUST_TABLE_N[i + 1][j] + b * THRUST_TABLE_N[i + 1][j + 1]);
        }

        // Advances one engine and returns its thrust (N).
        double engine_dynamics(EngineState& engine,
            double throttleInput,
            double mach,
            double alt,
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

                double rate = (targetN2 > engine.N2) ? SPOOL_UP_RATE : SPOOL_DOWN_RATE;

                engine.N2 += (targetN2 - engine.N2) * rate * frameTime;
                engine.N2 = limit(engine.N2, 0.0, MAX_N2);
            }
            else
            {
                // Shutdown or flameout: allow a gradual windmill decay.
                engine.N2 += (0.0 - engine.N2) * WINDMILL_DECAY * frameTime;
                if (engine.N2 < SHUTDOWN_N2_THRESHOLD)
                {
                    engine.N2 = 0.0;
                }
            }

            // Below this threshold the engine is treated as producing no useful thrust.
            if (engine.N2 < idleN2 * IDLE_THRUST_CUTOFF_FACTOR)
            {
                engine.thrustFractionOfMax = 0.0;
                return 0.0;
            }

            // Ground idle thrust is held lower than airborne idle so the aircraft
            // does not start creeping on the runway at zero throttle.
            double Tidle = weightOnWheels
                ? GROUND_IDLE_THRUST_N * max_thrust_from_table(altitude, THRUST_TABLE_MACH[0]) / THRUST_TABLE_N[0][0]
                : FLIGHT_IDLE_THRUST_SEA_LEVEL_N + FLIGHT_IDLE_THRUST_PER_10KM_N * (std::min)(altitude, FLIGHT_IDLE_THRUST_ALTITUDE_CAP_M) / 10000.0;
            double Tmax = max_thrust_from_table(altitude, machLimited);
            if (machLimited < THRUST_TABLE_MACH[0])
            {
                Tmax *= 1.0 + STATIC_THRUST_RISE * (1.0 - machLimited / THRUST_TABLE_MACH[0]);
            }

            if (Tmax < Tidle)
            {
                Tmax = Tidle * MIN_THRUST_IDLE_MARGIN;
            }

            // Normalize N2 above idle before shaping the thrust response.
            double normN2 =
                (engine.N2 - idleN2) / (MAX_N2 - idleN2);

            normN2 = limit(normN2, 0.0, 1.0);

            // Preserve the existing non-linear thrust curve.
            double thrustFraction = pow(normN2, 2.5);

            double thrust =
                Tidle + thrustFraction * (Tmax - Tidle);

            thrust = limit(thrust, 0.0, MAX_THRUST_LIMIT_N);
            engine.thrustFractionOfMax = Tmax > 0.0 ? limit(thrust / Tmax, 0.0, 1.0) : 0.0;

            return thrust / ENGINE_COUNT;
        }
    }
}


