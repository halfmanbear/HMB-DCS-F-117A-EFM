#include "../stdafx.h"
#include <cmath>

namespace F117
{
    namespace ENGINE
    {
        // Engine state (persistent per session, reset via start functions)
        static double N2 = 0.0;   // Core RPM (%), 0 = cold/off

        double engine_dynamics(double throttleInput,
            double mach,
            double alt,
            double frameTime,
            bool engineRunning = true,
            bool weightOnWheels = false)
        {
            // ------------------------------------------------------------
            // 1. LIMIT INPUTS
            // ------------------------------------------------------------
            double throttle = limit(throttleInput, 0.0, 100.0);
            double machLimited = limit(mach, 0.0, 0.92);
            double altitude = limit(alt, 0.0, 45000.0);

            // ------------------------------------------------------------
            // 2. RPM PARAMETERS (F404-F1D2 characteristics)
            // ------------------------------------------------------------
            double idleN2 = weightOnWheels ? 60.0 : 65.0;   // Ground idle lower than flight idle
            double maxN2 = 100.0;

            // ------------------------------------------------------------
            // 3. SPOOL DYNAMICS
            // ------------------------------------------------------------
            // Idle -> Max : ~4-5 sec  (tau ~1.5s, rate ~0.65)
            // Max -> Idle : ~6-7 sec  (tau ~2.2s, rate ~0.45)
            double spoolUpRate = 0.65;
            double spoolDownRate = 0.45;
            double windmillDecay = 0.10;

            if (engineRunning)
            {
                // FADEC: throttle commands target RPM
                double targetN2 =
                    idleN2 + (throttle / 100.0) * (maxN2 - idleN2);

                double rate = (targetN2 > N2) ? spoolUpRate : spoolDownRate;

                N2 += (targetN2 - N2) * rate * frameTime;
                N2 = limit(N2, 0.0, maxN2);
            }
            else
            {
                // Flame-out / shutdown: gradual spool-down
                N2 += (0.0 - N2) * windmillDecay * frameTime;
                if (N2 < 0.5) N2 = 0.0;
            }

            // No useful thrust below ~90% of idle N2
            if (N2 < idleN2 * 0.9)
                return 0.0;

            // ------------------------------------------------------------
            // 4. THRUST MODEL (F404-F1D2, non-afterburning)
            // ------------------------------------------------------------
            // Idle thrust decreases with altitude (less air density)
            double Tidle = 8000.0 * (1.0 - (altitude / 45000.0) * 0.7);

            double altitude_factor = 1.0 - (altitude / 45000.0) * 0.7;
            // Ram recovery: ~14% increase at Mach 0.92
            double mach_factor = 1.0 + 0.15 * machLimited;

            double Tmax = 80400.0 * altitude_factor * mach_factor;

            if (Tmax < Tidle)
                Tmax = Tidle * 1.1;

            // Normalize RPM above idle
            double normN2 =
                (N2 - idleN2) / (maxN2 - idleN2);

            normN2 = limit(normN2, 0.0, 1.0);

            // Non-linear thrust response (real turbofan ~N2^2.5)
            double thrustFraction = pow(normN2, 2.5);

            double thrust =
                Tidle + thrustFraction * (Tmax - Tidle);

            thrust = limit(thrust, 0.0, 96000.0);

            return thrust;
        }
    }
}
