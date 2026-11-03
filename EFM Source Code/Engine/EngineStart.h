#pragma once
#include <algorithm>

// Main engine start, one sequence per engine. TO 1F-117A-1 section I and II:
// - The air turbine starter on each AMAD runs on start air from the APU, an external air cart, or crossbleed
//   from the other engine once it is running. The procedure is the same whichever supplies it.
// - The throttle goes from OFF to IDLE at 10-15 % core rpm (light-off), the starter disengages as core rpm passes
//   54 %, and the engine settles at ground idle.
// - The right engine is the preferred first engine.
// Rates are chosen to give a start of about 30 s; the manual gives no times.
namespace F117
{
    namespace ENGINE_START
    {
        enum Phase { OFF = 0, MOTORING = 1, LIGHT_OFF = 2, ACCELERATING = 3, RUNNING = 4 };
        enum Side { LEFT = 0, RIGHT = 1 };

        constexpr double MOTOR_RATE = 2.5;          // core rpm %/s on the starter alone
        constexpr double STARTER_MAX_N2 = 25.0;     // the starter alone can't turn the core faster than this
        constexpr double LIGHT_OFF_N2 = 15.0;       // throttle to IDLE: fuel and ignition on
        constexpr double LIGHT_OFF_TIME = 2.0;      // s from fuel on to a stable flame
        constexpr double ACCEL_RATE = 2.0;          // core rpm %/s once lit, starter assisting
        constexpr double STARTER_CUTOUT_N2 = 54.0;  // starter disengages; the engine accelerates to idle on its own

        // One engine.
        struct Sequencer
        {
            int phase = OFF;
            double timer = 0.0;
            bool starter = false; // starter engaged; a windmill relight lights without it

            void reset(bool running)
            {
                phase = running ? RUNNING : OFF;
                timer = 0.0;
                starter = false;
            }

            // An engine still turning above light-off speed (spooling down or windmilling) relights without the
            // starter. Otherwise the starter needs start air. False when the start can't begin.
            bool request_start(double n2, bool startAir)
            {
                if (phase != OFF) return true;
                timer = 0.0;
                if (n2 >= LIGHT_OFF_N2)
                {
                    phase = LIGHT_OFF;
                    starter = false;
                    return true;
                }
                if (!startAir) return false;
                phase = MOTORING;
                starter = true;
                return true;
            }

            // Throttle to OFF: fuel cut, and any start in progress is abandoned.
            void request_stop() { reset(false); }

            bool running() const { return phase == RUNNING; }
            bool starting() const { return phase >= MOTORING && phase <= ACCELERATING; }
            bool lit() const { return phase >= LIGHT_OFF; }
            bool starter_engaged() const { return starter && starting(); }

            // While starting this owns N2; otherwise the engine model does. canRun: fuel and an intact engine.
            void update(double dt, double& n2, bool startAir, bool canRun)
            {
                if (phase == OFF) return;
                if (!canRun && lit())
                {
                    reset(false); // flameout or fuel starvation
                    return;
                }
                switch (phase)
                {
                case MOTORING:
                    if (!startAir)
                    {
                        reset(false); // start air lost before light-off: the start is abandoned
                        return;
                    }
                    if (n2 < STARTER_MAX_N2) n2 = (std::min)(n2 + MOTOR_RATE * dt, STARTER_MAX_N2);
                    if (n2 >= LIGHT_OFF_N2 && canRun)
                    {
                        phase = LIGHT_OFF;
                        timer = 0.0;
                    }
                    break;
                case LIGHT_OFF:
                    if (starter && startAir && n2 < STARTER_MAX_N2) n2 = (std::min)(n2 + MOTOR_RATE * dt, STARTER_MAX_N2);
                    timer += dt;
                    if (timer >= LIGHT_OFF_TIME) phase = ACCELERATING;
                    break;
                case ACCELERATING:
                    n2 += ACCEL_RATE * dt;
                    if (n2 >= STARTER_CUTOUT_N2)
                    {
                        phase = RUNNING;
                        starter = false;
                    }
                    break;
                default:
                    break;
                }
            }
        };

        // Both engines and their start air. The start air feeds one starter at a time, so a start requested while
        // the other engine's starter is engaged waits for that starter to cut out.
        struct Starts
        {
            Sequencer engine[2];
            bool queued[2] = { false, false };
            int rejected = 0; // start attempts with no start air this session, for the cockpit to report

            void reset(bool running)
            {
                for (int side = 0; side < 2; ++side)
                {
                    engine[side].reset(running);
                    queued[side] = false;
                }
                rejected = 0;
            }

            // externalAir: the APU or an air cart. A running engine crossbleeds to the other.
            bool start_air(int side, bool externalAir) const
            {
                return externalAir || engine[1 - side].running();
            }

            void request_start(int side, double n2, bool externalAir)
            {
                if (engine[side].phase != OFF || queued[side]) return;
                if (n2 < LIGHT_OFF_N2 && engine[1 - side].starter_engaged())
                {
                    queued[side] = true;
                    return;
                }
                if (!engine[side].request_start(n2, start_air(side, externalAir))) ++rejected;
            }

            // Engines Start: right first, then the left on the start air or right engine crossbleed.
            void request_both(const double n2[2], bool externalAir)
            {
                request_start(RIGHT, n2[RIGHT], externalAir);
                request_start(LEFT, n2[LEFT], externalAir);
            }

            void request_stop(int side)
            {
                engine[side].request_stop();
                queued[side] = false;
            }

            void update(double dt, double n2[2], bool externalAir, const bool canRun[2])
            {
                for (int side = 0; side < 2; ++side)
                {
                    engine[side].update(dt, n2[side], start_air(side, externalAir), canRun[side]);
                }
                // A waiting start begins as soon as the other starter cuts out or its start is abandoned.
                for (int side = 0; side < 2; ++side)
                {
                    if (queued[side] && !engine[1 - side].starter_engaged())
                    {
                        queued[side] = false;
                        request_start(side, n2[side], externalAir);
                    }
                }
            }

            bool any_running() const { return engine[LEFT].running() || engine[RIGHT].running(); }
        };
    }
}
