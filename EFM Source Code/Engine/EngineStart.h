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
namespace Engine_start
{
enum Phase
{
    phase_off = 0,
    phase_motoring = 1,
    phase_light_off = 2,
    phase_accelerating = 3,
    phase_running = 4
};
enum Side
{
    left = 0,
    right = 1
};

constexpr double motor_rate = 2.5;          // core rpm %/s on the starter alone
constexpr double starter_max_n2 = 25.0;     // the starter alone can't turn the core faster than this
constexpr double light_off_n2 = 15.0;       // throttle to IDLE: fuel and ignition on
constexpr double light_off_time = 2.0;      // s from fuel on to a stable flame
constexpr double accel_rate = 2.0;          // core rpm %/s once lit, starter assisting
constexpr double starter_cutout_n2 = 54.0;  // starter disengages; the engine accelerates to idle on its own

// One engine.
struct Sequencer
{
    int phase = phase_off;
    double timer = 0.0;
    bool starter = false;  // starter engaged; a windmill relight lights without it

    void reset(bool running)
    {
        phase = running ? phase_running : phase_off;
        timer = 0.0;
        starter = false;
    }

    // An engine still turning above light-off speed (spooling down or windmilling) relights without the
    // starter. Otherwise the starter needs start air. False when the start can't begin.
    bool request_start(double n2, bool start_air)
    {
        if (phase != phase_off)
            return true;
        timer = 0.0;
        if (n2 >= light_off_n2)
        {
            phase = phase_light_off;
            starter = false;
            return true;
        }
        if (!start_air)
            return false;
        phase = phase_motoring;
        starter = true;
        return true;
    }

    // Throttle to OFF: fuel cut, and any start in progress is abandoned.
    void request_stop()
    {
        reset(false);
    }

    bool running() const
    {
        return phase == phase_running;
    }
    bool starting() const
    {
        return phase >= phase_motoring && phase <= phase_accelerating;
    }
    bool lit() const
    {
        return phase >= phase_light_off;
    }
    bool starter_engaged() const
    {
        return starter && starting();
    }

    // While starting this owns N2; otherwise the engine model does. canRun: fuel and an intact engine.
    void update(double dt, double& n2, bool start_air, bool can_run)
    {
        if (phase == phase_off)
            return;
        if (!can_run && lit())
        {
            reset(false);  // flameout or fuel starvation
            return;
        }
        switch (phase)
        {
            case phase_motoring:
                if (!start_air)
                {
                    reset(false);  // start air lost before light-off: the start is abandoned
                    return;
                }
                if (n2 < starter_max_n2)
                    n2 = (std::min)(n2 + motor_rate * dt, starter_max_n2);
                if (n2 >= light_off_n2 && can_run)
                {
                    phase = phase_light_off;
                    timer = 0.0;
                }
                break;
            case phase_light_off:
                if (starter && start_air && n2 < starter_max_n2)
                    n2 = (std::min)(n2 + motor_rate * dt, starter_max_n2);
                timer += dt;
                if (timer >= light_off_time)
                    phase = phase_accelerating;
                break;
            case phase_accelerating:
                n2 += accel_rate * dt;
                if (n2 >= starter_cutout_n2)
                {
                    phase = phase_running;
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
    bool queued[2] = {false, false};
    int rejected = 0;  // start attempts with no start air this session, for the cockpit to report

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
    bool start_air(int side, bool external_air) const
    {
        return external_air || engine[1 - side].running();
    }

    void request_start(int side, double n2, bool external_air)
    {
        if (engine[side].phase != phase_off || queued[side])
            return;
        if (n2 < light_off_n2 && engine[1 - side].starter_engaged())
        {
            queued[side] = true;
            return;
        }
        if (!engine[side].request_start(n2, start_air(side, external_air)))
            ++rejected;
    }

    // Engines Start: right first, then the left on the start air or right engine crossbleed.
    void request_both(const double n2[2], bool external_air)
    {
        request_start(right, n2[right], external_air);
        request_start(left, n2[left], external_air);
    }

    void request_stop(int side)
    {
        engine[side].request_stop();
        queued[side] = false;
    }

    void update(double dt, double n2[2], bool external_air, const bool can_run[2])
    {
        for (int side = 0; side < 2; ++side)
        {
            engine[side].update(dt, n2[side], start_air(side, external_air), can_run[side]);
        }
        // A waiting start begins as soon as the other starter cuts out or its start is abandoned.
        for (int side = 0; side < 2; ++side)
        {
            if (queued[side] && !engine[1 - side].starter_engaged())
            {
                queued[side] = false;
                request_start(side, n2[side], external_air);
            }
        }
    }

    bool any_running() const
    {
        return engine[left].running() || engine[right].running();
    }
};
}  // namespace Engine_start
}  // namespace F117
