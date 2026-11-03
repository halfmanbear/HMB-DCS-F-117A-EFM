"""Main engine start: start air from the APU, an air cart or crossbleed, one starter at a time, light-off,
starter cut-out, relight, and abandoned and failed starts, on two engines."""
from pathlib import Path
import re, subprocess, sys
repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
src = repo / 'EFM Source Code'
engine = (src / 'Engine/Engine.h').read_text(encoding='utf-8-sig').replace('#include "../stdafx.h"', '')
source = (src / 'F_117_NightHawk.cpp').read_text(encoding='utf-8-sig')


def extract(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


constants = '\n'.join(re.search(r'constexpr double ' + name + r' = [^;]+;', source).group()
                      for name in ['kIdleEgtCelsius', 'kMaxEgtCelsius', 'kStartEgtCelsius', 'kEgtHeatTau', 'kEgtCoolTau',
                                   'kIdleOilPsi', 'kMaxOilPsi', 'kStartOilPsi', 'kOilTau'])
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
double limit(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
''' + engine + r'''
#include "START_HEADER"
using namespace F117;
using namespace F117::ENGINE_START;

// The same order as update_propulsion_and_control_effectiveness: each sequence owns its N2 while starting.
struct Aircraft {
    Starts starts; ENGINE::EngineState eng[2]; bool air = true, fuel[2] = { true, true }, ground = true;
    double thrust[2] = { 0, 0 }, t = 0;
    void step(double dt) {
        double n2[2] = { eng[0].N2, eng[1].N2 };
        starts.update(dt, n2, air, fuel);
        for (int s = 0; s < 2; ++s) {
            eng[s].N2 = n2[s];
            thrust[s] = starts.engine[s].starting() ? 0.0
                : ENGINE::engine_dynamics(eng[s], 0.0, 0.0, 0.0, dt, starts.engine[s].running() && fuel[s], ground);
        }
        t += dt;
    }
    template <class F> double until(F done, double limitS = 150.0) {
        const double t0 = t;
        while (!done()) { if (t - t0 > limitS) return -1; step(0.02); }
        return t - t0;
    }
    int phase(int s) const { return starts.engine[s].phase; }
    bool running(int s) const { return starts.engine[s].running(); }
    double n2(int s) const { return eng[s].N2; }
    void both() { const double n2s[2] = { eng[0].N2, eng[1].N2 }; starts.request_both(n2s, air); }
    void start(int s) { starts.request_start(s, eng[s].N2, air); }
};

Aircraft cold() { Aircraft a; a.starts.reset(false); return a; }

// The production EGT and oil pressure indications.
namespace F117 {
double ambientTemperature_DegK = 288.15; bool weight_on_wheels = true;
ENGINE::EngineState engines[2]; double engine_egt_C[2] = { NAN, NAN }, engine_oil_psi[2] = { 0, 0 };
}
Starts g_engineStarts;
''' + constants + '\n' + extract('void update_engine_indications(double dt)') + r'''
Aircraft hot(double n2) { Aircraft a; a.starts.reset(true); a.eng[0].N2 = a.eng[1].N2 = n2; return a; }

int main() {
    const double dt = 0.02;
    {   // No start air: nothing turns, and the cockpit is told.
        Aircraft a = cold(); a.air = false;
        a.both();
        assert(a.phase(LEFT) == OFF && a.phase(RIGHT) == OFF && a.starts.rejected == 2);
        for (int i = 0; i < 500; ++i) a.step(dt);
        assert(a.n2(LEFT) == 0 && a.n2(RIGHT) == 0);
    }
    {   // Engines Start on APU air: the right engine first, the left once the right starter cuts out.
        Aircraft a = cold();
        a.both();
        assert(a.phase(RIGHT) == MOTORING && a.phase(LEFT) == OFF && a.starts.queued[LEFT]);
        const double lightOff = a.until([&] { return a.phase(RIGHT) == LIGHT_OFF; });
        assert(a.n2(RIGHT) >= LIGHT_OFF_N2 && a.n2(RIGHT) < LIGHT_OFF_N2 + 0.1 && a.n2(LEFT) == 0);
        double peak = 0;
        const double cutout = a.until([&] {
            if (a.running(RIGHT)) return true;
            assert(a.thrust[RIGHT] == 0 && a.phase(LEFT) == OFF); peak = (std::max)(peak, a.n2(RIGHT)); return false; });
        assert(peak < STARTER_CUTOUT_N2 && a.n2(RIGHT) < STARTER_CUTOUT_N2 + 0.6);
        assert(a.phase(LEFT) == MOTORING && !a.starts.queued[LEFT]); // the left starter takes over the start air
        const double idle = a.until([&] { return a.n2(RIGHT) > ENGINE::GROUND_IDLE_N2 - 0.5; });
        std::printf("right engine: light-off %.1f s, starter cut-out %.1f s, ground idle %.1f s\n",
            lightOff, lightOff + cutout, lightOff + cutout + idle);
        assert(lightOff > 4 && lightOff < 8 && lightOff + cutout + idle > 20 && lightOff + cutout + idle < 40);
        const double left = a.until([&] { return a.running(LEFT); });
        std::printf("left engine running %.1f s after the right\n", left + idle);
        assert(a.thrust[RIGHT] > 0);
        // The APU can come off once the engines run.
        a.air = false; for (int i = 0; i < 500; ++i) a.step(dt);
        assert(a.running(LEFT) && a.running(RIGHT));
        assert(std::fabs(a.n2(LEFT) - ENGINE::GROUND_IDLE_N2) < 0.5 && std::fabs(a.n2(RIGHT) - ENGINE::GROUND_IDLE_N2) < 0.5);
    }
    {   // Crossbleed: with the right engine running and no other start air, the left starts on bleed from the right.
        Aircraft a = hot(ENGINE::GROUND_IDLE_N2); a.air = false;
        a.starts.request_stop(LEFT);
        a.until([&] { return a.n2(LEFT) == 0; });
        a.start(LEFT);
        assert(a.phase(LEFT) == MOTORING && a.starts.rejected == 0);
        assert(a.until([&] { return a.running(LEFT); }) > 0);
        // Losing the right engine during a crossbleed start before light-off abandons the start.
        a.starts.request_stop(LEFT); a.until([&] { return a.n2(LEFT) == 0; });
        a.start(LEFT); a.step(dt); assert(a.phase(LEFT) == MOTORING);
        a.starts.request_stop(RIGHT); a.step(dt);
        assert(a.phase(LEFT) == OFF);
    }
    {   // Separate starts: a second start waits while the first starter is engaged, even with an air cart.
        Aircraft a = cold();
        a.start(LEFT); a.start(RIGHT);
        assert(a.phase(LEFT) == MOTORING && a.phase(RIGHT) == OFF && a.starts.queued[RIGHT]);
        a.starts.request_stop(RIGHT); assert(!a.starts.queued[RIGHT]); // a stop cancels a waiting start
        a.until([&] { return a.running(LEFT); });
        assert(a.phase(RIGHT) == OFF);
        a.start(RIGHT); assert(a.phase(RIGHT) == MOTORING);
    }
    {   // Start air lost before light-off abandons the start; after light-off the engine carries on.
        Aircraft a = cold();
        a.start(RIGHT);
        a.until([&] { return a.n2(RIGHT) > 8; });
        a.air = false; a.step(dt);
        assert(a.phase(RIGHT) == OFF);
        const double n2 = a.n2(RIGHT); a.step(dt); assert(a.n2(RIGHT) < n2);
        a = cold(); a.start(RIGHT);
        a.until([&] { return a.phase(RIGHT) == ACCELERATING; });
        a.air = false;
        assert(a.until([&] { return a.running(RIGHT); }) > 0);
    }
    {   // A queued start whose start air has gone by the time its turn comes is refused.
        Aircraft a = cold();
        a.both();
        a.until([&] { return a.n2(RIGHT) > 8; });
        a.air = false; a.step(dt);
        assert(a.phase(RIGHT) == OFF && a.phase(LEFT) == OFF && !a.starts.queued[LEFT] && a.starts.rejected == 1);
    }
    {   // No fuel: the starter dry-motors the core but it never lights. Fuel lost once lit is a failed start.
        Aircraft a = cold(); a.fuel[RIGHT] = false;
        a.start(RIGHT);
        for (int i = 0; i < 2000; ++i) a.step(dt);
        assert(a.phase(RIGHT) == MOTORING && a.n2(RIGHT) == STARTER_MAX_N2 && a.thrust[RIGHT] == 0);
        a = cold(); a.start(RIGHT);
        a.until([&] { return a.phase(RIGHT) == ACCELERATING; });
        a.fuel[RIGHT] = false; a.step(dt); assert(a.phase(RIGHT) == OFF);
    }
    {   // One engine flames out in flight; the other keeps running. A windmill relight needs no start air.
        Aircraft a = hot(90); a.ground = false; a.air = false;
        a.fuel[LEFT] = false; a.step(dt);
        assert(a.phase(LEFT) == OFF && a.running(RIGHT));
        a.until([&] { return a.n2(LEFT) < 45; });
        a.fuel[LEFT] = true;
        a.start(LEFT);
        assert(a.phase(LEFT) == LIGHT_OFF && !a.starts.engine[LEFT].starter_engaged() && a.starts.rejected == 0);
        a.until([&] { return a.running(LEFT); });
        assert(a.until([&] { return a.n2(LEFT) > ENGINE::FLIGHT_IDLE_N2 - 0.5; }) > 0);
        // Spooled down too far, the starter is needed; crossbleed from the running engine supplies it.
        a.starts.request_stop(LEFT); a.until([&] { return a.n2(LEFT) < LIGHT_OFF_N2 - 1; });
        a.start(LEFT);
        assert(a.phase(LEFT) == MOTORING && a.starts.engine[LEFT].starter_engaged());
        // With both engines out and no APU, a slow engine can't be started.
        Aircraft b = hot(90); b.ground = false; b.air = false;
        b.starts.request_stop(LEFT); b.starts.request_stop(RIGHT);
        b.until([&] { return b.n2(LEFT) < LIGHT_OFF_N2 - 1; });
        b.both();
        assert(b.phase(LEFT) == OFF && b.phase(RIGHT) == OFF && b.starts.rejected == 2);
    }
    {   // Start while running or starting does nothing; stop during a start abandons it.
        Aircraft a = cold(); a.start(RIGHT);
        a.until([&] { return a.phase(RIGHT) == LIGHT_OFF; });
        a.start(RIGHT); assert(a.phase(RIGHT) == LIGHT_OFF);
        a.starts.request_stop(RIGHT); a.step(dt); assert(a.phase(RIGHT) == OFF && a.thrust[RIGHT] == 0);
        Aircraft h = hot(ENGINE::GROUND_IDLE_N2);
        h.both(); assert(h.running(LEFT) && h.running(RIGHT));
        h.starts.reset(false); assert(h.starts.rejected == 0 && !h.starts.queued[LEFT]);
    }
    {   // Indications through a right engine start, against TO 1F-117A-1 page 2-7.
        Aircraft a = cold(); g_engineStarts = a.starts;
        update_engine_indications(0.02);
        assert(std::fabs(F117::engine_egt_C[RIGHT] - 15.0) < 1e-9 && F117::engine_oil_psi[RIGHT] == 0);
        a.start(RIGHT);
        double egtPeak = 0, oilPeak = 0;
        for (double t = 0; t < 60; t += 0.02) {
            a.step(0.02); g_engineStarts = a.starts;
            F117::engines[RIGHT] = a.eng[RIGHT]; update_engine_indications(0.02);
            egtPeak = (std::max)(egtPeak, F117::engine_egt_C[RIGHT]); oilPeak = (std::max)(oilPeak, F117::engine_oil_psi[RIGHT]);
        }
        std::printf("start EGT peak %.0f C, idle %.0f C; oil peak %.0f psi, idle %.0f psi\n",
            egtPeak, F117::engine_egt_C[RIGHT], oilPeak, F117::engine_oil_psi[RIGHT]);
        assert(egtPeak >= 550 && egtPeak <= 630 && F117::engine_egt_C[RIGHT] >= 350 && F117::engine_egt_C[RIGHT] <= 460);
        assert(oilPeak >= 80 && oilPeak <= 100 && F117::engine_oil_psi[RIGHT] >= 60 && F117::engine_oil_psi[RIGHT] <= 80);
        // After shutdown EGT cools towards ambient and oil pressure falls with the core.
        a.starts.request_stop(RIGHT);
        for (double t = 0; t < 120; t += 0.02) {
            a.step(0.02); g_engineStarts = a.starts;
            F117::engines[RIGHT] = a.eng[RIGHT]; update_engine_indications(0.02);
        }
        assert(F117::engine_egt_C[RIGHT] < 40 && F117::engine_oil_psi[RIGHT] < 5);
    }
    std::puts("PASS: indications, no-air refusal, right-then-left start, APU off after start, crossbleed start and loss, one starter at a time, abandoned and refused starts, dry motoring, failed start, single-engine flameout, windmill relight");
}
'''
code = code.replace('START_HEADER', str(src / 'Engine/EngineStart.h').replace('\\', '/'))
p = out / 'engine_start_tests.cpp'; p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++14 /W4 "{p}" /Fo"{out}/engine_start_tests.obj" /Fe"{out}/engine_start_tests.exe"'
r = subprocess.run(cmd, shell=True, capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
r = subprocess.run([str(out / 'engine_start_tests.exe')], capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
