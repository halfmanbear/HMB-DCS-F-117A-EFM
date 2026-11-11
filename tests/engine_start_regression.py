"""Main engine start: start air from the APU, an air cart or crossbleed, one starter at a time, light-off,
starter cut-out, relight, and abandoned and failed starts, on two engines."""
from pathlib import Path
import re, subprocess, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402
repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
src = repo / 'EFM Source Code'
engine = '#include "' + (src / 'Engine/Engine.h').as_posix() + '"\n'
source = load_source()


def extract(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


constants = '\n'.join(re.search(r'constexpr double ' + name + r' = [^;]+;', source).group()
                      for name in ['idle_egt_celsius', 'max_egt_celsius', 'start_egt_celsius', 'egt_heat_tau', 'egt_cool_tau',
                                   'idle_oil_psi', 'max_oil_psi', 'start_oil_psi', 'oil_tau'])
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>

''' + engine + r'''
#include "START_HEADER"
using namespace F117;
using namespace F117::Engine_start;

// The same order as update_propulsion_and_control_effectiveness: each sequence owns its n2 while starting.
struct Aircraft {
    Starts starts; Engine::Engine_state eng[2]; bool air = true, fuel[2] = { true, true }, ground = true;
    double thrust[2] = { 0, 0 }, t = 0;
    void step(double dt) {
        double n2[2] = { eng[0].n2, eng[1].n2 };
        starts.update(dt, n2, air, fuel);
        for (int s = 0; s < 2; ++s) {
            eng[s].n2 = n2[s];
            thrust[s] = starts.engine[s].starting() ? 0.0
                : Engine::engine_dynamics(eng[s], 0.0, 0.0, 0.0, dt, starts.engine[s].running() && fuel[s], ground);
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
    double n2(int s) const { return eng[s].n2; }
    void both() { const double n2s[2] = { eng[0].n2, eng[1].n2 }; starts.request_both(n2s, air); }
    void start(int s) { starts.request_start(s, eng[s].n2, air); }
};

Aircraft cold() { Aircraft a; a.starts.reset(false); return a; }

// The production EGT and oil pressure indications.
namespace F117 {
double ambient_temperature_deg_k = 288.15; bool weight_on_wheels = true;
Engine::Engine_state engines[2]; double engine_egt_c[2] = { NAN, NAN }, engine_oil_psi[2] = { 0, 0 };
}
Starts g_engine_starts;
''' + constants + '\n' + extract('void update_engine_indications(double dt)') + r'''
Aircraft hot(double n2) { Aircraft a; a.starts.reset(true); a.eng[0].n2 = a.eng[1].n2 = n2; return a; }

int main() {
    const double dt = 0.02;
    {   // No start air: nothing turns, and the cockpit is told.
        Aircraft a = cold(); a.air = false;
        a.both();
        assert(a.phase(left) == phase_off && a.phase(right) == phase_off && a.starts.rejected == 2);
        for (int i = 0; i < 500; ++i) a.step(dt);
        assert(a.n2(left) == 0 && a.n2(right) == 0);
    }
    {   // Engines Start on APU air: the right engine first, the left once the right starter cuts out.
        Aircraft a = cold();
        a.both();
        assert(a.phase(right) == phase_motoring && a.phase(left) == phase_off && a.starts.queued[left]);
        const double lightOff = a.until([&] { return a.phase(right) == phase_light_off; });
        assert(a.n2(right) >= light_off_n2 && a.n2(right) < light_off_n2 + 0.1 && a.n2(left) == 0);
        double peak = 0;
        const double cutout = a.until([&] {
            if (a.running(right)) return true;
            assert(a.thrust[right] == 0 && a.phase(left) == phase_off); peak = (std::max)(peak, a.n2(right)); return false; });
        assert(peak < starter_cutout_n2 && a.n2(right) < starter_cutout_n2 + 0.6);
        assert(a.phase(left) == phase_motoring && !a.starts.queued[left]); // the left starter takes over the start air
        const double idle = a.until([&] { return a.n2(right) > Engine::ground_idle_n2 - 0.5; });
        std::printf("right engine: light-off %.1f s, starter cut-out %.1f s, ground idle %.1f s\n",
            lightOff, lightOff + cutout, lightOff + cutout + idle);
        assert(lightOff > 4 && lightOff < 8 && lightOff + cutout + idle > 20 && lightOff + cutout + idle < 40);
        const double left_delay = a.until([&] { return a.running(left); });
        std::printf("left engine running %.1f s after the right\n", left_delay + idle);
        assert(a.thrust[right] > 0);
        // The APU can come off once the engines run.
        a.air = false; for (int i = 0; i < 500; ++i) a.step(dt);
        assert(a.running(left) && a.running(right));
        assert(std::fabs(a.n2(left) - Engine::ground_idle_n2) < 0.5 && std::fabs(a.n2(right) - Engine::ground_idle_n2) < 0.5);
    }
    {   // Crossbleed: with the right engine running and no other start air, the left starts on bleed from the right.
        Aircraft a = hot(Engine::ground_idle_n2); a.air = false;
        a.starts.request_stop(left);
        a.until([&] { return a.n2(left) == 0; });
        a.start(left);
        assert(a.phase(left) == phase_motoring && a.starts.rejected == 0);
        assert(a.until([&] { return a.running(left); }) > 0);
        // Losing the right engine during a crossbleed start before light-off abandons the start.
        a.starts.request_stop(left); a.until([&] { return a.n2(left) == 0; });
        a.start(left); a.step(dt); assert(a.phase(left) == phase_motoring);
        a.starts.request_stop(right); a.step(dt);
        assert(a.phase(left) == phase_off);
    }
    {   // Separate starts: a second start waits while the first starter is engaged, even with an air cart.
        Aircraft a = cold();
        a.start(left); a.start(right);
        assert(a.phase(left) == phase_motoring && a.phase(right) == phase_off && a.starts.queued[right]);
        a.starts.request_stop(right); assert(!a.starts.queued[right]); // a stop cancels a waiting start
        a.until([&] { return a.running(left); });
        assert(a.phase(right) == phase_off);
        a.start(right); assert(a.phase(right) == phase_motoring);
    }
    {   // Start air lost before light-off abandons the start; after light-off the engine carries on.
        Aircraft a = cold();
        a.start(right);
        a.until([&] { return a.n2(right) > 8; });
        a.air = false; a.step(dt);
        assert(a.phase(right) == phase_off);
        const double n2 = a.n2(right); a.step(dt); assert(a.n2(right) < n2);
        a = cold(); a.start(right);
        a.until([&] { return a.phase(right) == phase_accelerating; });
        a.air = false;
        assert(a.until([&] { return a.running(right); }) > 0);
    }
    {   // A queued start whose start air has gone by the time its turn comes is refused.
        Aircraft a = cold();
        a.both();
        a.until([&] { return a.n2(right) > 8; });
        a.air = false; a.step(dt);
        assert(a.phase(right) == phase_off && a.phase(left) == phase_off && !a.starts.queued[left] && a.starts.rejected == 1);
    }
    {   // No fuel: the starter dry-motors the core but it never lights. Fuel lost once lit is a failed start.
        Aircraft a = cold(); a.fuel[right] = false;
        a.start(right);
        for (int i = 0; i < 2000; ++i) a.step(dt);
        assert(a.phase(right) == phase_motoring && a.n2(right) == starter_max_n2 && a.thrust[right] == 0);
        a = cold(); a.start(right);
        a.until([&] { return a.phase(right) == phase_accelerating; });
        a.fuel[right] = false; a.step(dt); assert(a.phase(right) == phase_off);
    }
    {   // One engine flames out in flight; the other keeps running. A windmill relight needs no start air.
        Aircraft a = hot(90); a.ground = false; a.air = false;
        a.fuel[left] = false; a.step(dt);
        assert(a.phase(left) == phase_off && a.running(right));
        a.until([&] { return a.n2(left) < 45; });
        a.fuel[left] = true;
        a.start(left);
        assert(a.phase(left) == phase_light_off && !a.starts.engine[left].starter_engaged() && a.starts.rejected == 0);
        a.until([&] { return a.running(left); });
        assert(a.until([&] { return a.n2(left) > Engine::flight_idle_n2 - 0.5; }) > 0);
        // Spooled down too far, the starter is needed; crossbleed from the running engine supplies it.
        a.starts.request_stop(left); a.until([&] { return a.n2(left) < light_off_n2 - 1; });
        a.start(left);
        assert(a.phase(left) == phase_motoring && a.starts.engine[left].starter_engaged());
        // With both engines out and no APU, a slow engine can't be started.
        Aircraft b = hot(90); b.ground = false; b.air = false;
        b.starts.request_stop(left); b.starts.request_stop(right);
        b.until([&] { return b.n2(left) < light_off_n2 - 1; });
        b.both();
        assert(b.phase(left) == phase_off && b.phase(right) == phase_off && b.starts.rejected == 2);
    }
    {   // Start while running or starting does nothing; stop during a start abandons it.
        Aircraft a = cold(); a.start(right);
        a.until([&] { return a.phase(right) == phase_light_off; });
        a.start(right); assert(a.phase(right) == phase_light_off);
        a.starts.request_stop(right); a.step(dt); assert(a.phase(right) == phase_off && a.thrust[right] == 0);
        Aircraft h = hot(Engine::ground_idle_n2);
        h.both(); assert(h.running(left) && h.running(right));
        h.starts.reset(false); assert(h.starts.rejected == 0 && !h.starts.queued[left]);
    }
    {   // Indications through a right engine start, against TO 1F-117A-1 page 2-7.
        Aircraft a = cold(); g_engine_starts = a.starts;
        update_engine_indications(0.02);
        assert(std::fabs(F117::engine_egt_c[right] - 15.0) < 1e-9 && F117::engine_oil_psi[right] == 0);
        a.start(right);
        double egtPeak = 0, oilPeak = 0;
        for (double t = 0; t < 60; t += 0.02) {
            a.step(0.02); g_engine_starts = a.starts;
            F117::engines[right] = a.eng[right]; update_engine_indications(0.02);
            egtPeak = (std::max)(egtPeak, F117::engine_egt_c[right]); oilPeak = (std::max)(oilPeak, F117::engine_oil_psi[right]);
        }
        std::printf("start EGT peak %.0f C, idle %.0f C; oil peak %.0f psi, idle %.0f psi\n",
            egtPeak, F117::engine_egt_c[right], oilPeak, F117::engine_oil_psi[right]);
        assert(egtPeak >= 550 && egtPeak <= 630 && F117::engine_egt_c[right] >= 350 && F117::engine_egt_c[right] <= 460);
        assert(oilPeak >= 80 && oilPeak <= 100 && F117::engine_oil_psi[right] >= 60 && F117::engine_oil_psi[right] <= 80);
        // After shutdown EGT cools towards ambient and oil pressure falls with the core.
        a.starts.request_stop(right);
        for (double t = 0; t < 120; t += 0.02) {
            a.step(0.02); g_engine_starts = a.starts;
            F117::engines[right] = a.eng[right]; update_engine_indications(0.02);
        }
        assert(F117::engine_egt_c[right] < 40 && F117::engine_oil_psi[right] < 5);
    }
    std::puts("PASS: indications, no-air refusal, right-then-left start, APU off after start, crossbleed start and loss, one starter at a time, abandoned and refused starts, dry motoring, failed start, single-engine flameout, windmill relight");
}
'''
code = code.replace('START_HEADER', str(src / 'Engine/EngineStart.h').replace('\\', '/'))
p = out / 'engine_start_tests.cpp'; p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/engine_start_tests.obj" /Fe"{out}/engine_start_tests.exe"'
r = subprocess.run(cmd, shell=True, capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
r = subprocess.run([str(out / 'engine_start_tests.exe')], capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
