from pathlib import Path
import subprocess, sys
repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
src = repo / 'EFM Source Code'
engine = (src / 'Engine/Engine.h').read_text(encoding='utf-8-sig').replace('#include "../stdafx.h"', '')
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
double limit(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
''' + engine + r'''
#include "AT_HEADER"
using namespace Autothrottle;

// Point-mass F-117 along the flight path; ISA atmosphere and a simple drag polar.
struct Sim {
    double altM, tasMps, gammaDeg = 0, throttle = 0, massKg = 23000;
    double temperature() const { return 288.15 - 0.0065 * altM; }
    double pressure() const { return 101325.0 * std::pow(temperature() / 288.15, 5.2559); }
    double density() const { return pressure() / (287.058 * temperature()); }
    double mach() const { return tasMps / std::sqrt(1.4 * 287.058 * temperature()); }
    double cas() const { return calibrated_airspeed_kt(tasMps, temperature(), pressure()); }
    void step(double dt) {
        const double q = 0.5 * density() * tasMps * tasMps, area = 84.8;
        const double cl = massKg * 9.81 * std::cos(gammaDeg * 3.14159265 / 180) / (q * area);
        const double drag = q * area * (0.020 + 0.194 * cl * cl);
        const double thrust = F117::ENGINE::engine_dynamics(throttle, mach(), altM, dt, true, false);
        tasMps += ((thrust - drag) / massKg - 9.81 * std::sin(gammaDeg * 3.14159265 / 180)) * dt;
    }
};

// Throttle that holds a CAS without the A/T, found by bisection on the settled speed.
double trim_throttle(double altM, double casKt) {
    double lo = 0, hi = 100;
    for (int i = 0; i < 40; ++i) {
        Sim s{altM, 0}; double mid = (lo + hi) / 2;
        s.tasMps = casKt / kKnotsPerMps / std::sqrt(s.density() / 1.225); s.throttle = mid;
        for (int k = 0; k < 6000; ++k) s.step(0.02);
        (s.cas() > casKt ? hi : lo) = mid;
    }
    return (lo + hi) / 2;
}

Sim trimmed(double altM, double casKt, Controller& at) {
    Sim s{altM, 0}; s.throttle = trim_throttle(altM, casKt);
    s.tasMps = casKt / kKnotsPerMps / std::sqrt(s.density() / 1.225);
    for (int k = 0; k < 6000; ++k) s.step(0.02);
    assert(at.engage(s.cas(), s.throttle, true, true));
    return s;
}

struct Run { double maxCas = -1e9, minCas = 1e9, finalCas = 0, maxLeverRate = 0; };
Run fly(Sim& s, Controller& at, double seconds, double dt = 0.02) {
    Run r; double previous = s.throttle;
    for (double t = 0; t < seconds; t += dt) {
        s.throttle = at.update(dt, s.cas(), s.throttle, true, true);
        r.maxLeverRate = std::max(r.maxLeverRate, std::fabs(s.throttle - previous) / dt); previous = s.throttle;
        s.step(dt);
        r.maxCas = std::max(r.maxCas, s.cas()); r.minCas = std::min(r.minCas, s.cas());
    }
    r.finalCas = s.cas(); return r;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // CAS: equals TAS at ISA sea level; 300 KCAS is 431.5 KTAS (Mach 0.72) at 25,000 ft.
    assert(std::fabs(calibrated_airspeed_kt(150, 288.15, 101325) - 150 * kKnotsPerMps) < 0.05);
    Sim fl250{7620, 431.5 / kKnotsPerMps};
    assert(std::fabs(fl250.cas() - 300) < 0.5);
    assert(calibrated_airspeed_kt(0, 288.15, 101325) == 0 && calibrated_airspeed_kt(100, 0, 1) == 0);

    // Engage switch: refused on the ground or with engines stopped; CAS target clamped to keypad limits.
    Controller at;
    assert(!at.engage(300, 50, false, true) && !at.engage(300, 50, true, false) && !at.engaged());
    assert(at.engage(100, 40, true, true) && at.target_cas_kt() == kCasMinKt);
    at.reset(); assert(at.engage(600, 40, true, true) && at.target_cas_kt() == kCasMaxKt);

    // Disconnect button / paddle: first press disengages and lights A/T DISC, second extinguishes.
    at.disconnect_button(); assert(!at.engaged() && at.disconnect_light());
    at.disconnect_button(); assert(!at.disconnect_light());
    // Touchdown and engine stop disengage; not engaged leaves the throttle untouched.
    at.engage(300, 40, true, true); assert(at.update(0.02, 300, 40, false, true) == 40 && !at.engaged() && at.disconnect_light());
    at.engage(300, 40, true, true); at.update(0.02, 300, 40, true, false); assert(!at.engaged());
    assert(at.update(0.02, 250, 33, true, true) == 33);

    // Axis arbitration: jitter ignored while engaged, deliberate travel overpowers;
    // after a button disengage the lever is held until the axis moves.
    at.reset(); assert(at.accept_axis(60));
    at.engage(300, 60, true, true);
    assert(!at.accept_axis(63) && at.engaged());
    assert(at.accept_axis(70) && !at.engaged() && at.disconnect_light());
    at.engage(300, 70, true, true); at.disconnect_button();
    assert(!at.accept_axis(71) && at.accept_axis(73) && at.accept_axis(73.5));
    at.engage(300, 50, true, true); at.pilot_throttle_step(); assert(!at.engaged() && at.accept_axis(73.6));

    for (double alt : {0.0, 7620.0}) {
        // Bumpless engage and steady hold.
        at.reset(); Sim s = trimmed(alt, 300, at);
        double lever = s.throttle; Run hold = fly(s, at, 30);
        assert(std::fabs(hold.finalCas - 300) < 1 && hold.maxCas - hold.minCas < 1.5);
        assert(std::fabs(s.throttle - lever) < 3);

        // Climb disturbance within MIL thrust (3 deg at SL, 0.5 deg at 25,000 ft): recovers within 2 kt.
        s.gammaDeg = alt == 0 ? 3 : 0.5; Run climb = fly(s, at, 90);
        std::printf("alt %5.0f climb: min %.1f final %.1f\n", alt, climb.minCas, climb.finalCas);
        assert(std::fabs(climb.finalCas - 300) < 2 && climb.minCas > 285);
        s.gammaDeg = 0; fly(s, at, 90);

        // Speed change up (330 at SL; 310 at 25,000 ft, below the MIL-limited ~318) and down to 270:
        // bounded overshoot, settles, servo rate respected.
        const double upKt = alt == 0 ? 330 : 310;
        at.select_cas(upKt); Run up = fly(s, at, 120); assert(s.throttle < 100);
        at.select_cas(270); Run down = fly(s, at, 120);
        std::printf("alt %5.0f 300->%.0f max %.1f final %.1f | ->270 min %.1f final %.1f | lever rate %.1f %%/s\n",
            alt, upKt, up.maxCas, up.finalCas, down.minCas, down.finalCas, std::max(up.maxLeverRate, down.maxLeverRate));
        assert(up.maxCas < upKt + 5 && std::fabs(up.finalCas - upKt) < 2);
        assert(down.minCas > 265 && std::fabs(down.finalCas - 270) < 2);
        assert(std::max(up.maxLeverRate, down.maxLeverRate) <= kServoRatePctPerS + 1e-6);

        // Unreachable target saturates at MIL without wind-up, and comes off promptly.
        at.select_cas(525); fly(s, at, 60); assert(s.throttle == 100);
        at.select_cas(s.cas() - 10); double t = 0;
        for (; t < 15 && s.throttle >= 99.9; t += 0.02) { s.throttle = at.update(0.02, s.cas(), s.throttle, true, true); s.step(0.02); }
        std::printf("alt %5.0f lever leaves MIL after %.1f s\n", alt, t);
        assert(t < 3);
    }
    std::puts("PASS: CAS conversion, engage/disengage/A/T DISC rules, axis override and hold, bumpless engage, hold, climb disturbance, speed changes, servo rate, saturation without wind-up at SL and 25,000 ft");
}
'''
code = code.replace('AT_HEADER', str(src / 'Avionics/Autothrottle.h').replace('\\', '/'))
p = out / 'autothrottle_tests.cpp'; p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++14 /W4 "{p}" /Fo"{out}/autothrottle_tests.obj" /Fe"{out}/autothrottle_tests.exe"'
r = subprocess.run(cmd, shell=True, capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
r = subprocess.run([str(out / 'autothrottle_tests.exe')], capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
