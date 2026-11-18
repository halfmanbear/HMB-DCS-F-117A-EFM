"""Checks the laser-guided bomb time-of-flight model in Weapons/LgbBallistics.h.

Compiles a small test against the header (needs Visual Studio 2022) and checks:
- the drag calibration reproduces each store's char_time,
- the drag curve is continuous where its pieces meet,
- a GBU-12 dropped level from 4343 m above the target at 230 m/s falls ballistically in about 30.9 s, which
  matches the stock F-16C CCRP prediction for the same drop (30.6-31.1 s),
- the full prediction is within 1.8 s of every drop measured in DCS (F-117 drops from
  tools/missions/store_drop_test.py, and the F-16C drop, which the fit did not use, within 0.5 s).
"""
from pathlib import Path
import subprocess
import tempfile

src = Path(__file__).resolve().parents[1] / 'EFM Source Code'
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include "Weapons/LgbBallistics.h"
using namespace Weapons::Lgb_ballistics;
int main()
{
    for (int i = 0; i < store_count; ++i) {
        const Store& s = stores[i];
        const double scale = calibrate_drag_scale(s);
        const double t = fall_time(s, scale, standard_atmosphere, 2000.0, 0.0, 40.0, 0.0);
        std::printf("%s drag scale %.3f, char_time %.3f (want %.2f)\n", s.name, scale, t, s.char_time_s);
        assert(std::fabs(t - s.char_time_s) < 0.01 && scale > 0.2 && scale < 2.0);
        for (int edge = 2; edge <= 4; edge += 2) {
            const double m = s.cx_coeff[edge];
            assert(std::fabs(drag_coefficient(s.cx_coeff, m - 1e-7) - drag_coefficient(s.cx_coeff, m + 1e-7)) < 1e-4);
        }
    }
    assert(find_store("GBU-12") == 0 && find_store("MK-82") == -1 && find_store(nullptr) == -1);

    // F-16C test drop: 4500 m MSL to a target at 157 m, 230 m/s level, 10.4 C and 720 mmHg at sea level.
    const double t0 = 283.55, p0 = 720.09 * 133.322;
    const Atmosphere mission{0.0, t0, p0 / (287.05 * t0)};
    const double fall12 = fall_time(stores[0], calibrate_drag_scale(stores[0]), mission, 4500.0, 157.0, 230.0, 0.0);
    std::printf("GBU-12 F-16C drop, ballistic fall: %.2f s (offline model 30.90, F-16C 30.6-31.1)\n", fall12);
    assert(std::fabs(fall12 - 30.90) < 0.05);
    const double gbu12 = time_of_flight(find_store("GBU-12"), mission, 4500.0, 157.0, 230.0, 0.0, 0.0, 0.0);
    std::printf("GBU-12 F-16C drop, prediction: %.2f s (real 32.67)\n", gbu12);
    assert(std::fabs(gbu12 - 32.67) < 0.5);

    // The same air described from the aircraft's height gives the same answer.
    double rho = 0.0, sound = 0.0;
    air_at(mission, 4500.0, &rho, &sound);
    const Atmosphere from_aircraft{4500.0, t0 - 0.0065 * 4500.0, rho};
    assert(std::fabs(time_of_flight(0, from_aircraft, 4500.0, 157.0, 230.0, 0.0, 0.0, 0.0) - gbu12) < 1e-6);

    // F-117 drops measured in DCS: store, height above target, release speed, release range, real flight time.
    struct Drop { const char* store; double drop, speed, range, real; };
    const Drop drops[] = {
        {"GBU-12", 1280, 160, 2990, 20.27}, {"GBU-12", 1294, 250, 4649, 21.88}, {"GBU-12", 7281, 180, 7830, 45.95},
        {"GBU-12", 7302, 260, 11367, 51.21}, {"GBU-12", 4295, 230, 6134, 32.58}, {"GBU-12", 7302, 260, 9142, 44.44},
        {"GBU-12", 4296, 220, 7800, 37.82}, {"GBU-12", 2789, 192, 5484, 30.05},
        {"GBU-27", 1279, 160, 2982, 23.35}, {"GBU-27", 7300, 260, 11329, 51.73}, {"GBU-27", 4296, 220, 7800, 41.70},
        {"GBU-10", 1279, 160, 2982, 20.05}, {"GBU-10", 7299, 260, 11320, 51.56}, {"GBU-10", 4296, 220, 7800, 39.23},
    };
    for (const Drop& d : drops) {
        const double t = time_of_flight(find_store(d.store), standard_atmosphere, d.drop, 0.0, d.speed, 0.0, d.range, 0.0);
        std::printf("%s %4.0f m %3.0f m/s %5.0f m: %.2f s (DCS %.2f, %+.2f)\n", d.store, d.drop, d.speed, d.range, t,
                    d.real, t - d.real);
        assert(std::fabs(t - d.real) < 1.8);
    }
    // Short of the ballistic range only the guided share is added; a tailwind carries the bomb farther.
    double d = 0.0;
    const double t12 = fall_time(stores[0], calibrate_drag_scale(stores[0]), standard_atmosphere, 4296.0, 0.0, 220.0, 0.0, &d);
    assert(std::fabs(time_of_flight(0, standard_atmosphere, 4296.0, 0.0, 220.0, 0.0, d - 500.0, 0.0) -
                     t12 * (1.0 + stores[0].guided_time_share)) < 1e-9);
    assert(time_of_flight(0, standard_atmosphere, 4296.0, 0.0, 220.0, 0.0, 7800.0, 20.0) <
           time_of_flight(0, standard_atmosphere, 4296.0, 0.0, 220.0, 0.0, 7800.0, 0.0));

    // Climbing at release lengthens the flight; no fall when below the target.
    assert(time_of_flight(0, mission, 4500.0, 157.0, 230.0, 20.0, 0.0, 0.0) > gbu12 + 1.0);
    assert(time_of_flight(0, mission, 100.0, 157.0, 230.0, 0.0, 0.0, 0.0) < 0.0);
    assert(time_of_flight(7, mission, 4500.0, 157.0, 230.0, 0.0, 0.0, 0.0) < 0.0);
    std::puts("PASS: LGB time of flight");
}
'''
out = Path(tempfile.mkdtemp(prefix='lgb_tof_'))
p = out / 'lgb_tof_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = (f'call "{vc}" >nul && cl /nologo /EHsc /std:c++14 /W4 /O2 /I"{src}" '
           f'"{p}" /Fo"{out}/lgb_tof_tests.obj" /Fe"{out}/lgb_tof_tests.exe"')
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'lgb_tof_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
