"""Pilot pitch trim against TO 1F-117A-1 page 1-46, using the production trim code.

Autotrim (gear up, A/R closed): +1.0 g nose-up and -0.4 g nose-down, slow at neutral, 20 to 25 s end to end.
Autotrim off: a conventional elevon trim, 6 to 9 s end to end.
"""
from pathlib import Path
import re
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import extract, load_source  # noqa: E402

out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = load_source('FlightModel')

constants = '\n'.join(re.findall(r'^constexpr double (?:autotrim|manual_trim)_\w+ = [^;]+;', source, re.M))
code = r"""
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
namespace F117 {
template <typename T> T limit(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
double pitch_trim_position = 0, pitch_trim_elevon_deg = 0;
int pitch_trim_direction = 0;
""" + constants + '\n' + extract(source, 'double autotrim_nz(') + '\n' + extract(source, 'void update_pitch_trim(') + r"""
}
using namespace F117;
// Hold the switch one way from the far stop and time the travel to the other stop.
double travel_s(bool autotrim, int direction)
{
    pitch_trim_position = -direction;
    pitch_trim_direction = direction;
    const double dt = 1.0 / 400.0;
    double t = 0;
    while (std::fabs(pitch_trim_position - direction) > 1e-9 && t < 100) { update_pitch_trim(dt, autotrim); t += dt; }
    pitch_trim_direction = 0;
    return t;
}
int main() {
    const double up_to_down = travel_s(true, -1), down_to_up = travel_s(true, 1);
    std::printf("autotrim travel %.1f s / %.1f s, manual travel %.1f s\n", up_to_down, down_to_up, travel_s(false, -1));
    assert(up_to_down >= 20 && up_to_down <= 25 && down_to_up >= 20 && down_to_up <= 25);
    assert(travel_s(false, -1) >= 6 && travel_s(false, -1) <= 9);
    // Authority: +1.0 g nose-up, -0.4 g nose-down.
    assert(std::fabs(autotrim_nz(1) - 1.0) < 1e-12 && std::fabs(autotrim_nz(-1) + 0.4) < 1e-12);
    // Slow at neutral, faster with more trim.
    pitch_trim_position = 0; pitch_trim_direction = 1; update_pitch_trim(0.1, true);
    const double near_neutral = autotrim_nz(pitch_trim_position);
    pitch_trim_position = 0.8; update_pitch_trim(0.1, true);
    const double far = autotrim_nz(pitch_trim_position) - 0.8;
    assert(near_neutral > 0 && far > 2 * near_neutral);
    // No movement once released; no elevon bias in autotrim; nose-up trim is trailing edge up when it is off.
    pitch_trim_direction = 0; pitch_trim_position = 0.5; pitch_trim_elevon_deg = 0;
    for (int i = 0; i < 4000; ++i) update_pitch_trim(1.0 / 400.0, true);
    assert(pitch_trim_position == 0.5 && pitch_trim_elevon_deg == 0);
    update_pitch_trim(0.05, false);
    assert(pitch_trim_elevon_deg < 0 && pitch_trim_elevon_deg > -0.5 * manual_trim_elevon_deg);  // eased in
    for (int i = 0; i < 4000; ++i) update_pitch_trim(1.0 / 400.0, false);
    assert(std::fabs(pitch_trim_elevon_deg + 0.5 * manual_trim_elevon_deg) < 1e-9);
    puts("PASS: pitch trim authority, rates and travel times match TO 1F-117A-1 page 1-46");
}
"""
p = out / 'pitch_trim_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/pitch_trim_tests.obj" /Fe"{out}/pitch_trim_tests.exe"'
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'pitch_trim_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
