"""Track replay designation: compiles the production designate handling and ray choice.

A replayed track delivers the cockpit's relayed designate command (10114) twice: the recorded
copy and the one the cockpit Lua sends again from the replayed TDC press. The EFM must act on
the press edge only, or the second press undesignates. When the IRADS camera has not rendered
recently (replay watched from outside), the designation must use the simulated sensor ray, not
the frozen rendered one.

Usage: python tests/replay_designate_regression.py <output_dir>
"""
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source, extract  # noqa: E402

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
text = load_source('Avionics')
inputs = load_source('Inputs.h')
slew_stop = int(re.search(r'constexpr int irads_slew_stop\s*=\s*(\d+)', inputs).group(1))
assert slew_stop == 10114

case_start = text.index('case irads_slew_stop:')
case_end = text.index('return true;', text.index('stop_slew_and_designate(active);', case_start)) + len('return true;')
designate_case = text[case_start:case_end]
stop_and_designate = extract(text, 'static void stop_slew_and_designate(')
ray_length = extract(text, 'static double ray_direction_length(')
stabilize = extract(text, 'bool ground_stabilize_irads(')

harness = r"""
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
constexpr int irads_slew_stop = SLEW_STOP;
constexpr double irads_ray_max_age = 0.5, rad_to_deg = 57.295779513, min_ray_norm_sq = 0.5;
constexpr unsigned terrain_sequence_wrap = 10000000;
static int sensor_object;
static void* g_av_tv_sensor_instance = &sensor_object;
static bool powered = true;
static bool is_irads_powered() { return powered; }
static bool s_irads_ray_valid = false, s_irads_sensor_ray_valid = false, s_irads_designate_held = false;
static bool s_irads_terrain_pending = false, s_irads_lock_valid = false;
static double s_irads_render_ray[6]{}, s_irads_sensor_ray[6]{}, s_irads_request_ray[6]{};
static double s_irads_ray_age = 1e9, s_irads_terrain_wait = 0;
static unsigned s_irads_terrain_sequence = 0;
static int queued_count = 0, undesignated = 0, ignored = 0;
static void log_format(const char*, ...) {}
static void log_message(const char* m)
{
    if (std::strstr(m, "queued")) ++queued_count;
    if (std::strcmp(m, "IRADS undesignated") == 0) ++undesignated;
    if (std::strstr(m, "repeated press ignored")) ++ignored;
}
static void clear_slew_demand() {}
static void (*g_av_tv_sensor_slew_stop)(void*) = nullptr;
static void set_external_designation(bool) {}
static void clear_irads_lock() { s_irads_lock_valid = false; s_irads_terrain_pending = false; }
RAY_LENGTH
STABILIZE
STOP_AND_DESIGNATE
static bool handle(int command, float value)
{
    const bool active = value > 0.05f || value < -0.05f;
    switch (command)
    {
        DESIGNATE_CASE
        default: return false;
    }
}
static void reset() { queued_count = undesignated = ignored = 0; s_irads_designate_held = false; clear_irads_lock(); }
static void set_ray(double* ray, double ex, double ey, double ez, double dx, double dy, double dz)
{
    const double r[6] = {ex, ey, ez, dx, dy, dz};
    std::memcpy(ray, r, sizeof(r));
}
int main()
{
    // Rendered ray fresh (cockpit view): live single press designates; next press undesignates.
    set_ray(s_irads_render_ray, 0, 3700, 0, 0.37, -0.16, -0.91); s_irads_ray_valid = true; s_irads_ray_age = 0;
    set_ray(s_irads_sensor_ray, 0, 3700, 0, 0.37, -0.17, -0.91); s_irads_sensor_ray_valid = true;
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 1 && undesignated == 0 && s_irads_terrain_pending);
    assert(std::fabs(s_irads_request_ray[4] - (-0.16 / std::sqrt(0.37*0.37 + 0.16*0.16 + 0.91*0.91))) < 1e-12);
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 1 && undesignated == 1 && !s_irads_terrain_pending);
    puts("PASS: live press designates from the rendered ray; next press undesignates");

    // Replay order from the track: Lua re-sent press, recorded press, then both releases.
    reset();
    for (float v : {1.0f, 1.0f, 0.0f, 0.0f}) handle(irads_slew_stop, v);
    assert(queued_count == 1 && undesignated == 0 && ignored == 1 && s_irads_terrain_pending);
    // A later doubled press still undesignates exactly once.
    for (float v : {1.0f, 1.0f, 0.0f, 0.0f}) handle(irads_slew_stop, v);
    assert(queued_count == 1 && undesignated == 1 && ignored == 2 && !s_irads_terrain_pending);
    puts("PASS: doubled replay press designates once and undesignates once");

    // Camera not rendered for longer than the ray age limit: use the simulated sensor ray.
    reset(); s_irads_ray_age = 30.0;
    set_ray(s_irads_render_ray, -228093, 1689, -173787, -0.84, -0.01, -0.54);  // frozen on the ground
    set_ray(s_irads_sensor_ray, -202864, 3676, -190620, 0.369, -0.162, -0.915);
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 1 && s_irads_request_ray[0] == -202864 && s_irads_request_ray[1] == 3676);
    puts("PASS: stale rendered ray is replaced by the sensor ray");

    // No fresh render and no sensor ray: nothing is queued.
    reset(); s_irads_sensor_ray_valid = false;
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 0 && !s_irads_terrain_pending);
    // Invalid sensor ray is refused.
    reset(); s_irads_sensor_ray_valid = true; set_ray(s_irads_sensor_ray, 0, 0, 0, 0, 0, 0);
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 0);
    // Unpowered IRADS refuses.
    reset(); powered = false; s_irads_ray_age = 0;
    handle(irads_slew_stop, 1); handle(irads_slew_stop, 0);
    assert(queued_count == 0);
    puts("PASS: no usable ray or no power queues nothing");
}
"""
for key, value in [('SLEW_STOP', str(slew_stop)), ('RAY_LENGTH', ray_length), ('STABILIZE', stabilize),
                   ('STOP_AND_DESIGNATE', stop_and_designate), ('DESIGNATE_CASE', designate_case)]:
    harness = harness.replace(key, value)
source = out / 'replay_designate_tests.cpp'
source.write_text(harness, encoding='utf-8')
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
command = (f'call "{vcvars}" >nul && cl /nologo /utf-8 /EHsc /std:c++17 /W4 "{source}" '
           f'/Fo"{out / "replay_designate_tests.obj"}" /Fe"{out / "replay_designate_tests.exe"}"')
build = subprocess.run(command, shell=True, text=True, capture_output=True)
print(build.stdout, build.stderr)
assert build.returncode == 0
run = subprocess.run([str(out / 'replay_designate_tests.exe')], text=True, capture_output=True)
print(run.stdout, run.stderr)
assert run.returncode == 0
