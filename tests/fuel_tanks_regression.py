"""Check the fuel tank model against TO 1F-117A-1: capacities, usage order, and CG inside the limits.

Usage: python tests/fuel_tanks_regression.py <build-directory>
Requires the Visual Studio 2022 C++ tools.
"""
from pathlib import Path
import subprocess
import sys

repo = Path(__file__).resolve().parents[1]
src = repo / 'EFM Source Code'
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)

code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include "Systems/FuelTanks.h"
using namespace F117::Fuel_tanks;

static double lb(double kg) { return kg / kg_per_lb; }
static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

// CG in % MAC, from the aero model's reference: 44.5% MAC at x = 0.078 m, 10.9 m MAC.
static double percent_mac(double x) { return 44.5 + (0.078 - x) / 0.109; }

// Empty aircraft (M_empty at the config CG) plus fuel and an optional payload at the bay pylons.
static void cg(double fuel_kg, double payload_kg, double& x, double& z) {
    double c[tank_count];
    distribute(fuel_kg, c);
    double m = 13381.0, mx = 0.0, mz = 0.0;
    for (int i = 0; i < tank_count; ++i) { m += c[i]; mx += c[i] * tanks[i].x; mz += c[i] * tanks[i].z; }
    m += payload_kg; mx += payload_kg * 0.763;
    x = mx / m; z = mz / m;
}

int main() {
    double sum = 0;
    for (const Tank& t : tanks) sum += t.capacity_kg;
    assert(near(sum, total_capacity_kg) && near(lb(total_capacity_kg), 18200));

    double c[tank_count];
    distribute(total_capacity_kg, c);
    for (int i = 0; i < tank_count; ++i) assert(near(c[i], tanks[i].capacity_kg));
    distribute(0, c);
    for (double v : c) assert(near(v, 0));
    distribute(-5, c);
    for (double v : c) assert(near(v, 0));
    distribute(std::numeric_limits<double>::quiet_NaN(), c);
    for (int i = 0; i < tank_count; ++i) assert(near(c[i], tanks[i].capacity_kg));
    distribute(total_capacity_kg + 10, c);  // more than the tanks hold stays in the feed tanks
    assert(near(c[left_feed], tanks[left_feed].capacity_kg + 5) && near(c[right_feed], tanks[right_feed].capacity_kg + 5));

    // Page 1-11 usage order, at the end of each stage.
    distribute(14000 * kg_per_lb, c);  // wings empty
    assert(near(c[left_wing], 0) && near(c[right_wing], 0) && near(lb(c[left_forward]), 2700) && near(lb(c[right_aft]), 1900));
    distribute(10200 * kg_per_lb, c);  // left forward and right aft until the right aft empties
    assert(near(c[right_aft], 0) && near(lb(c[left_forward]), 800) && near(lb(c[right_forward]), 2700) && near(lb(c[left_aft]), 1900));
    distribute(8600 * kg_per_lb, c);  // both forward until the left forward empties
    assert(near(c[left_forward], 0) && near(lb(c[right_forward]), 1900) && near(lb(c[left_aft]), 1900));
    distribute(4800 * kg_per_lb, c);  // right forward and left aft until both empty: only the feed tanks remain
    for (int i = 0; i < tank_count; ++i)
        assert(i == left_feed || i == right_feed ? near(lb(c[i]), 2400) : near(c[i], 0));

    // Whole range: mass kept, no tank fills as fuel burns, CG inside 41.3 to 46.5% MAC and near the centreline,
    // with and without the 1,982 kg of two GBU-27s.
    double last[tank_count];
    distribute(total_capacity_kg, last);
    double most_aft = 100, most_forward = 0;
    for (double fuel = total_capacity_kg; fuel >= 0; fuel -= 5) {
        distribute(fuel, c);
        double total = 0;
        for (int i = 0; i < tank_count; ++i) { total += c[i]; assert(c[i] <= last[i] + 1e-9 && c[i] >= -1e-9); last[i] = c[i]; }
        assert(std::fabs(total - fuel) < 1e-6);
        for (double payload : {0.0, 1982.0}) {
            double x, z;
            cg(fuel, payload, x, z);
            const double p = percent_mac(x);
            assert(p > 41.3 && p < 46.5 && std::fabs(z) < 0.01);
            if (p < most_aft) most_aft = p;
            if (p > most_forward) most_forward = p;
        }
    }
    printf("PASS: Figure 1-7 capacities, page 1-11 usage order, mass kept, CG %.1f to %.1f%% MAC with and without payload\n",
           most_aft, most_forward);
}
'''
p = out / 'fuel_tanks_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = (f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 /I"{src}" '
           f'"{p}" /Fo"{out}/fuel_tanks_tests.obj" /Fe"{out}/fuel_tanks_tests.exe"')
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'fuel_tanks_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
