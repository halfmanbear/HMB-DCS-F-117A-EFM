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

    // TO 00-105E-9 diagram: 9 fuselage tanks split at the baffle plus 2 wing tanks, in proportion to its gallons.
    assert(tank_count == 20);
    const double F = group_capacity_kg(left_forward), D = group_capacity_kg(left_feed);
    const double A = group_capacity_kg(left_aft), W = group_capacity_kg(left_wing);
    assert(near(F, 397 * kg_per_gal) && near(D, 353 * kg_per_gal) && near(A, 288.4 * kg_per_gal) && near(W, 308.8 * kg_per_gal));
    const double manual_lb[] = {2700, 2400, 1900, 2100};
    const double single_box_x[] = {1.85, 1.85, 0.10, 0.10, -1.80, -1.80, -1.50, -1.50};
    for (int g = 0; g < group_count; g += 2) {
        assert(near(group_capacity_kg(g), group_capacity_kg(g + 1)));
        assert(std::fabs(lb(group_capacity_kg(g)) - manual_lb[g / 2]) <= 50);  // Figure 1-7, rounded
    }
    for (int i = 0; i < tank_count; i += 2)  // left and right halves mirror each other
        assert(tanks[i].group + 1 == tanks[i + 1].group && near(tanks[i].capacity_kg, tanks[i + 1].capacity_kg) &&
               tanks[i].x == tanks[i + 1].x && tanks[i].z == -tanks[i + 1].z && tanks[i].z < 0);
    for (int g = 0; g < group_count; ++g) {  // each group's centroid where the single-box model had it
        double m = 0, mx = 0;
        for (const Tank& t : tanks) if (t.group == g) { m += t.capacity_kg; mx += t.capacity_kg * t.x; }
        assert(std::fabs(mx / m - single_box_x[g]) < 0.002);
    }

    double c[tank_count], gc[group_count];
    distribute(total_capacity_kg, c);
    for (int i = 0; i < tank_count; ++i) assert(near(c[i], tanks[i].capacity_kg));
    distribute(0, c);
    for (double v : c) assert(near(v, 0));
    distribute(-5, c);
    for (double v : c) assert(near(v, 0));
    distribute(std::numeric_limits<double>::quiet_NaN(), c);
    for (int i = 0; i < tank_count; ++i) assert(near(c[i], tanks[i].capacity_kg));
    distribute_groups(total_capacity_kg + 10, gc);  // more than the tanks hold stays in the feed tanks
    assert(near(gc[left_feed], D + 5) && near(gc[right_feed], D + 5));
    distribute(total_capacity_kg + 10, c);
    for (int i = 0; i < tank_count; ++i)
        assert(tanks[i].group == left_feed || tanks[i].group == right_feed ? c[i] > tanks[i].capacity_kg : near(c[i], tanks[i].capacity_kg));

    // Page 1-11 usage order, at the end of each stage.
    distribute_groups(total_capacity_kg - 2 * W, gc);  // wings empty
    assert(near(gc[left_wing], 0) && near(gc[right_wing], 0) && near(gc[left_forward], F) && near(gc[right_aft], A));
    distribute_groups(total_capacity_kg - 2 * W - 2 * A, gc);  // left forward and right aft until the right aft empties
    assert(near(gc[right_aft], 0) && near(gc[left_forward], F - A) && near(gc[right_forward], F) && near(gc[left_aft], A));
    distribute_groups(2 * A + 2 * D, gc);  // both forward until the left forward empties
    assert(near(gc[left_forward], 0) && near(gc[right_forward], A) && near(gc[left_aft], A));
    distribute(2 * D, c);  // right forward and left aft until both empty: only the feed tanks remain
    for (int i = 0; i < tank_count; ++i)
        assert(tanks[i].group == left_feed || tanks[i].group == right_feed ? near(c[i], tanks[i].capacity_kg) : near(c[i], 0));

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
