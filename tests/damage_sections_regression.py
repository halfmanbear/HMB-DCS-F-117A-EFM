"""Exercise the production EFM damage-section mapping and its derived values."""
from pathlib import Path
import re
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402

repo = Path(__file__).resolve().parents[1]
source = load_source()
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)


def extract(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


constants = '\n'.join(re.findall(r'constexpr (?:int damage_element_\w+|double (?:outer_wing_lift_share|'
                                 r'structural_drag_penalty|fire_damage_threshold|damage_cell_destroyed_integrity)) = [^;]+;', source))
code = r"""
#include <algorithm>
#include <cassert>
#include <cstdio>
using std::min;
namespace F117 {
DAMAGE_STATE;
}
F117::Damage_state g_damage;
CONSTANTS
int fires[9] = {};
static void push_fire_event(int handle, double, double, double) { ++fires[handle]; }
static double limit(double value, double low, double high) { return std::max(low, std::min(value, high)); }
UPDATE_DERIVED
INTEGRITY_HELPER
WING_HELPER
APPLY_DAMAGE
NEEDS_REPAIR
static void reset() { g_damage = F117::Damage_state(); std::fill(fires, fires + 9, 0); }
int main() {
    // DCS's lowest report, in float and double form, means destroyed; anything above it does not.
    assert(damage_integrity_from_dcs(0.1) == 0.0 && damage_integrity_from_dcs(0.1f) == 0.0);
    assert(damage_integrity_from_dcs(0.1016) == 0.1016 && damage_integrity_from_dcs(1.5) == 1.0);
    assert(damage_integrity_from_dcs(-1.0) == 0.0);

    // Each engine is damaged independently and fires only its own emitter.
    reset(); apply_damage_for_element(damage_element_engine_left, 0.2);
    assert(g_damage.left_engine == 0.2 && g_damage.right_engine == 1.0 && fires[8] == 1 && fires[7] == 0);
    reset(); apply_damage_for_element(damage_element_engine_right, 0.2);
    assert(g_damage.right_engine == 0.2 && g_damage.left_engine == 1.0 && fires[7] == 1 && fires[8] == 0);

    // Fuselage hits no longer reduce wing lift.
    reset(); apply_damage_for_element(damage_element_fuselage_right, 0.0);
    assert(g_damage.right_fuselage == 0.0 && g_damage.left_wing == 1.0 && g_damage.right_wing == 1.0);

    // Losing an outer panel costs its share of lift and its elevon only.
    reset(); apply_damage_for_element(damage_element_wing_outer_left, 0.0);
    assert(g_damage.left_wing == 1.0 - outer_wing_lift_share && g_damage.right_wing == 1.0);
    assert(g_damage.left_outer_elevon == 0.0 && g_damage.left_inner_elevon == 1.0);

    // Losing an inner panel loses the wing and both elevons, on that side only.
    reset(); apply_damage_for_element(damage_element_wing_inner_right, 0.0);
    assert(g_damage.right_wing == 0.0 && g_damage.right_wing_outer == 0.0);
    assert(g_damage.right_inner_elevon == 0.0 && g_damage.right_outer_elevon == 0.0);
    assert(g_damage.left_wing == 1.0 && g_damage.left_inner_elevon == 1.0);

    // Damage stays monotonic through later, smaller reports.
    reset(); apply_damage_for_element(damage_element_wing_outer_left, 0.5);
    const double wing = g_damage.left_wing;
    apply_damage_for_element(damage_element_wing_outer_left, 0.9);
    assert(g_damage.left_wing_outer == 0.5 && g_damage.left_wing == wing);

    // A destroyed tail section takes its ruddervator; partial damage does not.
    reset(); apply_damage_for_element(damage_element_tail_left, 0.5);
    assert(g_damage.left_tail_structure == 0.5 && g_damage.left_tail == 1.0);
    apply_damage_for_element(damage_element_tail_left, 0.0);
    assert(g_damage.left_tail == 0.0 && g_damage.right_tail == 1.0);

    // Drag grows with structural damage only, up to the full penalty.
    reset(); update_derived_damage_values(); assert(g_damage.structural_drag_factor == 1.0);
    apply_damage_for_element(damage_element_nose_wheel, 0.0); update_derived_damage_values();
    assert(g_damage.structural_drag_factor == 1.0);
    for (int cell : {damage_element_nose, damage_element_fuselage_left, damage_element_fuselage_right, damage_element_belly,
                     damage_element_tail_left, damage_element_tail_right, damage_element_wing_inner_left,
                     damage_element_wing_inner_right, damage_element_wing_outer_left, damage_element_wing_outer_right})
        apply_damage_for_element(cell, 0.0);
    update_derived_damage_values();
    assert(g_damage.structural_drag_factor == 1.0 + structural_drag_penalty);

    // Every section is named in the damage log and requests repair.
    for (int cell : {damage_element_nose, damage_element_cockpit, damage_element_fuselage_left, damage_element_fuselage_right,
                     damage_element_belly, damage_element_engine_left, damage_element_engine_right, damage_element_tail_left,
                     damage_element_tail_right, damage_element_wing_inner_left, damage_element_wing_inner_right,
                     damage_element_wing_outer_left, damage_element_wing_outer_right, damage_element_nose_gear,
                     damage_element_left_gear, damage_element_right_gear, damage_element_nose_wheel,
                     damage_element_left_wheel, damage_element_right_wheel, damage_element_aileron_left,
                     damage_element_aileron_right, damage_element_flap_left, damage_element_flap_right,
                     damage_element_rudder_left, damage_element_rudder_right}) {
        reset(); assert(!ed_fm_need_to_be_repaired());
        const char* mapped = apply_damage_for_element(cell, 0.5);
        assert(mapped[0] != 'I');
        assert(ed_fm_need_to_be_repaired());
    }
    reset(); assert(apply_damage_for_element(100, 0.5)[0] == 'I');
    puts("PASS: independent engines, wing panels, tail sections, structural drag, monotonic damage and repair requests");
}
"""
code = (code.replace('DAMAGE_STATE', extract('struct Damage_state'))
        .replace('CONSTANTS', constants)
        .replace('UPDATE_DERIVED', extract('void update_derived_damage_values()'))
        .replace('INTEGRITY_HELPER', extract('double damage_integrity_from_dcs('))
        .replace('WING_HELPER', extract('void update_wing_from_sections('))
        .replace('APPLY_DAMAGE', extract('const char* apply_damage_for_element('))
        .replace('NEEDS_REPAIR', extract('bool ed_fm_need_to_be_repaired()')))
p = out / 'damage_sections_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/damage_sections_tests.obj" /Fe"{out}/damage_sections_tests.exe"'
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'damage_sections_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
