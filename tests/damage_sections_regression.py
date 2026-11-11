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


constants = '\n'.join(re.findall(r'constexpr (?:int kDamageElement\w+|double (?:kOuterWingLiftShare|'
                                 r'kStructuralDragPenalty|kFireDamageThreshold|kDamageCellDestroyedIntegrity)) = [^;]+;', source))
code = r"""
#include <algorithm>
#include <cassert>
#include <cstdio>
using std::min;
namespace F117 {
DAMAGE_STATE;
}
F117::DamageState g_damage;
CONSTANTS
int fires[9] = {};
static void pushFireEvent(int handle, double, double, double) { ++fires[handle]; }
static double limit(double value, double low, double high) { return std::max(low, std::min(value, high)); }
UPDATE_DERIVED
INTEGRITY_HELPER
WING_HELPER
APPLY_DAMAGE
NEEDS_REPAIR
static void reset() { g_damage = F117::DamageState(); std::fill(fires, fires + 9, 0); }
int main() {
    // DCS's lowest report, in float and double form, means destroyed; anything above it does not.
    assert(damage_integrity_from_dcs(0.1) == 0.0 && damage_integrity_from_dcs(0.1f) == 0.0);
    assert(damage_integrity_from_dcs(0.1016) == 0.1016 && damage_integrity_from_dcs(1.5) == 1.0);
    assert(damage_integrity_from_dcs(-1.0) == 0.0);

    // Each engine is damaged independently and fires only its own emitter.
    reset(); apply_damage_for_element(kDamageElementEngineLeft, 0.2);
    assert(g_damage.leftEngine == 0.2 && g_damage.rightEngine == 1.0 && fires[8] == 1 && fires[7] == 0);
    reset(); apply_damage_for_element(kDamageElementEngineRight, 0.2);
    assert(g_damage.rightEngine == 0.2 && g_damage.leftEngine == 1.0 && fires[7] == 1 && fires[8] == 0);

    // Fuselage hits no longer reduce wing lift.
    reset(); apply_damage_for_element(kDamageElementFuselageRight, 0.0);
    assert(g_damage.rightFuselage == 0.0 && g_damage.leftWing == 1.0 && g_damage.rightWing == 1.0);

    // Losing an outer panel costs its share of lift and its elevon only.
    reset(); apply_damage_for_element(kDamageElementWingOuterLeft, 0.0);
    assert(g_damage.leftWing == 1.0 - kOuterWingLiftShare && g_damage.rightWing == 1.0);
    assert(g_damage.leftOuterElevon == 0.0 && g_damage.leftInnerElevon == 1.0);

    // Losing an inner panel loses the wing and both elevons, on that side only.
    reset(); apply_damage_for_element(kDamageElementWingInnerRight, 0.0);
    assert(g_damage.rightWing == 0.0 && g_damage.rightWingOuter == 0.0);
    assert(g_damage.rightInnerElevon == 0.0 && g_damage.rightOuterElevon == 0.0);
    assert(g_damage.leftWing == 1.0 && g_damage.leftInnerElevon == 1.0);

    // Damage stays monotonic through later, smaller reports.
    reset(); apply_damage_for_element(kDamageElementWingOuterLeft, 0.5);
    const double wing = g_damage.leftWing;
    apply_damage_for_element(kDamageElementWingOuterLeft, 0.9);
    assert(g_damage.leftWingOuter == 0.5 && g_damage.leftWing == wing);

    // A destroyed tail section takes its ruddervator; partial damage does not.
    reset(); apply_damage_for_element(kDamageElementTailLeft, 0.5);
    assert(g_damage.leftTailStructure == 0.5 && g_damage.leftTail == 1.0);
    apply_damage_for_element(kDamageElementTailLeft, 0.0);
    assert(g_damage.leftTail == 0.0 && g_damage.rightTail == 1.0);

    // Drag grows with structural damage only, up to the full penalty.
    reset(); update_derived_damage_values(); assert(g_damage.structuralDragFactor == 1.0);
    apply_damage_for_element(kDamageElementNoseWheel, 0.0); update_derived_damage_values();
    assert(g_damage.structuralDragFactor == 1.0);
    for (int cell : {kDamageElementNose, kDamageElementFuselageLeft, kDamageElementFuselageRight, kDamageElementBelly,
                     kDamageElementTailLeft, kDamageElementTailRight, kDamageElementWingInnerLeft,
                     kDamageElementWingInnerRight, kDamageElementWingOuterLeft, kDamageElementWingOuterRight})
        apply_damage_for_element(cell, 0.0);
    update_derived_damage_values();
    assert(g_damage.structuralDragFactor == 1.0 + kStructuralDragPenalty);

    // Every section is named in the damage log and requests repair.
    for (int cell : {kDamageElementNose, kDamageElementCockpit, kDamageElementFuselageLeft, kDamageElementFuselageRight,
                     kDamageElementBelly, kDamageElementEngineLeft, kDamageElementEngineRight, kDamageElementTailLeft,
                     kDamageElementTailRight, kDamageElementWingInnerLeft, kDamageElementWingInnerRight,
                     kDamageElementWingOuterLeft, kDamageElementWingOuterRight, kDamageElementNoseGear,
                     kDamageElementLeftGear, kDamageElementRightGear, kDamageElementNoseWheel,
                     kDamageElementLeftWheel, kDamageElementRightWheel, kDamageElementAileronLeft,
                     kDamageElementAileronRight, kDamageElementFlapLeft, kDamageElementFlapRight,
                     kDamageElementRudderLeft, kDamageElementRudderRight}) {
        reset(); assert(!ed_fm_need_to_be_repaired());
        const char* mapped = apply_damage_for_element(cell, 0.5);
        assert(mapped[0] != 'I');
        assert(ed_fm_need_to_be_repaired());
    }
    reset(); assert(apply_damage_for_element(100, 0.5)[0] == 'I');
    puts("PASS: independent engines, wing panels, tail sections, structural drag, monotonic damage and repair requests");
}
"""
code = (code.replace('DAMAGE_STATE', extract('struct DamageState'))
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
