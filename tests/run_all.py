"""Run every regression test and print a PASS/FAIL/SKIP summary.

Usage: python tests/run_all.py [output_dir]

Environment:
  F117_MOD_DIR  path to the companion aircraft mod repository (enables the
                irads_*, terrain_lock and weapon_delivery tests)
  F117_API_DIR  path to the DCS API directory (enables standalone_provenance)
"""
import os
import subprocess
import sys
from pathlib import Path

here = Path(__file__).resolve().parent
out = Path(sys.argv[1] if len(sys.argv) > 1 else here.parent / "build" / "tests").resolve()
out.mkdir(parents=True, exist_ok=True)
mod = os.environ.get("F117_MOD_DIR")
api = os.environ.get("F117_API_DIR")

# name -> (argument builder, requirement description or None)
OUT_ONLY = lambda n: [str(out / n)]
MOD_OUT = lambda n: [mod, str(out / n)]
TESTS = [
    ("autothrottle_regression.py", OUT_ONLY, None),
    ("damage_sections_regression.py", OUT_ONLY, None),
    ("efm_adapters_regression.py", OUT_ONLY, None),
    ("engine_start_regression.py", OUT_ONLY, None),
    ("lerx_vortex_regression.py", OUT_ONLY, None),
    ("paars_regression.py", OUT_ONLY, None),
    ("release_zone_regression.py", OUT_ONLY, None),
    ("simulation_state_regression.py", OUT_ONLY, None),
    ("jdam_envelope_regression.py", lambda n: [], None),
    ("irads_input_regression.py", MOD_OUT, "F117_MOD_DIR"),
    ("irads_tracking_store_regression.py", MOD_OUT, "F117_MOD_DIR"),
    ("terrain_lock_regression.py", MOD_OUT, "F117_MOD_DIR"),
    ("weapon_delivery_regression.py", MOD_OUT, "F117_MOD_DIR"),
    ("standalone_provenance.py", lambda n: [api, str(out / n)], "F117_API_DIR"),
]

results = []
for script, build_args, requires in TESTS:
    name = script[:-3]
    if requires and not os.environ.get(requires):
        results.append((script, "SKIP", f"{requires} not set"))
        continue
    cmd = [sys.executable, str(here / script)] + build_args(name)
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=here.parent)
    tail = (proc.stdout + proc.stderr).strip().splitlines()
    summary = tail[-1] if tail else ""
    if proc.returncode == 0:
        status = "SKIP" if summary.upper().startswith("SKIP") else "PASS"
    else:
        status = "FAIL"
    results.append((script, status, summary[:160]))

width = max(len(r[0]) for r in results)
for script, status, summary in results:
    print(f"{status:4}  {script:<{width}}  {summary}")
failed = [r for r in results if r[1] == "FAIL"]
print(f"\n{sum(r[1] == 'PASS' for r in results)} passed, {len(failed)} failed, "
      f"{sum(r[1] == 'SKIP' for r in results)} skipped")
sys.exit(1 if failed else 0)
