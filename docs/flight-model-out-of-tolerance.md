# Flight model: measurements out of tolerance

Build `2026-11-02-thrust-table-v25`, flown on the flight model bench on 2 November 2026 against TO 1F-117A-1.

**2 of 249 manual tests are out of tolerance**, down from 24 on the previous build. All 34 added stability checks pass (roll stop, AoA limit, overshoot and load factor swing).

Inertia is `{40000, 260000, 225000}` (roll, yaw, pitch, kg m2).

Difference is model minus manual. For times, a negative value means quicker than the manual. "Over by" is how far the difference goes past the allowed amount.

To regenerate the numbers, run `python tools/fm_bench/scorecard.py` in the mod repo. This file is a snapshot and does not update itself.

## Still out of tolerance

| Test | Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---|---:|---:|---:|---:|---:|---:|
| Level acceleration (min) | 20000 ft, 40000 lb, to 250 KCAS | 0.70 | 0.58 | -0.12 | -17.0% | ±0.10 | 0.02 |
| Sustained turn (g) | 0 ft, Mach 0.4 | 3.26 | 3.42 | +0.16 | +4.8% | ±0.15 | 0.01 |

These two are left as they are. The acceleration row sits between 32,000 lb and 52,000 lb rows at the same altitude, and the manual prints those times to 0.1 min. The turn row is read by eye from a chart that is good to about 0.1 g. Closing either one by more fitting would push neighbouring rows out or fit reading error.

## What changed from the previous build

- Maximum thrust is now a table of altitude and Mach instead of a formula. Thrust at altitude now rises with Mach, which the acceleration and top speed data both needed.
- Fuel flow per unit of thrust now falls with altitude.
- Gear down, lift falls away above 6 deg AOA, which matches the AOA and speed tables on page 2-13.
- The drag polar and idle thrust were refitted, including the row-by-row difference between the fit model and the built EFM.
- The sustained turn test now flies the load factor directly. The earlier test held altitude with the stick, which didn't settle in a steep bank, so some earlier turn results were test error. The manual's figures and the tolerances did not change.
- A turn that can't quite hold 1 g now reports the load factor it can hold, instead of no result.

## Checked in DCS, outside the 249 tests

| Takeoff at 46,000 lb, 66 ft airfield | Manual | DCS |
|---|---:|---:|
| Ground run, ft | 4,700 | 5,025 |
| Distance to clear 50 ft | 6,950 | 6,711 |
