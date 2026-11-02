# Flight model: measurements out of tolerance

Build `2026-11-02-roll-law-v23`, flown on the flight model bench on 2 November 2026 against TO 1F-117A-1.

**24 of 249 manual tests are out of tolerance.** All 34 added stability checks pass (roll stop, AoA limit, overshoot and load factor swing).

Inertia is `{40000, 260000, 225000}` (roll, yaw, pitch, kg m2).

Difference is model minus manual. For times and fuel, a negative value means quicker or less fuel than the manual. "Over by" is how far the difference goes past the allowed amount.

To regenerate the numbers, run `python tools/fm_bench/scorecard.py` in the mod repo. This file is a snapshot and does not update itself.

## Level acceleration (12 rows, min)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 30000 ft, 36000 lb, to 250 KCAS | 2.10 | 1.56 | -0.54 | -25.8% | ±0.21 | 0.33 |
| 30000 ft, 32000 lb, to 250 KCAS | 1.20 | 0.96 | -0.24 | -20.2% | ±0.12 | 0.12 |
| 30000 ft, 36000 lb, to 300 KCAS | 3.80 | 3.08 | -0.72 | -18.9% | ±0.38 | 0.34 |
| 30000 ft, 32000 lb, to 347 KCAS | 4.30 | 3.57 | -0.73 | -17.0% | ±0.43 | 0.30 |
| 30000 ft, 32000 lb, to 300 KCAS | 2.40 | 2.00 | -0.40 | -16.7% | ±0.24 | 0.16 |
| 30000 ft, 36000 lb, to 347 KCAS | 6.60 | 5.55 | -1.05 | -16.0% | ±0.66 | 0.39 |
| 20000 ft, 40000 lb, to 250 KCAS | 0.70 | 0.56 | -0.14 | -19.7% | ±0.10 | 0.04 |
| 20000 ft, 32000 lb, to 300 KCAS | 0.80 | 0.68 | -0.12 | -15.0% | ±0.10 | 0.02 |
| 10000 ft, 32000 lb, to 495 KCAS | 1.80 | 1.61 | -0.19 | -10.8% | ±0.18 | 0.01 |
| 20000 ft, 32000 lb, to 350 KCAS | 1.20 | 1.08 | -0.12 | -10.4% | ±0.12 | <0.01 |
| 20000 ft, 32000 lb, to 400 KCAS | 1.80 | 1.62 | -0.18 | -10.2% | ±0.18 | <0.01 |
| 20000 ft, 52000 lb, to 424 KCAS | 7.10 | 7.82 | +0.72 | +10.1% | ±0.71 | 0.01 |

## Sustained turn (4 rows, g)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 0 ft, Mach 0.85 | 2.78 | 2.43 | -0.35 | -12.6% | ±0.15 | 0.20 |
| 0 ft, Mach 0.8 | 3.69 | 3.44 | -0.25 | -6.8% | ±0.15 | 0.10 |
| 0 ft, Mach 0.7 | 4.27 | 4.06 | -0.21 | -5.0% | ±0.15 | 0.06 |
| 5000 ft, Mach 0.4 | 2.74 | 2.91 | +0.17 | +6.1% | ±0.15 | 0.02 |

## Top speed, excess power at the limit (2 rows, ft/min)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 42000 lb, 32600 ft, 312 KCAS | 0 | -408 | -408 |  | ±300 | 108 |
| 52000 lb, 27400 ft, 349 KCAS | 0 | -310 | -310 |  | ±300 | 10 |

## Cruise fuel flow (2 rows, lb/hr per engine)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 52000 lb, 285 KCAS, 20000 ft | 3190 | 3649 | +459 | +14.4% | ±319 | 140 |
| 32000 lb, 222 KCAS, 0 ft | 2320 | 2030 | -290 | -12.5% | ±232 | 58 |

## Fuel in acceleration (1 row, lb)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 10000 ft, 32000 lb | 427 | 365 | -62 | -14.5% | ±51 | 11 |

## Climb time (1 row, min)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 52000 lb, to 25000 ft | 8.60 | 9.52 | +0.92 | +10.7% | ±0.86 | 0.06 |

## Angle of attack in level flight (2 rows, deg)

| Condition | Manual | Model | Difference | % | Allowed | Over by |
|---|---:|---:|---:|---:|---:|---:|
| 30000 lb, 143 KCAS, 1000 ft, gear down | 9.50 | 8.83 | -0.67 | -7.1% | ±0.50 | 0.17 |
| 40000 lb, 166 KCAS, 1000 ft, gear down | 9.50 | 8.95 | -0.55 | -5.8% | ±0.50 | 0.05 |
