from pathlib import Path
import subprocess, sys
repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
src = repo / 'EFM Source Code'
engine = (src / 'Engine/Engine.h').read_text(encoding='utf-8-sig').replace('#include "../stdafx.h"', '')
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
double limit(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
''' + engine + r'''
#include "AT_HEADER"
#include "PAARS_HEADER"
using namespace Paars;

struct V3 { double x, y, z; };
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator*(double s, V3 a) { return {s * a.x, s * a.y, s * a.z}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
V3 unit(V3 a) { return (1.0 / std::sqrt(dot(a, a))) * a; }
const V3 kUp{0, 0, 1};

// Point-mass F-117 (alpha ~ 0, so pitch ~ flight path) with an idealised FCS: load factor and
// roll rate follow their commands with first-order lags; lift is limited by CLmax.
struct Jet {
    V3 x, y, z;  // body axes in a z-up world frame: nose, right wing, floor
    double v, alt, n = 1, p = 0, throttle = 60, mass = 23000;
    Autothrottle::Controller at;
    double temperature() const { return 288.15 - 0.0065 * alt; }
    double pressure() const { return 101325.0 * std::pow(temperature() / 288.15, 5.2559); }
    double density() const { return pressure() / (287.058 * temperature()); }
    double cas() const { return Autothrottle::calibrated_airspeed_kt(v, temperature(), pressure()); }
    double pitch() const { return std::asin(limit(dot(kUp, x), -1, 1)) / kDegToRad; }
    double bank() const { return std::atan2(-dot(kUp, y), -dot(kUp, z)) / kDegToRad; }
    double pitchRate() const { return 9.81 * (n - (-dot(kUp, z))) / v / kDegToRad; }
    Jet(double pitchDeg, double bankDeg, double tas, double altitude) : v(tas), alt(altitude) {
        const double t = pitchDeg * kDegToRad, b = bankDeg * kDegToRad;
        x = {std::cos(t), 0, std::sin(t)};
        V3 y0{0, -1, 0}, z0 = cross(x, y0);  // level wings; floor axis completes the frame
        y = std::cos(b) * y0 + std::sin(b) * z0;
        z = cross(x, y);
        n = -dot(kUp, z);
    }
    void step(double dt, const Command& c) {
        const double q = 0.5 * density() * v * v, area = 84.8, weight = mass * 9.81;
        const double nMax = q * area * 1.0 / weight, nMin = -q * area * 0.5 / weight;
        n += (limit(c.nz, nMin, nMax) - n) * (1 - std::exp(-dt / 0.4));
        p += (limit(c.rollRateDegS, -90, 90) - p) * (1 - std::exp(-dt / 0.3));
        const double cl = n * weight / (q * area);
        const double drag = q * area * (0.020 + 0.194 * cl * cl);
        throttle = at.update(dt, cas(), throttle, true, true);
        const double thrust = F117::ENGINE::engine_dynamics(throttle, v / std::sqrt(1.4 * 287.058 * temperature()), alt, dt, true, false);
        const double qBody = 9.81 * (n - (-dot(kUp, z))) / v, rBody = -9.81 * dot(kUp, y) / v, pBody = p * kDegToRad;
        V3 w = pBody * x + qBody * y + rBody * z;
        x = unit(x + dt * cross(w, x)); y = unit(y + dt * cross(w, y)); y = unit(y + (-dot(x, y)) * x); z = cross(x, y);
        v += ((thrust - drag) / mass - 9.81 * dot(kUp, x)) * dt;
        alt += v * dot(kUp, x) * dt;
    }
};

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // Sector map: labelled regions of Figure 4-3, both sides.
    for (double s : {1.0, -1.0}) {
        assert(classify_sector(87, s * 120) == kPullZeroRollNoseUp);
        assert(classify_sector(-80, s * 30) == kPullZeroRollNoseDown);
        assert(classify_sector(60, s * 140) == kPullRollInverted && classify_sector(40, s * 170) == kPullRollInverted);
        assert(classify_sector(40, s * 50) == kPushRollUpright && classify_sector(10, s * 5) == kPushRollUpright);
        assert(classify_sector(15, s * 135) == kRollUpright && classify_sector(0, s * 180) == kRollUpright);
        assert(classify_sector(5, s * 25) == kRollUpright && classify_sector(-10, s * 70) == kRollUpright);
        assert(classify_sector(-45, s * 60) == kPullRollUpright && classify_sector(0, 0) == kPullRollUpright);
        assert(classify_sector(-45, s * 135) == kPushRollUprightInverted);
        assert(classify_sector(5, s * 10) == kPullRollUpright && classify_sector(6, s * 10) == kPushRollUpright);
    }
    // Sector edges digitised from the figure.
    assert(std::fabs(upper_boundary_deg(180) - 31) < 0.01 && std::fabs(sector2_bank_limit_deg(19) - 104) < 0.01);
    assert(std::fabs(upper_boundary_deg(104) - 18.8) < 0.2);

    // Not functional: A/R door open near level, weight on wheels near level.
    Controller ars;
    assert(!ars.engage(0, 10, false, true) && !ars.engage(5, 0, true, false) && !ars.engaged());
    assert(ars.engage(0, 60, false, true));
    Command c{};
    assert(!ars.update(0, 10, 0, 0, false, false, true, &c) && !ars.engaged());
    assert(ars.engage(30, 0, true, false)); ars.disengage(); assert(!ars.engaged());

    // Commands: vertical sectors pull with zero roll rate, inverted nose-low pushes.
    c = compute_command(87, 100, 0, 0, false); assert(c.nz == kVerticalPullNz && c.rollRateDegS == 0);
    c = compute_command(-45, 150, 0, 0, false); assert(c.nz == kInvertedPushNz && c.rollRateDegS < 0); // short way upright
    c = compute_command(60, 150, 0, 0, false); assert(c.nz == kPullNz && c.rollRateDegS > 0);   // roll on to inverted
    c = compute_command(40, 30, 0, 0, false); assert(c.nz < 0.5 && c.rollRateDegS < 0);          // push, roll upright
    c = compute_command(5, 0, 0, 0, false); assert(std::fabs(c.nz - std::cos(5 * kDegToRad)) < 1e-9 && c.rollRateDegS == 0);
    // Pullout estimate: level or climbing returns current altitude; floored display.
    assert(minimum_pullout_altitude_ft(20000, 200, 5, 90) == 20000);
    assert(displayed_pullout_altitude_ft(15999) == 15000 && displayed_pullout_altitude_ft(-1) == -1000);

    // Closed-loop recoveries from a grid of unusual attitudes, A/T in CAS hold.
    // Below 200 KCAS the manual warns the aircraft may sink before climbing; there a recovery
    // means wings level and pitch no lower than -10 deg (counted separately as low-energy cases).
    int cases = 0, lowEnergy = 0; double worstTime = 0, worstOptimism = -1e9, worstMargin = 0;
    // Slow start (~210 KCAS) at 5,000 ft; faster starts at 20,000 ft, where MIL cannot hold a slow climb.
    const double starts[3][2] = {{110.0, 1500.0}, {200.0, 6000.0}, {250.0, 6000.0}};
    for (const auto& start : starts)
    for (double pitch : {-80.0, -60.0, -30.0, 0.0, 30.0, 60.0, 85.0})
    for (double bank : {-170.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0}) {
        const double tas = start[0];
        Jet jet(pitch, bank, tas, start[1]);
        assert(std::fabs(jet.pitch() - pitch) < 1e-6 && std::fabs(wrap_deg(jet.bank() - bank)) < 1e-6);
        jet.throttle = 60; jet.at.engage(jet.cas(), jet.throttle, true, true);
        const double predicted = minimum_pullout_altitude_ft(jet.alt * 3.28084, jet.v, pitch, bank);
        double minAlt = jet.alt, recoveredAt = -1, maxN = -9, minN = 9;
        for (double t = 0; t < 60; t += 0.01) {
            Command cmd = compute_command(jet.pitch(), jet.bank(), jet.p, jet.pitchRate(), false);
            jet.step(0.01, cmd);
            minAlt = std::min(minAlt, jet.alt); maxN = std::max(maxN, jet.n); minN = std::min(minN, jet.n);
            const double floorDeg = jet.cas() < 200 ? -10 : kTargetPitchGearUpDeg - 3;
            const bool near = std::fabs(jet.bank()) < 5 && jet.pitch() > floorDeg && jet.pitch() < kTargetPitchGearUpDeg + 3;
            if (near && recoveredAt < 0) recoveredAt = t; else if (!near) recoveredAt = -1;
        }
        const double optimism = minAlt * 3.28084 - predicted;  // positive = prediction conservative
        if (recoveredAt < 0 || recoveredAt > 45 || maxN > kVerticalPullNz + 0.01 || minN < kInvertedPushNz - 0.01)
            std::printf("FAIL tas %.0f pitch %.0f bank %.0f: recovered %.1f s, final pitch %.1f bank %.1f, n %.2f..%.2f, final n %.2f cas %.0f (A/T %d cmd %.0f) throttle %.0f\n",
                tas, pitch, bank, recoveredAt, jet.pitch(), jet.bank(), minN, maxN, jet.n, jet.cas(), (int)jet.at.engaged(), jet.at.target_cas_kt(), jet.throttle);
        assert(recoveredAt >= 0 && recoveredAt <= 45);
        assert(maxN <= kVerticalPullNz + 0.01 && minN >= kInvertedPushNz - 0.01);
        if (pitch < 0 && std::fabs(bank) <= 90) { worstOptimism = std::max(worstOptimism, -optimism); worstMargin = std::max(worstMargin, optimism); }
        if (std::fabs(jet.pitch() - kTargetPitchGearUpDeg) >= 3) ++lowEnergy;
        worstTime = std::max(worstTime, recoveredAt); ++cases;
    }
    std::printf("%d recoveries (%d low-energy, below target pitch); slowest settles in %.1f s; pullout estimate (upright dives) optimistic by at most %.0f ft, conservative by at most %.0f ft\n",
        cases, lowEnergy, worstTime, worstOptimism, worstMargin);
    assert(worstOptimism <= 0);  // the readout must never be optimistic
    std::puts("PASS: Figure 4-3 sector map and edges, inhibits, sector commands, pullout estimate, closed-loop recovery from 189 attitudes/speeds with A/T");
}
'''
code = code.replace('AT_HEADER', str(src / 'Avionics/Autothrottle.h').replace('\\', '/'))
code = code.replace('PAARS_HEADER', str(src / 'Avionics/Paars.h').replace('\\', '/'))
p = out / 'paars_tests.cpp'; p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++14 /W4 "{p}" /Fo"{out}/paars_tests.obj" /Fe"{out}/paars_tests.exe"'
r = subprocess.run(cmd, shell=True, capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
r = subprocess.run([str(out / 'paars_tests.exe')], capture_output=True, text=True); print(r.stdout, r.stderr); assert r.returncode == 0
