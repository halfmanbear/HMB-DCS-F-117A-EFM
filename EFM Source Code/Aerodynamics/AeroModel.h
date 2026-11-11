#pragma once
// F-117A aerodynamic model.
//
// A component build-up: the wing-body, the four elevons, the two canted fins and the fuselage side force, each
// placed where it is on the aircraft, so pitching, rolling and yawing moments come out about the real centre of
// mass. Coordinates are body axes in metres with x forward, y up and z right, the same as the DCS model.
//
// Sources:
// - TO 1F-117A-1: trimmed lift line (Figures A6-1 and 2-2), CG range 41.3 to 46.5% MAC, pitch-up above 20 deg
//   indicated AOA and pitch-down departure at large negative AOA (pages 6-2, 6-3), elevons almost full up at
//   9.5 deg AOA with an extreme forward CG (page 6-4), high dihedral effect (page 6-1).
// - Vermeersch, "Investigation of the F117A Vortical Flow Characteristics", MIT 1993: wind tunnel lift to 25 deg
//   and Euler results to 30 deg (Figure 6.21), and the forward movement of the pitching moment slope above about
//   18 deg (Figure 6.22).
// - Geometry measured from the aircraft's exterior model: planform, elevon and fin positions.
// - Slender wing and strip theory for the damping derivatives, which no F-117 source gives.
//
// Sign conventions are those of the rest of the EFM: CN is positive up, CY positive to the right, Cl positive
// right wing down, Cm positive nose up, Cn positive nose right. Rates are body rates with roll positive right
// wing down, pitch positive nose up and yaw positive nose right. Elevator is positive trailing edge down (nose
// down), aileron positive rolls left, rudder positive pushes the tail right (nose left).
//
// Uncertainty: lift and pitching moment inside the normal envelope are set by the manual. The elevon, fin and
// damping values are estimates, good to about 30%; the high-AOA shapes beyond 30 deg are qualitative.

#include <algorithm>
#include <cmath>

namespace F117
{
namespace AERO
{
// Reference values for every coefficient.
constexpr double kRefArea_M2 = 84.77;
constexpr double kRefSpan_M = 13.205;
constexpr double kRefChord_M = 6.422;

// Piecewise-linear lookup, held at the end values outside its range.
template <int N>
inline double lookup(const double (&x)[N], const double (&y)[N], double v)
{
    if (v <= x[0])
        return y[0];
    if (v >= x[N - 1])
        return y[N - 1];
    int i = 1;
    while (x[i] < v)
        ++i;
    const double t = (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + t * (y[i] - y[i - 1]);
}

// Wing-body normal force with the elevons neutral.
// Between -10 and 12 deg it is 0.0542 per degree from 0.72 deg. With the elevons where the pitching moment
// below needs them at the DCS centre of mass, that gives the manual's trimmed line, CN = 0.056 x (AOA -
// 1.02). Untrimmed, it is within 0.03 of the wind tunnel at 10 and 15 deg.
// From 15 to 30 deg the points are the wind tunnel and Euler lift, converted to normal force. Past 30 deg
// they follow sharp-edged delta wings in general: a peak near 40 deg and about 1.25 flat on to the flow.
// Negative AOA has no data; it mirrors the positive side with a little less vortex lift.
constexpr double kAlphaCn[] = {-90, -60, -45, -30, -20, -15, -10, 12, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90};
constexpr double kCnWingBody[] = {-1.20, -1.22, -1.20, -1.10, -0.92, -0.80, -0.581, 0.611, 0.77, 0.945,
                                  1.18,  1.40,  1.50,  1.52,  1.48,  1.42,  1.33,   1.28,  1.26, 1.25};

// Where the wing-body normal force acts. 0.078 m is 44.5% of the planform's 10.9 m geometric MAC, the
// neutral point. The manual's CG range then gives a static margin from +3.2% at the forward limit to -2.0%
// at the aft limit: near-conventional forward, needing the flight controls aft. DCS flies the aircraft at
// x = 0, which is 45.2%, so -0.7%.
// Above about 16 deg the vortices burst over the wing and the centre of pressure moves forward. Its size
// is set so full nose-down elevon can no longer hold the nose just above 20 deg, as the manual says. Below
// -8 deg, the manual's negative AOA limit, it moves forward too, so full nose-up elevon can no longer hold
// the nose beyond about -12 deg: the pitch-down departure the manual warns of. Hands off inverted at
// 170 KCAS the 1 g law settles near -10 deg, which stays controllable. Past 45 deg the aft deck and fins in
// the cross flow move it behind the CG, so a departure oscillates rather than locking in.
constexpr double kAlphaXac[] = {-90, -40, -20, -14, -12, -10, -8, 16, 19, 22, 28, 35, 45, 60, 90};
constexpr double kXacWingBody_M[] = {-0.924, -0.524, 1.28,  1.0,   0.55,  0.25,   0.078, 0.078,
                                     0.326,  0.676,  1.176, 1.076, 0.076, -0.724, -0.924};

// Pitching moment at zero lift about the neutral point. With the elevon power below, the manual's extreme
// forward CG case (page 6-4: a fuel transfer failure, so beyond the 41.3% limit) needs nearly full up
// elevon to hold 9.5 deg if that CG is about 37% MAC. The manual doesn't give the CG, so this is a choice
// in a range of about 0 to -0.02; nearer zero leaves little steady elevon on a gear-down approach, where
// the pitch law has no integrator to hold it.
constexpr double kCm0 = -0.006;

// Elevons. The inboard pair is centred at x = -3.6 m, 3.6 m out; the outboard pair at x = -5.5 m, 4.85 m
// out. A trailing-edge surface also loads the wing ahead of it, so the lift it adds acts at about 45% of
// the local chord: x = -1.1 m inboard and -3.3 m outboard, about -2.3 m and 3.4 m out together. 0.0078
// per degree of both sides together is flap theory for elevons spanning the outer 35% of the wing area.
// With the 2.3 m arm it gives -0.0028 per degree of pitching moment, which is what the forward CG case on
// manual page 6-4 needs.
constexpr double kCnPerElevonDeg = 0.0078;
constexpr double kElevonLiftX_M = -2.3;
constexpr double kElevonLiftY_M = 3.4;
// Effectiveness falls as burst vortices cover the trailing edge.
constexpr double kAlphaElevon[] = {-30, -15, 15, 25, 35, 50};
constexpr double kElevonEffect[] = {0.4, 1.0, 1.0, 0.75, 0.5, 0.3};
// The flow starts to separate from a deflected surface. Beyond 20 deg each degree adds half as much.
constexpr double kElevonLinear_DEG = 20.0;

// Fins: two all-moving panels canted 41 deg out from vertical, 4.0 m2 each, centred at x = -8.3 m and
// 0.95 m up. 2.7 per radian is the lift slope of a panel of aspect ratio 2.4 and its sweep, mounted on the
// body. Panel incidence beyond about 25 deg stalls it.
constexpr double kFinArea_M2 = 4.0;
constexpr double kFinLiftSlope = 2.7;
constexpr double kFinCant_DEG = 41.0;
constexpr double kFinX_M = -8.3;
constexpr double kFinY_M = 0.95;
constexpr double kFinMaxNormal = 1.1;
// The fins sit in the wing's wake and burst vortices at high AOA.
constexpr double kAlphaFin[] = {-20, -10, 15, 25, 35, 50, 90};
constexpr double kFinEffect[] = {0.6, 1.0, 1.0, 0.8, 0.5, 0.35, 0.3};

// Fuselage side force: slender-body lift on the faceted forebody, acting ahead of the CG, plus cross flow
// on the 22 m2 side area at large sideslip.
constexpr double kBodySideSlope = -0.12;  // per radian
constexpr double kBodySideX_M = 4.5;
constexpr double kBodyCrossFlow = -0.16;  // side area / reference area x a cross flow drag of 0.6
constexpr double kBodyCrossFlowX_M = 0.5;

// Dihedral effect of the swept wing, proportional to its lift. The manual's "high dihedral effect": full
// pedal sideslip (8 deg) near the AOA limit below 200 KCAS can need full opposite roll stick (page 6-2).
// This size needs about 90% of the roll law's elevon there, and about 65% at 1 g.
constexpr double kDihedralPerCn = -0.40;  // per radian of sideslip
constexpr double kAlphaDihedral[] = {25, 40, 60, 90};
constexpr double kDihedralEffect[] = {1.0, 0.6, 0.3, 0.2};

// Wing damping. Strip theory over the measured planform with a local lift slope of 2.2 per radian gives
// a roll damping of -0.195, the same as slender wing theory for this aspect ratio (-0.202), and a pitch
// damping of -1.8; the fins' vertical component adds about -0.4 to that. Lift from pitch rate is 0.4 from
// the wing and 0.3 from the fins. Burst vortices reduce the damping at high AOA.
constexpr double kClpWing = -0.195;
constexpr double kCmq = -2.2;
constexpr double kCnq = 0.7;
constexpr double kAlphaDamping[] = {20, 35, 50};
constexpr double kDampingEffect[] = {1.0, 0.5, 0.3};
// Yaw and roll rate cross terms of the wing, from the usual swept-wing approximations.
constexpr double kClrPerCn = 0.25;
constexpr double kCnpPerCn = -0.10;
constexpr double kCnrWing = -0.005;
constexpr double kCnrWingPerCn2 = -0.02;
// Adverse yaw from the elevons, as a fraction of their rolling moment per unit lift.
constexpr double kAdverseYawPerCn = -0.15;

struct Input
{
    double alpha_DEG;
    double beta_DEG;
    double speed_MPS;
    double rollRate_RPS;   // right wing down positive
    double pitchRate_RPS;  // nose up positive
    double yawRate_RPS;    // nose right positive
    double elevator_DEG;   // symmetric elevon, trailing edge down positive
    double aileron_DEG;    // differential elevon each side, positive rolls left
    double rudder_DEG;     // fins, positive pushes the tail right
    double cgX_M;
    double cgY_M;
};

struct Output
{
    double CN;
    double CY;
    double Cl;
    double Cm;
    double Cn;
    double ClPerAileronDeg;   // Rolling moment per degree of aileron here, for the roll law.
    double CmPerElevatorDeg;  // Pitching moment per degree of elevator here, for the pitch law.
    double Clp;               // Roll damping per unit of p b / 2V, wing and fins.
    double CnPerBetaDeg;      // Directional stability per degree, for the debug log.
    double ClPerBetaDeg;
    double CyPerRudderDeg;
    double CnPerRudderDeg;
    double ClPerRudderDeg;
};

// Each side's elevon, with the lift per degree falling off past the linear range.
inline double elevon_effective_deg(double deflection)
{
    const double magnitude = std::abs(deflection);
    const double effective =
        magnitude <= kElevonLinear_DEG ? magnitude : kElevonLinear_DEG + 0.5 * (magnitude - kElevonLinear_DEG);
    return deflection < 0.0 ? -effective : effective;
}

// Normal force coefficient of one fin panel on its own area, with a soft stall.
inline double fin_normal(double incidence_RAD)
{
    return kFinMaxNormal * std::tanh(kFinLiftSlope * incidence_RAD / kFinMaxNormal);
}

inline Output evaluate(const Input& in)
{
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    const double alpha = (std::max)(-90.0, (std::min)(90.0, in.alpha_DEG));
    const double beta = (std::max)(-60.0, (std::min)(60.0, in.beta_DEG));
    const double V = (std::max)(in.speed_MPS, 10.0);
    const double b = kRefSpan_M;
    const double c = kRefChord_M;

    Output out = {};

    // Wing-body lift and its centre of pressure. Lift falls with cos(beta) as the flow turns along the span.
    const double cosBeta = std::cos(beta * kDeg);
    const double cnWingBody = lookup(kAlphaCn, kCnWingBody, alpha) * cosBeta;
    const double xac = lookup(kAlphaXac, kXacWingBody_M, alpha);

    // Elevons through the mixer: right = elevator + aileron, left = elevator - aileron.
    const double elevonEffect = lookup(kAlphaElevon, kElevonEffect, alpha);
    const double perSide = 0.5 * kCnPerElevonDeg * elevonEffect;
    const double cnRight = perSide * elevon_effective_deg(in.elevator_DEG + in.aileron_DEG);
    const double cnLeft = perSide * elevon_effective_deg(in.elevator_DEG - in.aileron_DEG);

    const double pHat = in.rollRate_RPS * b / (2.0 * V);
    const double qHat = in.pitchRate_RPS * c / (2.0 * V);
    const double rHat = in.yawRate_RPS * b / (2.0 * V);
    const double damping = lookup(kAlphaDamping, kDampingEffect, alpha);

    out.CN = cnWingBody + cnRight + cnLeft + kCnq * damping * qHat;
    out.Cm = kCm0 + cnWingBody * (xac - in.cgX_M) / c + (cnRight + cnLeft) * (kElevonLiftX_M - in.cgX_M) / c +
             kCmq * damping * qHat;

    out.CmPerElevatorDeg = kCnPerElevonDeg * elevonEffect * (kElevonLiftX_M - in.cgX_M) / c;

    // Rolling moment from the elevons: lift on the right wing rolls left.
    const double clElevons = (cnLeft - cnRight) * kElevonLiftY_M / b;
    out.ClPerAileronDeg = -kCnPerElevonDeg * elevonEffect * kElevonLiftY_M / b;

    // Fins. The flow each sees sideways is the sideslip plus what roll and yaw rate add at the fin.
    const double finEffect = lookup(kAlphaFin, kFinEffect, alpha);
    const double cant = kFinCant_DEG * kDeg;
    const double finArm = kFinX_M - in.cgX_M;  // negative: behind the CG
    const double finHeight = kFinY_M - in.cgY_M;
    const double finSideslip = beta * kDeg + (in.rollRate_RPS * finHeight + in.yawRate_RPS * finArm) / V;
    // A fin panel's incidence is the sideways flow resolved normal to it, plus its own deflection.
    // Positive rudder pushes the tail right, as negative sideslip does.
    const double incidence = std::cos(cant) * finSideslip - in.rudder_DEG * kDeg;
    const double finToRef = 2.0 * kFinArea_M2 / kRefArea_M2 * std::cos(cant) * finEffect;
    const double cyFins = -finToRef * fin_normal(incidence);

    // Fuselage side force.
    const double sinBeta = std::sin(beta * kDeg);
    const double cyBodyLinear = kBodySideSlope * sinBeta * cosBeta;
    const double cyCrossFlow = kBodyCrossFlow * sinBeta * std::abs(sinBeta);

    out.CY = cyFins + cyBodyLinear + cyCrossFlow;

    // Yawing moment of a side force: CY x (its x - CG x) / span.
    const double dihedralEffect = lookup(kAlphaDihedral, kDihedralEffect, alpha);
    const double wingLift = cnWingBody;
    out.Cn = cyFins * finArm / b + cyBodyLinear * (kBodySideX_M - in.cgX_M) / b +
             cyCrossFlow * (kBodyCrossFlowX_M - in.cgX_M) / b + kCnpPerCn * wingLift * pHat +
             (kCnrWing + kCnrWingPerCn2 * wingLift * wingLift) * damping * rHat +
             kAdverseYawPerCn * wingLift * clElevons;

    // Rolling moment of the fins' side force acting above the CG.
    out.Cl = cyFins * finHeight / b + kDihedralPerCn * wingLift * dihedralEffect * beta * kDeg + clElevons +
             kClpWing * damping * pHat + kClrPerCn * wingLift * rHat;

    // Local derivatives for the flight controls and the debug log.
    const double finSlope = finToRef * kFinLiftSlope;  // side force per radian of fin incidence, small angles
    out.Clp = kClpWing * damping - finSlope * std::cos(cant) * 2.0 * finHeight * finHeight / (b * b);
    out.CyPerRudderDeg = finSlope * kDeg;
    out.CnPerRudderDeg = out.CyPerRudderDeg * finArm / b;
    out.ClPerRudderDeg = out.CyPerRudderDeg * finHeight / b;
    out.CnPerBetaDeg = (-finSlope * std::cos(cant) * finArm + kBodySideSlope * (kBodySideX_M - in.cgX_M)) / b * kDeg;
    out.ClPerBetaDeg = (-finSlope * std::cos(cant) * finHeight / b + kDihedralPerCn * wingLift * dihedralEffect) * kDeg;
    return out;
}
}  // namespace AERO
}  // namespace F117
