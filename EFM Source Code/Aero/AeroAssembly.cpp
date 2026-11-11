#include "stdafx.h"
#include "AeroAssembly.h"

#include <algorithm>
#include <cmath>
#include "../Aerodynamics/AeroModel.h"
#include "../Atmosphere/Atmosphere.h"
#include "../Damage/Damage.h"
#include "../FlightControls/FlightControls.h"
#include "../FlightModel/Forces.h"
#include "../FlightModel/State.h"
#include "../Actuators/Actuators.h"
#include "../Engine/EngineStart.h"

// Fraction of induced drag remaining near the ground (Wieselberger): 0 on the ground, 1 in free air.
double induced_drag_ground_effect()
{
    if (!F117::surface_height_valid || !F117::altitude_valid) return 1.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    const double ratio = 16.0 * clearance / F117::wingSpan_M;
    return ratio * ratio / (1.0 + ratio * ratio);
}

// Fraction of extra lift near the ground, at the same AOA. TO 1F-117A-1 (Appendix A3 introduction) says ground effect
// and the second engine's thrust together make liftoff AOA about 2 deg less than the 8.5 deg reference. 20% at the
// height of the aircraft origin on its wheels (2.12 m) gives that at the 46,000 lb, 194 KCAS takeoff example.
// The gain halves for every 0.15 span of height. That is a choice: the manual gives one point, and published data
// for low aspect ratio delta wings varies widely.
double lift_ground_effect_gain()
{
    if (!F117::surface_height_valid || !F117::altitude_valid) return 0.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    constexpr double kGainOnWheels = 0.20;
    constexpr double kHeightOnWheels_M = 2.12;
    constexpr double kHalvingSpans = 0.15;
    return kGainOnWheels * exp2(-(clearance - kHeightOnWheels_M) / (kHalvingSpans * F117::wingSpan_M));
}

void update_aerodynamic_coefficients()
{
    // On wing area. Set with the brakes by the landing distances in TO 1F-117A-1 (Figure A8-2).
    const double CDchute = 0.20 * F117::ACTUATORS::dragChute.position;
    const double Cxchute = CDchute * cos(F117::pi / 180.0);

    // 0.015 with both bays open, shared between them.
    double bayOpenSum = 0.0;
    for (const auto& servo : F117::ACTUATORS::weaponBay) bayOpenSum += servo.position;
    const double CDbay = 0.015 * bayOpenSum / F117::kWeaponBayCount;
    const double Czbay = -(CDbay * sin(F117::pi / 180.0));
    const double Cxbay = CDbay * cos(F117::pi / 180.0);

    // On wing area. TO 1F-117A-1 Figure 3-5 gives best glides of about 8.5 gear up and 5.9 gear down, so lowering
    // the gear adds about 44% to the drag. On this airframe's drag that is about 0.016.
    const double CDGear = 0.016 * F117::gearDown;
    const double CzGear = -(CDGear * sin(F117::pi / 180.0));
    const double CxGear = CDGear * cos(F117::pi / 180.0);

    // Control surfaces as the damage model leaves them.
    F117::AERO::Input in;
    in.alpha_DEG = F117::alpha_DEG;
    in.beta_DEG = F117::beta_DEG;
    in.speed_MPS = F117::totalVelocity_MPS;
    in.rollRate_RPS = F117::rollRate_RPS;
    in.pitchRate_RPS = F117::pitchRate_RPS;
    in.yawRate_RPS = F117::yawRate_RPS;
    in.elevator_DEG = F117::elevator_PCT * 25.0;
    in.aileron_DEG = F117::aileron_PCT * F117::kAileronFullScale_DEG;
    in.rudder_DEG = F117::rudder_PCT * 30.0;
    in.cgX_M = center_of_gravity.x;
    in.cgY_M = center_of_gravity.y;
    const F117::AERO::Output aero = F117::AERO::evaluate(in);

    const double alpha1_DEG_Limited = limit(F117::alpha_DEG, -90.0, 90.0);

    // Gear down, TO 1F-117A-1 Figure 2-2 needs more AOA at high lift than the gear-up line gives.
    const double gearDownLiftLoss = 0.0075 * F117::gearDown * (std::max)(alpha1_DEG_Limited - 6.0, 0.0);
    // Cz is negative for upward lift.
    const double Cz = -aero.CN + gearDownLiftLoss;
    // Ground effect adds lift at right angles to the airflow, so the drag terms below still use the free-air Cz.
    const double CzGroundEffect = lift_ground_effect_gain() * Cz;

    // Cd0 and the wave drag below Mlimit are fitted to TO 1F-117A-1 by tools/fm_bench/fit_performance.py in the
    // mod repository. Normal force acts along the body axis here, so tilting it with AOA supplies most of the
    // induced drag the data shows, as on a sharp-edged wing. Cz is negative for upward lift, so the lift terms are
    // the remainder: less drag than the tilt alone at low lift and more at high lift.
    const double Cd0 = 0.017152 + 0.038229 * Cz + 0.012645 * Cz * Cz;

    // Ground effect relieves part of that tilted normal force.
    const double phi_GE = induced_drag_ground_effect();
    // The second term cancels the drag the extra ground-effect lift would gain from being tilted with the body axis.
    const double Cd_induced = ((1.0 - phi_GE) * Cz + CzGroundEffect) * tan(limit(alpha1_DEG_Limited, -89.0, 89.0) * F117::pi / 180.0);

    double Cd_wave = 0.0;
    const double Mdiv = 0.85;
    // TO 1F-117A-1 limits the aircraft to Mach 0.90 and has no data above it. Published figures put the
    // maximum level speed between Mach 0.92 and 0.95; this barrier sets it near 0.92 at its best altitude.
    const double Mlimit = 0.91;

    if (F117::mach > Mdiv)
    {
        if (F117::mach < 1.05)
        {
            const double div_factor = (F117::mach - Mdiv) / (1.05 - Mdiv);
            Cd_wave = 0.007159 * div_factor * div_factor;

            if (F117::mach > Mlimit)
            {
                const double barrier_factor = (F117::mach - Mlimit) / (1.05 - Mlimit);
                Cd_wave += 0.096098 * barrier_factor * barrier_factor;
            }
        }
        else
        {
            Cd_wave = 0.12;
        }
    }

    F117::AERO::Cx_total = Cd0 + Cd_induced + Cd_wave + CxGear + Cxchute + Cxbay;
    F117::AERO::Cz_total = Cz + CzGroundEffect + CzGear + Czbay;
    F117::AERO::Cm_total = aero.Cm;
    F117::AERO::Cy_total = aero.CY;
    // One door open more than the other yaws the nose toward it (positive left-bay excess gives a nose-left moment,
    // which is negative Cn here). The size is the Cn that holds one degree of sideslip.
    const double leftDoor = limit(F117::ACTUATORS::weaponBay[0].position / F117::kWeaponBayDoorFullyOpenArg, 0.0, 1.0);
    const double rightDoor = limit(F117::ACTUATORS::weaponBay[1].position / F117::kWeaponBayDoorFullyOpenArg, 0.0, 1.0);
    const double CnBay = -std::fabs(aero.CnPerBetaDeg) * F117::kWeaponBayOneDoorSideslip_DEG * (leftDoor - rightDoor);
    F117::AERO::Cn_total = aero.Cn + CnBay;
    F117::AERO::Cl_total = aero.Cl;
    F117::AERO::ClPerAileronDeg = aero.ClPerAileronDeg;
    F117::AERO::CmPerElevatorDeg = aero.CmPerElevatorDeg;
    F117::AERO::CnPerRudderDeg = aero.CnPerRudderDeg;
    F117::AERO::Clp = aero.Clp;

    F117::FLIGHTCONTROLS::update_aero_debug_snapshot(
        aero.CyPerRudderDeg,
        aero.CnPerRudderDeg,
        aero.ClPerRudderDeg,
        aero.CnPerBetaDeg,
        aero.ClPerBetaDeg,
        aero.CY,
        aero.Cn,
        aero.Cl);
}

void apply_aerodynamic_forces_and_thrust()
{
    const double qS = F117::wingArea_M2 * F117::dynamicPressure_PA; // N

    Vec3 cy_force(0.0, 0.0, F117::AERO::Cy_total * qS);
    add_local_force(cy_force);

    Vec3 cx_force(-F117::AERO::Cx_total * qS * g_damage.structuralDragFactor, 0, 0);
    add_local_force(cx_force);

    const double liftDamageFactor = 1.0 - g_damage.totalWingLoss;
    Vec3 cz_force(0.0, -F117::AERO::Cz_total * qS * liftDamageFactor, 0.0);
    add_local_force(cz_force);

    const double qS_b = qS * F117::wingSpan_M; // N·m
    double rollMoment = F117::AERO::Cl_total * qS_b;
    rollMoment += g_damage.wingAsymmetry * qS_b * 0.05;


    const double pitchDamageFactor = 1.0 - g_damage.totalTailLoss * 0.5;
    const double noseDownBias = -g_damage.totalWingLoss * qS_b * 0.08;
    const double pitchMoment = F117::AERO::Cm_total * qS * F117::meanChord_M * pitchDamageFactor + noseDownBias;

    const double qS_span = qS_b; // N·m (same as qS_b for yaw)
    double yawMoment = -F117::AERO::Cn_total * qS_span;
    const double tailAsymDamage = g_damage.leftTail - g_damage.rightTail;
    yawMoment += tailAsymDamage * qS_span * 0.03;
    add_local_moment({rollMoment, yawMoment, pitchMoment});

    const double thrust_cant_up_deg = 0.1;
    const double thrust_toe_out_deg = 10;
    const double cant_up_rad = thrust_cant_up_deg * F117::pi / 180.0;
    const double toe_out_rad = thrust_toe_out_deg * F117::pi / 180.0;

    const double leftThrust = F117::engine_thrust_N[F117::ENGINE_START::LEFT] * g_damage.leftEngine;
    const double rightThrust = F117::engine_thrust_N[F117::ENGINE_START::RIGHT] * g_damage.rightEngine;

    const double thrust_x_L = leftThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_L = leftThrust * sin(cant_up_rad);
    const double thrust_z_L = leftThrust * cos(cant_up_rad) * sin(toe_out_rad);

    const double thrust_x_R = rightThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_R = rightThrust * sin(cant_up_rad);
    const double thrust_z_R = rightThrust * cos(cant_up_rad) * sin(toe_out_rad);

    Vec3 thrust_force_L(thrust_x_L, thrust_y_L, thrust_z_L);
    add_local_force(thrust_force_L);

    Vec3 thrust_force_R(thrust_x_R, thrust_y_R, -thrust_z_R);
    add_local_force(thrust_force_R);
}
