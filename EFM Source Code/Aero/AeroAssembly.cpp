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
    if (!F117::surface_height_valid || !F117::altitude_valid)
        return 1.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    const double ratio = 16.0 * clearance / F117::wing_span_m;
    return ratio * ratio / (1.0 + ratio * ratio);
}

// Fraction of extra lift near the ground, at the same AOA. TO 1F-117A-1 (Appendix A3 introduction) says ground effect
// and the second engine's thrust together make liftoff AOA about 2 deg less than the 8.5 deg reference. 20% at the
// height of the aircraft origin on its wheels (2.12 m) gives that at the 46,000 lb, 194 KCAS takeoff example.
// The gain halves for every 0.15 span of height. That is a choice: the manual gives one point, and published data
// for low aspect ratio delta wings varies widely.
double lift_ground_effect_gain()
{
    if (!F117::surface_height_valid || !F117::altitude_valid)
        return 0.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    constexpr double gain_on_wheels = 0.20;
    constexpr double height_on_wheels_m = 2.12;
    constexpr double halving_spans = 0.15;
    return gain_on_wheels * exp2(-(clearance - height_on_wheels_m) / (halving_spans * F117::wing_span_m));
}

namespace
{
// Wave drag rise: starts at the divergence Mach and steepens into a barrier near the level-speed limit.
double wave_drag_coefficient(double mach)
{
    double cd_wave = 0.0;
    const double mdiv = 0.85;
    // TO 1F-117A-1 limits the aircraft to Mach 0.90 and has no data above it. Published figures put the
    // maximum level speed between Mach 0.92 and 0.95; this barrier sets it near 0.92 at its best altitude.
    const double mlimit = 0.91;

    if (mach > mdiv)
    {
        if (mach < 1.05)
        {
            const double div_factor = (mach - mdiv) / (1.05 - mdiv);
            cd_wave = 0.007159 * div_factor * div_factor;

            if (mach > mlimit)
            {
                const double barrier_factor = (mach - mlimit) / (1.05 - mlimit);
                cd_wave += 0.096098 * barrier_factor * barrier_factor;
            }
        }
        else
        {
            cd_wave = 0.12;
        }
    }
    return cd_wave;
}

// One door open more than the other yaws the nose toward it (positive left-bay excess gives a nose-left moment,
// which is negative Cn here). The size is the Cn that holds one degree of sideslip.
double bay_door_yaw_coefficient(const F117::Aero::Output& aero)
{
    const double left_door =
        limit(F117::Actuators::weapon_bay[0].position / F117::weapon_bay_door_fully_open_arg, 0.0, 1.0);
    const double right_door =
        limit(F117::Actuators::weapon_bay[1].position / F117::weapon_bay_door_fully_open_arg, 0.0, 1.0);
    return -std::fabs(aero.cn_per_beta_deg) * F117::weapon_bay_one_door_sideslip_deg * (left_door - right_door);
}
}  // namespace

void update_aerodynamic_coefficients()
{
    // On wing area. Set with the brakes by the landing distances in TO 1F-117A-1 (Figure A8-2).
    const double c_dchute = 0.20 * F117::Actuators::drag_chute.position;
    const double cxchute = c_dchute * cos(F117::pi / 180.0);

    // 0.015 with both bays open, shared between them.
    double bay_open_sum = 0.0;
    for (const auto& servo : F117::Actuators::weapon_bay)
        bay_open_sum += servo.position;
    const double c_dbay = 0.015 * bay_open_sum / F117::weapon_bay_count;
    const double czbay = -(c_dbay * sin(F117::pi / 180.0));
    const double cxbay = c_dbay * cos(F117::pi / 180.0);

    // On wing area. TO 1F-117A-1 Figure 3-5 gives best glides of about 8.5 gear up and 5.9 gear down, so lowering
    // the gear adds about 44% to the drag. On this airframe's drag that is about 0.016.
    const double cd_gear = 0.016 * F117::gear_down;
    const double cz_gear = -(cd_gear * sin(F117::pi / 180.0));
    const double cx_gear = cd_gear * cos(F117::pi / 180.0);

    // Control surfaces as the damage model leaves them.
    F117::Aero::Input in;
    in.alpha_deg = F117::alpha_deg;
    in.beta_deg = F117::beta_deg;
    in.speed_mps = F117::total_velocity_mps;
    in.roll_rate_rps = F117::roll_rate_rps;
    in.pitch_rate_rps = F117::pitch_rate_rps;
    in.yaw_rate_rps = F117::yaw_rate_rps;
    in.elevator_deg = F117::elevator_pct * 25.0;
    in.aileron_deg = F117::aileron_pct * F117::aileron_full_scale_deg;
    in.rudder_deg = F117::rudder_pct * 30.0;
    in.cg_x_m = center_of_gravity.x;
    in.cg_y_m = center_of_gravity.y;
    const F117::Aero::Output aero = F117::Aero::evaluate(in);

    const double alpha1_deg_limited = limit(F117::alpha_deg, -90.0, 90.0);

    // Gear down, TO 1F-117A-1 Figure 2-2 needs more AOA at high lift than the gear-up line gives.
    const double gear_down_lift_loss = 0.0075 * F117::gear_down * (std::max)(alpha1_deg_limited - 6.0, 0.0);
    // Cz is negative for upward lift.
    const double cz = -aero.cn_force + gear_down_lift_loss;
    // Ground effect adds lift at right angles to the airflow, so the drag terms below still use the free-air Cz.
    const double cz_ground_effect = lift_ground_effect_gain() * cz;

    // Cd0 and the wave drag below Mlimit are fitted to TO 1F-117A-1 by tools/fm_bench/fit_performance.py in the
    // mod repository. Normal force acts along the body axis here, so tilting it with AOA supplies most of the
    // induced drag the data shows, as on a sharp-edged wing. Cz is negative for upward lift, so the lift terms are
    // the remainder: less drag than the tilt alone at low lift and more at high lift.
    const double cd0 = 0.017152 + 0.038229 * cz + 0.012645 * cz * cz;

    // Ground effect relieves part of that tilted normal force.
    const double phi_ge = induced_drag_ground_effect();
    // The second term cancels the drag the extra ground-effect lift would gain from being tilted with the body axis.
    const double cd_induced =
        ((1.0 - phi_ge) * cz + cz_ground_effect) * tan(limit(alpha1_deg_limited, -89.0, 89.0) * F117::pi / 180.0);

    const double cd_wave = wave_drag_coefficient(F117::mach);

    F117::Aero::cx_total = cd0 + cd_induced + cd_wave + cx_gear + cxchute + cxbay;
    F117::Aero::cz_total = cz + cz_ground_effect + cz_gear + czbay;
    F117::Aero::cm_total = aero.cm;
    F117::Aero::cy_total = aero.cy;
    const double cn_bay = bay_door_yaw_coefficient(aero);
    F117::Aero::cn_total = aero.cn + cn_bay;
    F117::Aero::cl_total = aero.cl;
    F117::Aero::cl_per_aileron_deg = aero.cl_per_aileron_deg;
    F117::Aero::cm_per_elevator_deg = aero.cm_per_elevator_deg;
    F117::Aero::cn_per_rudder_deg = aero.cn_per_rudder_deg;
    F117::Aero::clp = aero.clp;

    F117::Flight_controls::Aero_snapshot snapshot;
    snapshot.cy_delta_rudder = aero.cy_per_rudder_deg;
    snapshot.cn_delta_rudder = aero.cn_per_rudder_deg;
    snapshot.cl_delta_rudder = aero.cl_per_rudder_deg;
    snapshot.cn_delta_beta = aero.cn_per_beta_deg;
    snapshot.cl_delta_beta = aero.cl_per_beta_deg;
    snapshot.cy_total = aero.cy;
    snapshot.cn_total = aero.cn;
    snapshot.cl_total = aero.cl;
    F117::Flight_controls::update_aero_debug_snapshot(snapshot);
}

void apply_aerodynamic_forces_and_thrust()
{
    const double q_s = F117::wing_area_m2 * F117::dynamic_pressure_pa;  // N

    Vec3 cy_force(0.0, 0.0, F117::Aero::cy_total * q_s);
    add_local_force(cy_force);

    Vec3 cx_force(-F117::Aero::cx_total * q_s * g_damage.structural_drag_factor, 0, 0);
    add_local_force(cx_force);

    const double lift_damage_factor = 1.0 - g_damage.total_wing_loss;
    Vec3 cz_force(0.0, -F117::Aero::cz_total * q_s * lift_damage_factor, 0.0);
    add_local_force(cz_force);

    const double q_s_b = q_s * F117::wing_span_m;  // N·m
    double roll_moment = F117::Aero::cl_total * q_s_b;
    roll_moment += g_damage.wing_asymmetry * q_s_b * 0.05;

    const double pitch_damage_factor = 1.0 - g_damage.total_tail_loss * 0.5;
    const double nose_down_bias = -g_damage.total_wing_loss * q_s_b * 0.08;
    const double pitch_moment = F117::Aero::cm_total * q_s * F117::mean_chord_m * pitch_damage_factor + nose_down_bias;

    const double q_s_span = q_s_b;  // N·m (same as qS_b for yaw)
    double yaw_moment = -F117::Aero::cn_total * q_s_span;
    const double tail_asym_damage = g_damage.left_tail - g_damage.right_tail;
    yaw_moment += tail_asym_damage * q_s_span * 0.03;
    add_local_moment({roll_moment, yaw_moment, pitch_moment});

    const double thrust_cant_up_deg = 0.1;
    const double thrust_toe_out_deg = 10;
    const double cant_up_rad = thrust_cant_up_deg * F117::pi / 180.0;
    const double toe_out_rad = thrust_toe_out_deg * F117::pi / 180.0;

    const double left_thrust = F117::engine_thrust_n[F117::Engine_start::left] * g_damage.left_engine;
    const double right_thrust = F117::engine_thrust_n[F117::Engine_start::right] * g_damage.right_engine;

    const double thrust_x_l = left_thrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_l = left_thrust * sin(cant_up_rad);
    const double thrust_z_l = left_thrust * cos(cant_up_rad) * sin(toe_out_rad);

    const double thrust_x_r = right_thrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_r = right_thrust * sin(cant_up_rad);
    const double thrust_z_r = right_thrust * cos(cant_up_rad) * sin(toe_out_rad);

    Vec3 thrust_force_l(thrust_x_l, thrust_y_l, thrust_z_l);
    add_local_force(thrust_force_l);

    Vec3 thrust_force_r(thrust_x_r, thrust_y_r, -thrust_z_r);
    add_local_force(thrust_force_r);
}
