#pragma once

// Flight-model state shared by the translation units of the EFM. Every variable is an
// inline variable, so there is exactly one instance across the DLL.

#include "../EfmMath.h"
#include "../Engine/Engine.h"
#include "../Avionics/Autothrottle.h"
#include "../Avionics/Paars.h"
#include "../Engine/EngineStart.h"
#include "../param_functions.h"
#include "../Systems/FuelTanks.h"
#include "../include/Cockpit/CockpitAPI_Declare.h"

// Totals from the last aerodynamic update, with gear, chute, bay and ground effect included.
namespace F117
{
namespace Aero
{
inline double cx_total = 0.0;  // Drag along the body axis, positive aft
inline double cz_total = 0.0;  // Normal force, negative up
inline double cm_total = 0.0;
inline double cy_total = 0.0;
inline double cn_total = 0.0;
inline double cl_total = 0.0;
inline double cl_per_aileron_deg = 0.0;       // For the roll law
inline double cm_per_elevator_deg = -0.0028;  // For the pitch law
inline double cn_per_rudder_deg = -0.002;     // For the yaw damper
inline double clp = 0.0;
}  // namespace Aero
}  // namespace F117

// Body-frame totals are returned at the DCS centre of mass. Moments are supplied separately.
inline Vec3 common_force{}, common_moment{}, center_of_gravity{}, inertia{};
inline Vec3 wind{}, wind_world_cs{}, position_world_cs{}, velocity_world_cs{};

namespace F117
{
// Environment and measured motion (SI unless a suffix states otherwise).
inline double ambient_temperature_deg_k{};
inline double ambient_density_kg_per_m3{};
inline double ambient_pressure_pa{101325.0};
inline double altitude_m{};
inline double calibrated_airspeed_kt{};
inline double total_velocity_mps{};
inline double dynamic_pressure_pa{};
inline double mach{};
inline double alpha_deg{};
inline double beta_deg{};
inline double roll_rate_rps{};
inline double pitch_rate_rps{};
inline double yaw_rate_rps{};
inline double aircraft_quaternion_x{};
inline double aircraft_quaternion_y{};
inline double aircraft_quaternion_z{};
inline double aircraft_quaternion_w{1.0};
inline double pitch_angle{};
inline double roll_angle{};
inline double vspeed{};
inline double az{};
inline double ay{};
inline double ay_world{};
inline double weight_n{131222.538};
inline double surface_height_m{};
inline bool surface_height_valid{};
inline bool altitude_valid{};
inline bool weight_on_wheels{};
inline bool wow_from_draw_args{};
inline bool suspension_wow[3]{};

// Geometry and unit conversions used by the current aero and controller models.
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double radians_to_degrees = 180.0 / pi;
inline constexpr double wing_span_m = 13.205;
inline constexpr double wing_area_m2 = 84.77;
inline constexpr double mean_chord_m = 6.422;

// Pilot requests, commanded surfaces and actual surface feedback.
inline constexpr double aileron_full_scale_deg = 25.5;  // Aileron deflection that aileron_PCT = 1 stands for.
inline double pitch_trim_position{};       // Pilot pitch trim: -1 full nose-down to +1 full nose-up.
inline int pitch_trim_direction{};         // Trim switch held: +1 nose-up, -1 nose-down, 0 released.
inline double pitch_trim_elevon_deg{};     // Elevon bias from pilot trim while autotrim is off.
inline double autopilot_pitch_bias_deg{};  // Autopilot hold output, elevon degrees added after the pitch law.
inline double roll_trim{};
inline double yaw_trim{};
inline double roll_cmd{};
inline double ped_input{};
inline double elevator_deg{};
inline double aileron_deg{};
inline double rudder_deg{};
inline double elevator_deg_commanded{};
inline double aileron_deg_commanded{};
inline double rudder_deg_commanded{};
inline double aileron_pct{};
inline double rudder_pct{};
inline double elevator_pct{};
inline float elev_pos{};
inline float rudder_pos{};
inline int alt_hold{};
inline int altroll_hold{};
inline int horiz_hold{};
inline double delta_time{};
inline bool sim_initialized{};

// Landing gear, steering and ancillary mechanisms.
inline double gear_down{};
inline double gear_command{};
inline double wheel_brake_command{};
inline double rolling_friction{0.015};
// Normal/high-authority steering: TO 1F-117A-1, page 1-29.
inline constexpr double nose_wheel_yaw_low_rad = 0.1745;
inline constexpr double nose_wheel_yaw_high_rad = 0.7854;
inline constexpr double nose_wheel_yaw_range_rate_radps = 0.6;
inline double nose_wheel_yaw_range_rad{nose_wheel_yaw_low_rad};
inline bool nws_authority_held{};
inline bool air_refuel_door_open{};
inline double tailhook_command{};
inline double tailhook_pos{};
inline double dragchute_command{};
inline double dragchute{};
// Weapon bays: 0 is the left bay (station 1, arg 27), 1 is the right bay (station 2, arg 28).
// 0 is closed and 1 is door open with the trapeze fully lowered.
inline constexpr int weapon_bay_count = 2;
// Each bay arg opens the door over 0 to 0.7 and lowers the trapeze over 0.6 to 1 (the keys in the EDM).
inline constexpr double weapon_bay_door_fully_open_arg = 0.7;
// TO 1F-117A-1 (Section VI): opening a single bay door "has effects similar to gear extension", which is about one
// degree of sideslip that the pilot can retrim. The sign is a choice: the extra drag on the open side yaws the nose
// that way, as the left-side nose gear does.
inline constexpr double weapon_bay_one_door_sideslip_deg = 1.0;
inline float bay_cmd[weapon_bay_count]{};
inline float bay_state[weapon_bay_count]{};

// Engine telemetry and the fuel mass already acknowledged by DCS.
inline bool engineswitch{true};
inline double throttle_state{0.1};
// Combined lever position (mean of the two sides), used by the autothrottle and the cockpit lever. Each engine
// is driven by its own side input.
inline double throttle_input{0.1};
inline double throttle_side_input[2]{0.1, 0.1};
inline double thrust_n{};
inline Engine::Engine_state engines[2]{};
inline double engine_thrust_n[2]{};
inline double engine_fuel_flow_kg_s[2]{};
inline double engine_egt_c[2]{};
inline double engine_oil_psi[2]{};
inline double fuel_in_dcs_kg[Fuel_tanks::tank_count]{};  // each tank as last reported to DCS
inline double internal_fuel{};
inline double external_fuel{};
inline EDPARAM cockpit_api{};
inline Param_stuff param_class{};

// Weapon delivery state belongs to one aircraft session.
inline bool weapon_release_pickle_held{};
inline bool weapon_release_gate{};
inline bool weapon_release_gate_fired{};
// A bay is owned by the release when the pickle opened it, so the pickle closes it again.
// A bay the pilot opened by hand is never owned and stays open until the pilot closes it.
inline bool weapon_bay_owned_by_release[weapon_bay_count]{};
inline bool weapon_bay_close_pending[weapon_bay_count]{};
inline bool master_armed{};
inline bool laser_armed{};
inline double weapon_bay_close_delay[weapon_bay_count]{};
inline double weapon_release_gate_hold{};
inline double release_retry_delay{};
inline int release_status{};
inline unsigned release_sequence{};
inline double release_target[3]{};
inline double laser_window{};
inline double weapon_flight_time_remaining{};
inline double ccrp_time_of_flight{};

struct Damage_state
{
    double left_wing = 1.0;     // Remaining component integrity; damage is monotonic until repair.
    double right_wing = 1.0;    // Remaining component integrity; damage is monotonic until repair.
    double left_engine = 1.0;   // Remaining component integrity; damage is monotonic until repair.
    double right_engine = 1.0;  // Remaining component integrity; damage is monotonic until repair.
    double left_tail = 1.0;     // Remaining component integrity; damage is monotonic until repair.
    double right_tail = 1.0;    // Remaining component integrity; damage is monotonic until repair.
    double cockpit = 1.0;       // Remaining component integrity; damage is monotonic until repair.

    double left_outer_elevon = 1.0;
    double right_outer_elevon = 1.0;
    double left_inner_elevon = 1.0;
    double right_inner_elevon = 1.0;

    double nose = 1.0;
    double left_fuselage = 1.0;
    double right_fuselage = 1.0;
    double belly = 1.0;
    double left_tail_structure = 1.0;
    double right_tail_structure = 1.0;
    double left_wing_inner = 1.0;
    double right_wing_inner = 1.0;
    double left_wing_outer = 1.0;
    double right_wing_outer = 1.0;
    double nose_gear = 1.0;
    double left_gear = 1.0;
    double right_gear = 1.0;
    double nose_wheel = 1.0;
    double left_wheel = 1.0;
    double right_wheel = 1.0;
    bool fire_active[9] = {};

    // Derived values (updated each frame in ed_fm_simulate)
    double structural_drag_factor = 1.0;  // Cx multiplier from airframe section damage
    double wing_asymmetry = 0.0;          // leftWing - rightWing
    double total_wing_loss = 0.0;         // average damage across both wings
    double total_tail_loss = 0.0;         // average damage across both tails
    double engine_asymmetry = 0.0;        // leftEngine - rightEngine
};
}  // namespace F117

// World-axis state captured from DCS and reused during the simulation step.
inline double ax_world{};
inline double az_world{};
inline double vx_world{};
inline double vy_world{};
inline double vz_world{};

// Controllers and session flags shared across the control, propulsion and command units.
inline Autothrottle::Controller g_autothrottle;
inline Paars::Controller g_paars;
inline F117::Engine_start::Starts g_engine_starts;
inline bool g_paars_engaged_autothrottle = false;  // PAARS moved the power levers by engaging A/T
inline double g_paars_roll_rate_deg_s = 0.0;
