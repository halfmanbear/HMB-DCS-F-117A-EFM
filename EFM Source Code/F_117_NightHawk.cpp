#include "stdafx.h"
#include "F_117_NightHawk.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <string>

#include "EfmMath.h"
#include "Inputs.h"                              // DCS and aircraft command protocol
#include "param_functions.h"                     // Mission options
#include "include/Cockpit/CockpitAPI_Declare.h"  // Lua parameter bridge
#include "Aerodynamics/AeroModel.h"
#include "Actuators/Actuators.h"       // Airframe motion
#include "Atmosphere/Atmosphere.h"     // Air data
#include "FlightModel/State.h"         // Shared flight-model state
#include "Damage/Damage.h"             // Component damage and fire events
#include "Cockpit/ParamBridge.h"       // Named cockpit parameters
#include "Aero/AeroAssembly.h"         // Aero coefficients and forces
#include "Systems/Propulsion.h"        // Fuel, engine indications, propulsion
#include "Systems/Gear.h"              // Gear command and weight on wheels
#include "Commands/CommandHandlers.h"  // Pilot commands
#include "FlightModel/Forces.h"        // Force and moment accumulation
#include "Weapons/Ccrp.h"              // Release solution and cues
#include "Weapons/ReleaseSequence.h"   // Bays, pickle and laser
#include "Diagnostics/EngineLog.h"     // Engine and yaw debug CSV
#include "FlightModel/AirData.h"       // Air-relative speed, Mach, dynamic pressure
#include "FlightModel/ControlLaws.h"   // Control-law drive and actuators
#include "Engine/Engine.h"             // Installed propulsion
#include "Engine/EngineStart.h"
#include "FlightControls/FlightControls.h"  // Control laws
#include "Avionics/Autothrottle.h"
#include "Avionics/Paars.h"
#include "Avionics/JdamEnvelope.h"
#include "Avionics/CockpitBase_Interop.h"
#include "Avionics/Weapons_Interop.h"

namespace
{

constexpr size_t mirrored_gear_draw_arg_required_size = 616;

constexpr int gear_arg_nose = 0;
constexpr int gear_arg_left = 3;
constexpr int gear_arg_right = 5;
constexpr int wow_arg_nose = 1;
constexpr int wow_arg_left = 4;
constexpr int wow_arg_right = 6;

constexpr int left_aileron_arg = 11;
constexpr int right_aileron_arg = 12;
constexpr int left_elevator_arg = 15;
constexpr int right_elevator_arg = 16;
constexpr int left_rudder_arg = 17;
constexpr int right_rudder_arg = 18;
constexpr int tailhook_arg = 25;
constexpr int weapon_bay_args[F117::weapon_bay_count] = {27, 28};  // left, right; each moves its door, then its trapeze
// The standard bay argument. Nothing in the model is animated by it, but DCS draws the stores only while it is
// open, so it reads open while either bay is. Without it the store of a bay that is open vanishes while the
// other bay is shut.
constexpr int weapon_bay_any_arg = 26;
constexpr int drag_chute_arg = 35;
constexpr int mirrored_gear_arg_nose = 611;
constexpr int mirrored_gear_arg_left = 614;
constexpr int mirrored_gear_arg_right = 616;

constexpr int fc3_cockpit_pitch_arg = 1000;
constexpr int fc3_cockpit_roll_arg = 1001;
constexpr int fc3_cockpit_throttle_arg = 1002;
constexpr int fc3_cockpit_rudder_arg = 1003;

constexpr double ground_start_idle_n2 = 62.0;
constexpr double air_start_throttle = 77.5;
constexpr double air_start_n2 = 92.0;
constexpr double max_thrust_newtons = 92000.0;
constexpr double pa_per_psi = 6894.757;
constexpr double airframe_oxygen_supply = 1000.0;
constexpr double default_flow_velocity = 100.0;

}  // namespace

// Accumulate a body-axis force for the current frame.
// The position parameter is ignored here; forces are applied at the current center of mass.
void add_local_force(const Vec3& force)
{
    common_force = Vec3(common_force.x + force.x, common_force.y + force.y, common_force.z + force.z);
}

// Accumulate a body-axis moment for the current frame.
void add_local_moment(const Vec3& moment)
{
    common_moment = Vec3(common_moment.x + moment.x, common_moment.y + moment.y, common_moment.z + moment.z);
}

// Return the accumulated body-axis force to DCS after the current simulation step.
void ed_fm_add_local_force(double& fx, double& fy, double& fz, double& application_x, double& application_y,
                           double& application_z)
{
    // The component model supplies its moments explicitly; do not add r cross F here.
    const Vec3 totals = common_force;
    const Vec3 application = center_of_gravity;
    fx = totals.x;
    fy = totals.y;
    fz = totals.z;
    application_x = application.x;
    application_y = application.y;
    application_z = application.z;
}

void ed_fm_add_global_force(double& fx, double& fy, double& fz, double& px, double& py, double& pz)
{
    fx = fy = fz = px = py = pz = 0.0;
}

void ed_fm_add_global_moment(double& mx, double& my, double& mz)
{
    mx = my = mz = 0.0;
}

// Return the accumulated body-axis moment to DCS after the current simulation step.
void ed_fm_add_local_moment(double& mx, double& my, double& mz)
{
    const Vec3 totals = common_moment;
    mx = totals.x;
    my = totals.y;
    mz = totals.z;
}

// Main FM update called once per DCS simulation step.
// `dt` is the current frame step in seconds.
static void simulate_aircraft_frame(double dt)
{
    Cockpit_interop::force_irads_indicator_active();
    Cockpit_interop::update_irads_sensor_frame(dt, position_world_cs.x, position_world_cs.y, position_world_cs.z,
                                               F117::aircraft_quaternion_x, F117::aircraft_quaternion_y,
                                               F117::aircraft_quaternion_z, F117::aircraft_quaternion_w);
    Weapons::publish_irads_lock_params();

    log_simulation_frame(dt, sqrt(ax_world * ax_world + F117::ay_world * F117::ay_world + az_world * az_world));

    // Clear the per-frame force and moment accumulators.
    common_force = Vec3();
    common_moment = Vec3();
    update_derived_damage_values();

    // Air-relative speed from the current wind estimate, then Mach and dynamic pressure.
    F117::update_air_data();

    //-----CONTROL DYNAMICS------------------------
    //
    // While PAARS flies the recovery the autopilot holds stay selected but do not act.
    if (!g_paars.engaged())
    {
        F117::update_autopilot_hold_state();
    }
    update_autothrottle(dt);
    update_paars();

    update_fuel_system(dt);

    // Cockpit damage reduces effective control input below 0.7 integrity.
    const double control_degradation = get_control_degradation();

    // Run the tuned pitch/roll/yaw control laws and clamp the commanded surfaces.
    F117::update_flight_control_commands(dt, control_degradation);

    const double tail_integrity = F117::update_airframe_actuators(dt);
    Weapons::update_weapon_release_sequence(dt);
    update_propulsion_and_control_effectiveness(dt, tail_integrity);
    log_engine_debug_sample();

    // Aerodynamic coefficient update and force accumulation
    update_aerodynamic_coefficients();
    apply_aerodynamic_forces_and_thrust();

    // Mark subsystem initialization complete after the first successful simulation frame.
    F117::sim_initialized = F117::Actuators::initialized = F117::Flight_controls::sim_initialized = true;

    update_weight_on_wheels();
}

void ed_fm_simulate(double dt)
{
    F117::delta_time = dt;
    simulate_aircraft_frame(dt);
}

void ed_fm_set_surface(double h, double h_obj, unsigned, double, double, double)
{
    F117::surface_height_valid = std::isfinite(h);
    if (F117::surface_height_valid)
        F117::surface_height_m = std::isfinite(h_obj) ? (std::max)(h, h_obj) : h;
}

void ed_fm_set_atmosphere(double altitude, double temperature, double, double density, double pressure, double wind_x,
                          double wind_y, double wind_z)
{
    // Air data and ballistic wind are separate from the body-frame wind callback.
    wind_world_cs = {wind_x, wind_y, wind_z};
    F117::altitude_valid = std::isfinite(altitude);
    F117::altitude_m = altitude;
    F117::ambient_pressure_pa = pressure;
    F117::ambient_density_kg_per_m3 = density;
    F117::ambient_temperature_deg_k = temperature;
}

void ed_fm_set_current_mass_state(double mass_kg, double cg_x, double cg_y, double cg_z, double roll_inertia,
                                  double yaw_inertia, double pitch_inertia)
{
    center_of_gravity = {cg_x, cg_y, cg_z};
    inertia = {roll_inertia, yaw_inertia, pitch_inertia};
    // Preserve the controller's calibrated mass-to-load scale during this source replacement.
    constexpr double controller_weight_per_kg = 9.98665002864;
    F117::weight_n = controller_weight_per_kg * mass_kg;
}
void ed_fm_set_current_state(double accel_x, double accel_y, double accel_z, double speed_x, double speed_y,
                             double speed_z, double pos_x, double pos_y, double pos_z, double, double, double, double,
                             double, double, double qx, double qy, double qz, double qw)
{
    position_world_cs = {pos_x, pos_y, pos_z};
    vx_world = speed_x;
    vy_world = speed_y;
    vz_world = speed_z;
    ax_world = accel_x;
    F117::ay_world = accel_y;
    az_world = accel_z;
    F117::aircraft_quaternion_x = qx;
    F117::aircraft_quaternion_y = qy;
    F117::aircraft_quaternion_z = qz;
    F117::aircraft_quaternion_w = qw;
    // Existing hold-law vertical-speed reference, in m/s.
    F117::vspeed = speed_y - 3.25;
}

double ax_body{};
double ay_body{};
double az_body{};
double vx_body{};
double vy_body{};
double vz_body{};

void ed_fm_set_current_state_body_axis(double accel_x, double accel_y, double accel_z, double speed_x, double speed_y,
                                       double speed_z, double wind_x, double wind_y, double wind_z, double, double,
                                       double, double rate_x, double rate_y, double rate_z, double, double pitch_rad,
                                       double roll_rad, double attack_rad, double slip_rad)
{
    // ED uses X forward, Y up, Z right. The control laws use positive yaw to the right.
    wind = {wind_x, wind_y, wind_z};
    velocity_world_cs = {speed_x, speed_y, speed_z};  // Historical name: this snapshot is body-frame.
    vx_body = speed_x;
    vy_body = speed_y;
    vz_body = speed_z;
    ax_body = accel_x;
    ay_body = accel_y;
    az_body = accel_z;
    F117::roll_rate_rps = rate_x;
    F117::pitch_rate_rps = rate_z;
    F117::yaw_rate_rps = -rate_y;
    F117::pitch_angle = F117::radians_to_degrees * pitch_rad;
    F117::roll_angle = F117::radians_to_degrees * roll_rad;
    F117::alpha_deg = F117::radians_to_degrees * attack_rad;
    F117::beta_deg = F117::radians_to_degrees * slip_rad;
    // Retain the load-axis names used by the controller interface.
    F117::az = accel_y;
    F117::ay = accel_z;
}

void ed_fm_set_command(int command, float value)
{
    // Route a command to its first owner. IRADS has priority over flight controls.
    using Axis_handler = bool (*)(int, float);
    const Axis_handler axes[] = {Cockpit_interop::handle_irads_command, handle_roll_command, handle_pitch_command,
                                 handle_yaw_command, handle_engine_and_throttle_command};
    for (const auto handle : axes)
        if (handle(command, value))
            return;
    using Switch_handler = bool (*)(int);
    const Switch_handler switches[] = {handle_actuator_toggle_command, handle_autopilot_command,
                                       handle_autothrottle_command, handle_paars_command};
    for (const auto handle : switches)
        if (handle(command))
            return;
    if (command == reset_trim)
        F117::pitch_trim = F117::roll_trim = F117::yaw_trim = 0.0;
}

bool ed_fm_change_mass(double& delta_mass, double& delta_mass_pos_x, double& delta_mass_pos_y, double& delta_mass_pos_z,
                       double& delta_mass_moment_of_inertia_x, double& delta_mass_moment_of_inertia_y,
                       double& delta_mass_moment_of_inertia_z)
{
    // DCS repeatedly drains this callback. Acknowledge each fuel delta exactly once.
    const double pending_kg = F117::internal_fuel - F117::fuel_mass_in_dcs;
    const bool report = std::isfinite(pending_kg) && std::abs(pending_kg) >= 0.01;
    delta_mass = report ? pending_kg : 0.0;
    delta_mass_pos_x = 0.0;
    delta_mass_pos_z = 0.0;
    delta_mass_pos_y = report ? 0.1 : 0.0;  // Empty-aircraft CG in FM/config.lua.
    delta_mass_moment_of_inertia_x = 0.0;
    delta_mass_moment_of_inertia_y = 0.0;
    delta_mass_moment_of_inertia_z = 0.0;
    if (report)
        F117::fuel_mass_in_dcs = F117::internal_fuel;
    return report;
}

void ed_fm_set_internal_fuel(double kilograms)
{
    F117::internal_fuel = kilograms;
}
double ed_fm_get_internal_fuel()
{
    return F117::internal_fuel;  // The F-117 carries no external fuel tanks.
}
void ed_fm_set_external_fuel(int, double, double, double, double)
{
    // No supported loadout has an external tank; DCS owns all weapon masses.
}
double ed_fm_get_external_fuel()
{
    return 0.0;  // No external tank stations.
}

void ed_fm_refueling_add_fuel(double kilograms)
{
    F117::internal_fuel = F117::internal_fuel + kilograms;
}

namespace
{

void sync_gear_draw_args(float* drawargs)
{
    if (!F117::sim_initialized)
    {
        for (const int argument : {gear_arg_nose, gear_arg_left, gear_arg_right})
            drawargs[argument] = static_cast<float>(F117::Actuators::gear.position);
        return;
    }
    F117::Actuators::gear.position = drawargs[gear_arg_right];
}

void update_weight_on_wheels_from_draw_args(float* drawargs)
{
    const float contact = drawargs[wow_arg_nose] + drawargs[wow_arg_left] + drawargs[wow_arg_right];
    F117::wow_from_draw_args = contact > weight_on_wheels_draw_arg_threshold;
    if (!F117::wow_from_draw_args)
        return;
    F117::weight_on_wheels = F117::wow_from_draw_args;
    F117::gear_command = gear_command_down;
}

void update_control_surface_draw_args(float* drawargs)
{
    drawargs[left_aileron_arg] =
        static_cast<float>(limit((-F117::aileron_pct + (F117::roll_trim / 10) / (F117::mach + 1)), -0.75, 0.75));
    drawargs[right_aileron_arg] =
        static_cast<float>(limit((F117::aileron_pct + (F117::roll_trim / 10) / (F117::mach + 1)), -0.75, 0.75));
    drawargs[left_elevator_arg] = static_cast<float>(limit(-F117::elev_pos / (F117::mach + 1), -0.6, 0.6));
    drawargs[right_elevator_arg] = static_cast<float>(limit(-F117::elev_pos / (F117::mach + 1), -0.6, 0.6));
    drawargs[left_rudder_arg] = static_cast<float>(limit((F117::rudder_pos + (F117::yaw_trim / 10)), -0.75, 0.75));
    drawargs[right_rudder_arg] = static_cast<float>(limit((F117::rudder_pos + (F117::yaw_trim / 10)), -0.75, 0.75));
}

void update_misc_draw_args(float* drawargs, size_t size)
{
    drawargs[drag_chute_arg] = static_cast<float>(F117::Actuators::drag_chute.position);
    drawargs[tailhook_arg] = static_cast<float>(F117::Actuators::tailhook.position);
    float any_open = 0.0f;
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
    {
        const float position = static_cast<float>(limit(F117::bay_state[bay], 0.0, 1.0));
        drawargs[weapon_bay_args[bay]] = position;
        any_open = position > any_open ? position : any_open;
    }
    drawargs[weapon_bay_any_arg] = any_open;

    if (size > mirrored_gear_draw_arg_required_size)
    {
        drawargs[mirrored_gear_arg_nose] = drawargs[gear_arg_nose];
        drawargs[mirrored_gear_arg_left] = drawargs[gear_arg_left];
        drawargs[mirrored_gear_arg_right] = drawargs[gear_arg_right];
    }
}

void update_fc3_cockpit_draw_args(float* drawargs)
{
    drawargs[fc3_cockpit_roll_arg] = static_cast<float>(limit(F117::Flight_controls::lat_stick_input, -1.0, 1.0));
    drawargs[fc3_cockpit_pitch_arg] = static_cast<float>(limit(-F117::Flight_controls::long_stick_input, -1.0, 1.0));
    drawargs[fc3_cockpit_throttle_arg] = static_cast<float>(limit(F117::throttle_input, -1.0, 1.0));
    drawargs[fc3_cockpit_rudder_arg] = static_cast<float>(limit(F117::ped_input, -1.0, 1.0));
    drawargs[gear_arg_nose] = static_cast<float>(F117::gear_command);
    drawargs[gear_arg_left] = static_cast<float>(F117::gear_command);
    drawargs[gear_arg_right] = static_cast<float>(F117::gear_command);
}
}  // namespace

void ed_fm_set_draw_args_v2(float* drawargs,
                            size_t size)  // Sync external-model draw args between DCS and the FM state.
{
    // DCS owns damage visibility arguments. FM damage arrives via ed_fm_on_damage.
    sync_gear_draw_args(drawargs);
    update_weight_on_wheels_from_draw_args(drawargs);
    update_control_surface_draw_args(drawargs);
    update_misc_draw_args(drawargs, size);
}

// FC3 cockpit draw-arg callback used for stick, rudder pedal, throttle, and gear-handle animation.

void ed_fm_set_fc3_cockpit_draw_args_v2(float* arguments, size_t)
{
    update_fc3_cockpit_draw_args(arguments);
}

void ed_fm_configure([[maybe_unused]] const char* cfg_path)
{
    // Reserved DCS FM configuration hook; unused by this module.
}

namespace
{
bool try_get_suspension_param(unsigned index, double& value)
{
    const struct
    {
        unsigned id;
        double reading;
    } suspension[] = {
        {ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT, 0.0},
        {ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
        {ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
        {ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE, 0.0},
        {ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE, 0.0},
        {ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE, 0.0},
        {ED_FM_SUSPENSION_0_WHEEL_YAW, F117::rudder_pos * F117::nose_wheel_yaw_range_rad},
        {ED_FM_ANTI_SKID_ENABLE, 1.0},
        {ED_FM_SUSPENSION_0_GEAR_POST_STATE, F117::Actuators::gear.position},
        {ED_FM_SUSPENSION_1_GEAR_POST_STATE, F117::Actuators::gear.position},
        {ED_FM_SUSPENSION_2_GEAR_POST_STATE, F117::Actuators::gear.position},
        {ED_FM_SUSPENSION_0_DOWN_LOCK, F117::Actuators::gear.position},
        {ED_FM_FC3_GEAR_HANDLE_POS, F117::gear_command},
    };
    for (const auto& parameter : suspension)
        if (parameter.id == index)
        {
            value = parameter.reading;
            return true;
        }
    return false;
}

bool try_get_engine_param(unsigned index, double& value)
{
    if (index > ED_FM_END_ENGINE_BLOCK)
    {
        return false;
    }
    switch (index)
    {
        case ED_FM_FC3_THROTTLE_LEFT:
        case ED_FM_FC3_THROTTLE_RIGHT:
            value = limit(F117::throttle_input / 100.0, 0.0, 1.0);
            return true;
    }

    // Each engine has a block of parameters. Block 0 is the APU, which isn't reported; 1 is left, 2 is right.
    const unsigned block_size = ED_FM_ENGINE_1_RPM - ED_FM_ENGINE_0_RPM;
    const unsigned block = index / block_size;
    const unsigned field = index - block * block_size;
    if (block == 0)
    {
        switch (field)
        {
            case ED_FM_ENGINE_0_RPM:
            case ED_FM_ENGINE_0_RELATED_RPM:
            case ED_FM_ENGINE_0_THRUST:
            case ED_FM_ENGINE_0_RELATED_THRUST:
                value = 0.0;
                return true;
        }
        return false;
    }
    if (block > 2)
    {
        return false;
    }
    const int side = block == 1 ? F117::Engine_start::left : F117::Engine_start::right;
    // N2 is reported whenever the core turns: on the starter, while lit, and spooling down after shutdown.
    const double norm_n2 = limit(F117::engines[side].n2 / 100.0, 0.0, 1.0);
    switch (field)
    {
        case ED_FM_ENGINE_0_RPM:
        case ED_FM_ENGINE_0_RELATED_RPM:
        case ED_FM_ENGINE_0_CORE_RELATED_RPM:
            value = norm_n2;
            return true;
        case ED_FM_ENGINE_0_THRUST:
            value = F117::engine_thrust_n[side];
            return true;
        case ED_FM_ENGINE_0_RELATED_THRUST:
            value = limit(F117::engine_thrust_n[side] / (max_thrust_newtons / 2.0), 0.0, 1.0);
            return true;
        case ED_FM_ENGINE_0_TEMPERATURE:
            value = std::isfinite(F117::engine_egt_c[side]) ? F117::engine_egt_c[side] : 0.0;
            return true;
        case ED_FM_ENGINE_0_OIL_PRESSURE:
            value = F117::engine_oil_psi[side] * pa_per_psi;
            return true;
        case ED_FM_ENGINE_0_FUEL_FLOW:
            value = F117::engine_fuel_flow_kg_s[side];
            return true;
        case ED_FM_ENGINE_0_COMBUSTION:
            value = g_engine_starts.engine[side].lit() ? 1.0 : 0.0;
            return true;
    }

    return false;
}

bool try_get_misc_param(unsigned index, double& value)
{
    const struct
    {
        unsigned id;
        double reading;
    } cockpit[] = {
        {ED_FM_FUEL_INTERNAL_FUEL, F117::internal_fuel},
        {ED_FM_FUEL_TOTAL_FUEL, F117::internal_fuel},
        {ED_FM_OXYGEN_SUPPLY, airframe_oxygen_supply},
        {ED_FM_FLOW_VELOCITY, default_flow_velocity},
        {ED_FM_FC3_STICK_PITCH, F117::Flight_controls::long_stick_input},
        {ED_FM_FC3_STICK_ROLL, F117::Flight_controls::lat_stick_input},
        {ED_FM_FC3_RUDDER_PEDALS, F117::ped_input},
        {ED_FM_FC3_AUTOPILOT_STATUS, static_cast<double>(F117::alt_hold)},
    };
    for (const auto& parameter : cockpit)
        if (parameter.id == index)
        {
            value = parameter.reading;
            return true;
        }
    return false;
}
}  // namespace

double ed_fm_get_param(unsigned index)
{
    double value = 0.0;

    if (try_get_suspension_param(index, value))
    {
        return value;
    }

    if (try_get_engine_param(index, value))
    {
        return value;
    }

    if (try_get_misc_param(index, value))
    {
        return value;
    }

    return 0;
}

namespace
{

void reset_session_controls()
{
    F117::sim_initialized = F117::Actuators::initialized = false;
    F117::Flight_controls::reset_runtime_state();
    F117::alt_hold = F117::altroll_hold = F117::horiz_hold = 0;
    F117::air_refuel_door_open = false;
    F117::pitch_trim = F117::roll_trim = F117::yaw_trim = 0.0;
    F117::ped_input = F117::wheel_brake_command = 0.0;
    F117::nws_authority_held = false;
    F117::nose_wheel_yaw_range_rad = F117::nose_wheel_yaw_low_rad;
    for (double* input : {&F117::Flight_controls::lat_stick_input, &F117::Flight_controls::long_stick_input,
                          &F117::Flight_controls::long_stick_force})
        *input = 0.0;
    F117::throttle_input = F117::throttle_state = F117::thrust_n = 0.0;
    F117::engineswitch = false;
    g_engine_starts.reset(false);
    for (int side = 0; side < 2; ++side)
    {
        F117::engines[side] = F117::Engine::Engine_state();
        F117::engine_thrust_n[side] = F117::engine_fuel_flow_kg_s[side] = F117::engine_oil_psi[side] = 0.0;
        F117::engine_egt_c[side] = NAN;
    }
    F117::elev_pos = F117::rudder_pos = 0.0f;
    F117::elevator_deg = F117::aileron_deg = F117::rudder_deg = 0.0;
    F117::elevator_deg_commanded = F117::aileron_deg_commanded = F117::rudder_deg_commanded = 0.0;
    F117::elevator_pct = F117::aileron_pct = F117::rudder_pct = 0.0;
    F117::rolling_friction = wheel_brake_off_friction;
    F117::tailhook_command = F117::tailhook_pos = 0.0;
    F117::dragchute_command = F117::dragchute = 0.0;
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
        F117::bay_cmd[bay] = F117::bay_state[bay] = 0.0f;
    F117::surface_height_valid = F117::altitude_valid = false;
    for (bool& wow : F117::suspension_wow)
        wow = false;
    F117::Actuators::reset_all();
    g_autothrottle.reset();
    g_paars.reset();
    g_paars_engaged_autothrottle = false;
    g_paars_roll_rate_deg_s = 0.0;
}

void apply_ground_start_state(bool engines_running)
{
    reset_session_controls();
    F117::Actuators::gear.set(gear_command_down);
    F117::gear_down = gear_command_down;
    F117::gear_command = gear_command_down;
    F117::weight_on_wheels = F117::wow_from_draw_args = true;
    F117::throttle_input = 0.0;
    F117::engineswitch = engines_running;
    g_engine_starts.reset(engines_running);
    F117::rolling_friction = wheel_brake_off_friction;
    F117::engines[0].n2 = F117::engines[1].n2 = engines_running ? ground_start_idle_n2 : 0.0;
    reset_damage_and_events();
    Weapons::reset_weapon_release_sequence();
    Weapons::reset_laser();
}

void apply_air_start_state()
{
    reset_session_controls();
    F117::gear_down = gear_command_up;
    F117::gear_command = gear_command_up;
    F117::weight_on_wheels = F117::wow_from_draw_args = false;
    F117::throttle_input = air_start_throttle;
    F117::throttle_state = air_start_throttle;
    g_engine_starts.reset(true);
    F117::engineswitch = g_engine_starts.any_running();
    F117::rolling_friction = wheel_brake_off_friction;
    F117::engines[0].n2 = F117::engines[1].n2 = air_start_n2;
    reset_damage_and_events();
    Weapons::reset_weapon_release_sequence();
    Weapons::reset_laser();
}
}  // namespace

// Reset transient FM state when the aircraft is destroyed, restarted, or unloaded.

void ed_fm_release(void)
{
    reset_session_controls();
    // DCS will collect the next aircraft's initial fuel mass afresh.
    F117::fuel_mass_in_dcs = F117::delta_time = 0.0;
    F117::gear_down = F117::gear_command = 0.0;
    F117::weight_on_wheels = F117::wow_from_draw_args = false;
    reset_damage_and_events();
    Weapons::reset_weapon_release_sequence();
    Weapons::reset_laser();
    Cockpit_interop::reset_irads_session();
}

// Conditions to make the screen shake in first-person view.
double ed_fm_get_shake_amplitude(void)
{
    const double engine_loss = 1.0 - (g_damage.left_engine + g_damage.right_engine) / 2.0;
    // Highest-priority active source wins, matching the aircraft's existing shake schedule.
    const struct
    {
        bool active;
        double amplitude;
    } sources[] = {
        {g_damage.left_wing < 0.2 || g_damage.right_wing < 0.2, 10.0},
        {g_damage.cockpit < 0.5, 2.0 * (1.0 - g_damage.cockpit)},
        {engine_loss > 0.3, engine_loss * 5.0},
        {F117::az > 80.0, F117::az / 150.0},
        {F117::mach >= 0.1 && F117::alpha_deg > 45.0, F117::alpha_deg / 100.0},
        {F117::mach >= 0.05 && F117::beta_deg > 15.0, F117::beta_deg / 50.0},
    };
    for (const auto& source : sources)
        if (source.active)
            return source.amplitude;
    return 0.0;
}

// Optional DCS easy-flight hook. This FM currently leaves game-mode behavior unchanged.
void ed_fm_set_easy_flight(bool)
{
    // There is one control-law configuration for both DCS difficulty settings.
}

// DCS hook for the unlimited-fuel option.
void ed_fm_unlimited_fuel(bool enabled)
{
    F117::param_class.set_unlimited_fuel(enabled);
}

// Cold-start initialization.

void ed_fm_cold_start()
{
    apply_ground_start_state(false);
    log_damage_lifecycle_event("ed_fm_cold_start");
    Cockpit_interop::inject_irads_sensor_late();
}

// Ground hot-start initialization.

void ed_fm_hot_start()
{
    apply_ground_start_state(true);
    log_damage_lifecycle_event("ed_fm_hot_start");
    Cockpit_interop::inject_irads_sensor_late();
}

// Air-start initialization.

void ed_fm_hot_start_in_air()
{
    apply_air_start_state();
    log_damage_lifecycle_event("ed_fm_hot_start_in_air");
    Cockpit_interop::inject_irads_sensor_late();
}

// Damage callback. DCS reports remaining integrity from 1.0 intact down to 0.1 destroyed.

// Restore repairable FM damage state and clear queued events.

// -----------------------------------------------------------------------
// New API functions (DCS 2.5.7+ / July 2025 SDK) - stubs
// DCS calls these via GetProcAddress; returning safely is sufficient if unused.
// -----------------------------------------------------------------------

// Called asynchronously with cloud/precipitation density at the aircraft position.
void ed_fm_set_clouds_density(const atmo_clouds_and_precipation&)
{
}

// DCS requests the set of world points at which it should sample wind for this FM.
// Set in_out.field = nullptr to opt out.
void ed_fm_wind_vector_field_update_request(wind_vector_field& in_out)
{
    in_out.field = nullptr;
    in_out.field_points_count = 0;
}

// Called after DCS has filled all wind sample points requested above.
void ed_fm_wind_vector_field_done()
{
}

// DCS pushes in-game events (e.g. carrier cat/trap) to the FM.
bool ed_fm_push_simulation_event(const ed_fm_simulation_event&)
{
    return false;
}

// DCS feeds back per-gear suspension state each frame.
// idx: 0 = nose gear, 1 = left main, 2 = right main.
void ed_fm_suspension_feedback(int idx, const ed_fm_suspension_info* info)
{
    if (!info || idx < 0 || idx > 2)
        return;
    F117::suspension_wow[idx] = (info->struct_compression > 0.001);
}
