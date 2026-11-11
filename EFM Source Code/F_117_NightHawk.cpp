#include "stdafx.h"
#include "F_117_NightHawk.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <string>

#include "EfmMath.h"
#include "Inputs.h" // DCS and aircraft command protocol
#include "param_functions.h" // Mission options
#include "include/Cockpit/CockpitAPI_Declare.h" // Lua parameter bridge
#include "Aerodynamics/AeroModel.h"
#include "Actuators/Actuators.h" // Airframe motion
#include "Atmosphere/Atmosphere.h" // Air data
#include "FlightModel/State.h" // Shared flight-model state
#include "Damage/Damage.h" // Component damage and fire events
#include "Cockpit/ParamBridge.h" // Named cockpit parameters
#include "Aero/AeroAssembly.h" // Aero coefficients and forces
#include "Systems/Propulsion.h" // Fuel, engine indications, propulsion
#include "Systems/Gear.h" // Gear command and weight on wheels
#include "Commands/CommandHandlers.h" // Pilot commands
#include "FlightModel/Forces.h" // Force and moment accumulation
#include "Weapons/Ccrp.h" // Release solution and cues
#include "Weapons/ReleaseSequence.h" // Bays, pickle and laser
#include "Diagnostics/EngineLog.h" // Engine and yaw debug CSV
#include "FlightModel/AirData.h" // Air-relative speed, Mach, dynamic pressure
#include "FlightModel/ControlLaws.h" // Control-law drive and actuators
#include "Engine/Engine.h" // Installed propulsion
#include "Engine/EngineStart.h"
#include "FlightControls/FlightControls.h" // Control laws
#include "Avionics/Autothrottle.h"
#include "Avionics/Paars.h"
#include "Avionics/JdamEnvelope.h"
#include "Avionics/CockpitBase_Interop.h"
#include "Avionics/Weapons_Interop.h"

using namespace F117;
using namespace weapons; // Shared names used by the subsystem adapters below.



namespace
{



    constexpr size_t kMirroredGearDrawArgRequiredSize = 616;

    constexpr int kGearArgNose = 0;
    constexpr int kGearArgLeft = 3;
    constexpr int kGearArgRight = 5;
    constexpr int kWowArgNose = 1;
    constexpr int kWowArgLeft = 4;
    constexpr int kWowArgRight = 6;

    constexpr int kLeftAileronArg = 11;
    constexpr int kRightAileronArg = 12;
    constexpr int kLeftElevatorArg = 15;
    constexpr int kRightElevatorArg = 16;
    constexpr int kLeftRudderArg = 17;
    constexpr int kRightRudderArg = 18;
    constexpr int kTailhookArg = 25;
    constexpr int kWeaponBayArgs[kWeaponBayCount] = {27, 28}; // left, right; each moves its door, then its trapeze
    // The standard bay argument. Nothing in the model is animated by it, but DCS draws the stores only while it is
    // open, so it reads open while either bay is. Without it the store of a bay that is open vanishes while the
    // other bay is shut.
    constexpr int kWeaponBayAnyArg = 26;
    constexpr int kDragChuteArg = 35;
    constexpr int kMirroredGearArgNose = 611;
    constexpr int kMirroredGearArgLeft = 614;
    constexpr int kMirroredGearArgRight = 616;



    constexpr int kFc3CockpitPitchArg = 1000;
    constexpr int kFc3CockpitRollArg = 1001;
    constexpr int kFc3CockpitThrottleArg = 1002;
    constexpr int kFc3CockpitRudderArg = 1003;

    constexpr double kGroundStartIdleN2 = 62.0;
    constexpr double kAirStartThrottle = 77.5;
    constexpr double kAirStartN2 = 92.0;
    constexpr double kMaxThrustNewtons = 92000.0;
    constexpr double kPaPerPsi = 6894.757;
    constexpr double kAirframeOxygenSupply = 1000.0;
    constexpr double kDefaultFlowVelocity = 100.0;





}


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
void ed_fm_add_local_force(double& fx, double& fy, double& fz,
    double& applicationX, double& applicationY, double& applicationZ)
{
    // The component model supplies its moments explicitly; do not add r cross F here.
    const Vec3 totals = common_force;
    const Vec3 application = center_of_gravity;
    fx = totals.x; fy = totals.y; fz = totals.z;
    applicationX = application.x; applicationY = application.y; applicationZ = application.z;
}

void ed_fm_add_global_force(double& fx, double& fy, double& fz,
    double& px, double& py, double& pz)
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
    mx = totals.x; my = totals.y; mz = totals.z;
}



// Main FM update called once per DCS simulation step.
// `dt` is the current frame step in seconds.
static void simulate_aircraft_frame(double dt)
{
    CockpitInterop::ForceIRADSIndicatorActive();
    CockpitInterop::UpdateIRADSSensorFrame(dt,
        position_world_cs.x, position_world_cs.y, position_world_cs.z,
        F117::aircraftQuaternionX, F117::aircraftQuaternionY,
        F117::aircraftQuaternionZ, F117::aircraftQuaternionW);
    publish_irads_lock_params();

    log_simulation_frame(dt, sqrt(ax_world * ax_world + F117::ay_world * F117::ay_world + az_world * az_world));

    // Clear the per-frame force and moment accumulators.
    common_force = Vec3();
    common_moment = Vec3();
    update_derived_damage_values();

    // Air-relative speed from the current wind estimate, then Mach and dynamic pressure.
    update_air_data();

    //-----CONTROL DYNAMICS------------------------
    //
    // While PAARS flies the recovery the autopilot holds stay selected but do not act.
    if (!g_paars.engaged())
    {
        update_autopilot_hold_state();
    }
    update_autothrottle(dt);
    update_paars();

    update_fuel_system(dt);

    // Cockpit damage reduces effective control input below 0.7 integrity.
    const double controlDegradation = get_control_degradation();

    // Run the tuned pitch/roll/yaw control laws and clamp the commanded surfaces.
    update_flight_control_commands(dt, controlDegradation);

    const double tailIntegrity = update_airframe_actuators(dt);
    update_weapon_release_sequence(dt);
    update_propulsion_and_control_effectiveness(dt, tailIntegrity);
    log_engine_debug_sample();

    // Aerodynamic coefficient update and force accumulation
    update_aerodynamic_coefficients();
    apply_aerodynamic_forces_and_thrust();

    // Mark subsystem initialization complete after the first successful simulation frame.
    F117::simInitialized = F117::ACTUATORS::initialized = F117::FLIGHTCONTROLS::simInitialized = true;

    update_weight_on_wheels();
}

void ed_fm_simulate(double dt)
{
    DeltaTime = dt;
    simulate_aircraft_frame(dt);
}

void ed_fm_set_surface(double h, double h_obj, unsigned /*surface_type*/,
                       double /*normal_x*/, double /*normal_y*/, double /*normal_z*/)
{
    F117::surface_height_valid = std::isfinite(h);
    if (F117::surface_height_valid)
        F117::surface_height_m = std::isfinite(h_obj) ? (std::max)(h, h_obj) : h;
}

void ed_fm_set_atmosphere(double altitude, double temperature, double /*soundSpeed*/,
    double density, double pressure, double windX, double windY, double windZ)
{
    // Air data and ballistic wind are separate from the body-frame wind callback.
    wind_world_cs = {windX, windY, windZ};
    F117::altitude_valid = std::isfinite(altitude);
    F117::altitude_m = altitude;
    F117::ambientPressure_PA = pressure;
    F117::ambientDensity_KgPerM3 = density;
    F117::ambientTemperature_DegK = temperature;
}

void ed_fm_set_current_mass_state(double massKg, double cgX, double cgY, double cgZ,
    double rollInertia, double yawInertia, double pitchInertia)
{
    center_of_gravity = {cgX, cgY, cgZ};
    inertia = {rollInertia, yawInertia, pitchInertia};
    // Preserve the controller's calibrated mass-to-load scale during this source replacement.
    constexpr double controllerWeightPerKg = 9.98665002864;
    F117::weight_N = controllerWeightPerKg * massKg;
}
void ed_fm_set_current_state(double accelX, double accelY, double accelZ,
    double speedX, double speedY, double speedZ, double posX, double posY, double posZ,
    double /*angularAccelX*/, double /*angularAccelY*/, double /*angularAccelZ*/,
    double /*rateX*/, double /*rateY*/, double /*rateZ*/,
    double qx, double qy, double qz, double qw)
{
    position_world_cs = {posX, posY, posZ};
    vx_world = speedX; vy_world = speedY; vz_world = speedZ;
    ax_world = accelX; F117::ay_world = accelY; az_world = accelZ;
    F117::aircraftQuaternionX = qx; F117::aircraftQuaternionY = qy;
    F117::aircraftQuaternionZ = qz; F117::aircraftQuaternionW = qw;
    // Existing hold-law vertical-speed reference, in m/s.
    F117::vspeed = speedY - 3.25;
}

double ax_body{};
double ay_body{};
double az_body{};
double vx_body{};
double vy_body{};
double vz_body{};

void ed_fm_set_current_state_body_axis(double accelX, double accelY, double accelZ,
    double speedX, double speedY, double speedZ, double windX, double windY, double windZ,
    double /*angularAccelX*/, double /*angularAccelY*/, double /*angularAccelZ*/,
    double rateX, double rateY, double rateZ, double /*heading*/, double pitchRad, double rollRad,
    double attackRad, double slipRad)
{
    // ED uses X forward, Y up, Z right. The control laws use positive yaw to the right.
    wind = {windX, windY, windZ};
    velocity_world_cs = {speedX, speedY, speedZ}; // Historical name: this snapshot is body-frame.
    vx_body = speedX; vy_body = speedY; vz_body = speedZ;
    ax_body = accelX; ay_body = accelY; az_body = accelZ;
    F117::rollRate_RPS = rateX; F117::pitchRate_RPS = rateZ; F117::yawRate_RPS = -rateY;
    F117::pitch_angle = radiansToDegrees * pitchRad;
    F117::roll_angle = radiansToDegrees * rollRad;
    F117::alpha_DEG = radiansToDegrees * attackRad;
    F117::beta_DEG = radiansToDegrees * slipRad;
    // Retain the load-axis names used by the controller interface.
    F117::az = accelY; F117::ay = accelZ;
}


void ed_fm_set_command(int command, float value)
{
    // Route a command to its first owner. IRADS has priority over flight controls.
    using AxisHandler = bool (*)(int, float);
    const AxisHandler axes[] = {CockpitInterop::HandleIRADSCommand, handle_roll_command,
        handle_pitch_command, handle_yaw_command, handle_engine_and_throttle_command};
    for (const auto handle : axes) if (handle(command, value)) return;
    using SwitchHandler = bool (*)(int);
    const SwitchHandler switches[] = {handle_actuator_toggle_command, handle_autopilot_command,
        handle_autothrottle_command, handle_paars_command};
    for (const auto handle : switches) if (handle(command)) return;
    if (command == resetTrim) pitchTrim = rollTrim = yawTrim = 0.0;
}


bool ed_fm_change_mass(double& delta_mass, double& delta_mass_pos_x,
    double& delta_mass_pos_y, double& delta_mass_pos_z,
    double& delta_mass_moment_of_inertia_x, double& delta_mass_moment_of_inertia_y,
    double& delta_mass_moment_of_inertia_z)
{
    // DCS repeatedly drains this callback. Acknowledge each fuel delta exactly once.
    const double pendingKg = F117::internal_fuel - F117::fuel_mass_in_dcs;
    const bool report = std::isfinite(pendingKg) && std::abs(pendingKg) >= 0.01;
    delta_mass = report ? pendingKg : 0.0;
    delta_mass_pos_x = 0.0; delta_mass_pos_z = 0.0;
    delta_mass_pos_y = report ? 0.1 : 0.0; // Empty-aircraft CG in FM/config.lua.
    delta_mass_moment_of_inertia_x = 0.0;
    delta_mass_moment_of_inertia_y = 0.0;
    delta_mass_moment_of_inertia_z = 0.0;
    if (report) F117::fuel_mass_in_dcs = F117::internal_fuel;
    return report;
}

void ed_fm_set_internal_fuel(double kilograms)
{
    F117::internal_fuel = kilograms;
}
double ed_fm_get_internal_fuel()
{
    return F117::internal_fuel; // The F-117 carries no external fuel tanks.
}
void ed_fm_set_external_fuel(int /*station*/, double /*kilograms*/,
    double /*x*/, double /*y*/, double /*z*/)
{
    // No supported loadout has an external tank; DCS owns all weapon masses.
}
double ed_fm_get_external_fuel ()
{
    return 0.0; // No external tank stations.
}

void ed_fm_refueling_add_fuel(double kilograms)
{
    F117::internal_fuel = F117::internal_fuel + kilograms;
}


namespace
{


    void sync_gear_draw_args(float* drawargs)
    {
        if (!F117::simInitialized)
        {
            for (const int argument : {kGearArgNose, kGearArgLeft, kGearArgRight})
                drawargs[argument] = static_cast<float>(F117::ACTUATORS::gear.position);
            return;
        }
        F117::ACTUATORS::gear.position = drawargs[kGearArgRight];
    }

    void update_weight_on_wheels_from_draw_args(float* drawargs)
    {
        const float contact = drawargs[kWowArgNose] + drawargs[kWowArgLeft] + drawargs[kWowArgRight];
        F117::wow_from_draw_args = contact > kWeightOnWheelsDrawArgThreshold;
        if (!F117::wow_from_draw_args) return;
        F117::weight_on_wheels = F117::wow_from_draw_args;
        F117::GearCommand = kGearCommandDown;
    }

    void update_control_surface_draw_args(float* drawargs)
    {
        drawargs[kLeftAileronArg] = static_cast<float>(limit((-aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75));
        drawargs[kRightAileronArg] = static_cast<float>(limit((aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75));
        drawargs[kLeftElevatorArg] = static_cast<float>(limit(-elev_pos / (mach + 1), -0.6, 0.6));
        drawargs[kRightElevatorArg] = static_cast<float>(limit(-elev_pos / (mach + 1), -0.6, 0.6));
        drawargs[kLeftRudderArg] = static_cast<float>(limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75));
        drawargs[kRightRudderArg] = static_cast<float>(limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75));
    }

    void update_misc_draw_args(float* drawargs, size_t size)
    {
        drawargs[kDragChuteArg] = static_cast<float>(ACTUATORS::dragChute.position);
        drawargs[kTailhookArg] = static_cast<float>(ACTUATORS::tailhook.position);
        float anyOpen = 0.0f;
        for (int bay = 0; bay < kWeaponBayCount; ++bay)
        {
            const float position = static_cast<float>(limit(F117::bay_state[bay], 0.0, 1.0));
            drawargs[kWeaponBayArgs[bay]] = position;
            anyOpen = position > anyOpen ? position : anyOpen;
        }
        drawargs[kWeaponBayAnyArg] = anyOpen;

        if (size > kMirroredGearDrawArgRequiredSize)
        {
            drawargs[kMirroredGearArgNose] = drawargs[kGearArgNose];
            drawargs[kMirroredGearArgLeft] = drawargs[kGearArgLeft];
            drawargs[kMirroredGearArgRight] = drawargs[kGearArgRight];
        }
    }



    void update_fc3_cockpit_draw_args(float* drawargs)
    {
        drawargs[kFc3CockpitRollArg] = static_cast<float>(limit(F117::FLIGHTCONTROLS::latStickInput, -1.0, 1.0));
        drawargs[kFc3CockpitPitchArg] = static_cast<float>(limit(-F117::FLIGHTCONTROLS::longStickInput, -1.0, 1.0));
        drawargs[kFc3CockpitThrottleArg] = static_cast<float>(limit(F117::throttleInput, -1.0, 1.0));
        drawargs[kFc3CockpitRudderArg] = static_cast<float>(limit(F117::pedInput, -1.0, 1.0));
        drawargs[kGearArgNose] = static_cast<float>(F117::GearCommand);
        drawargs[kGearArgLeft] = static_cast<float>(F117::GearCommand);
        drawargs[kGearArgRight] = static_cast<float>(F117::GearCommand);
    }
}


void ed_fm_set_draw_args_v2(float* drawargs, size_t size) // Sync external-model draw args between DCS and the FM state.
{
    // DCS owns damage visibility arguments. FM damage arrives via ed_fm_on_damage.
    sync_gear_draw_args(drawargs);
    update_weight_on_wheels_from_draw_args(drawargs);
    update_control_surface_draw_args(drawargs);
    update_misc_draw_args(drawargs, size);
}


// FC3 cockpit draw-arg callback used for stick, rudder pedal, throttle, and gear-handle animation.

void ed_fm_set_fc3_cockpit_draw_args_v2(float* arguments, size_t /*count*/)
{
    update_fc3_cockpit_draw_args(arguments);
}

void ed_fm_configure([[maybe_unused]] const char * cfg_path)
{
	// Reserved DCS FM configuration hook; unused by this module.
}


namespace
{
    bool try_get_suspension_param(unsigned index, double& value)
    {
        const struct { unsigned id; double reading; } suspension[] = {
            {ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT, 0.0},
            {ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
            {ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
            {ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_0_WHEEL_YAW, F117::rudder_pos * F117::noseWheelYawRange_RAD},
            {ED_FM_ANTI_SKID_ENABLE, 1.0},
            {ED_FM_SUSPENSION_0_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_1_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_2_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_0_DOWN_LOCK, F117::ACTUATORS::gear.position},
            {ED_FM_FC3_GEAR_HANDLE_POS, F117::GearCommand},
        };
        for (const auto& parameter : suspension)
            if (parameter.id == index) { value = parameter.reading; return true; }
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
            value = limit(F117::throttleInput / 100.0, 0.0, 1.0);
            return true;
        }

        // Each engine has a block of parameters. Block 0 is the APU, which isn't reported; 1 is left, 2 is right.
        const unsigned blockSize = ED_FM_ENGINE_1_RPM - ED_FM_ENGINE_0_RPM;
        const unsigned block = index / blockSize;
        const unsigned field = index - block * blockSize;
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
        const int side = block == 1 ? F117::ENGINE_START::LEFT : F117::ENGINE_START::RIGHT;
        // N2 is reported whenever the core turns: on the starter, while lit, and spooling down after shutdown.
        const double normN2 = limit(F117::engines[side].N2 / 100.0, 0.0, 1.0);
        switch (field)
        {
        case ED_FM_ENGINE_0_RPM:
        case ED_FM_ENGINE_0_RELATED_RPM:
        case ED_FM_ENGINE_0_CORE_RELATED_RPM:
            value = normN2;
            return true;
        case ED_FM_ENGINE_0_THRUST:
            value = F117::engine_thrust_N[side];
            return true;
        case ED_FM_ENGINE_0_RELATED_THRUST:
            value = limit(F117::engine_thrust_N[side] / (kMaxThrustNewtons / 2.0), 0.0, 1.0);
            return true;
        case ED_FM_ENGINE_0_TEMPERATURE:
            value = std::isfinite(F117::engine_egt_C[side]) ? F117::engine_egt_C[side] : 0.0;
            return true;
        case ED_FM_ENGINE_0_OIL_PRESSURE:
            value = F117::engine_oil_psi[side] * kPaPerPsi;
            return true;
        case ED_FM_ENGINE_0_FUEL_FLOW:
            value = F117::engine_fuel_flow_kg_s[side];
            return true;
        case ED_FM_ENGINE_0_COMBUSTION:
            value = g_engineStarts.engine[side].lit() ? 1.0 : 0.0;
            return true;
        }

        return false;
    }

    bool try_get_misc_param(unsigned index, double& value)
    {
        const struct { unsigned id; double reading; } cockpit[] = {
            {ED_FM_FUEL_INTERNAL_FUEL, F117::internal_fuel},
            {ED_FM_FUEL_TOTAL_FUEL, F117::internal_fuel},
            {ED_FM_OXYGEN_SUPPLY, kAirframeOxygenSupply},
            {ED_FM_FLOW_VELOCITY, kDefaultFlowVelocity},
            {ED_FM_FC3_STICK_PITCH, F117::FLIGHTCONTROLS::longStickInput},
            {ED_FM_FC3_STICK_ROLL, F117::FLIGHTCONTROLS::latStickInput},
            {ED_FM_FC3_RUDDER_PEDALS, F117::pedInput},
            {ED_FM_FC3_AUTOPILOT_STATUS, static_cast<double>(F117::alt_hold)},
        };
        for (const auto& parameter : cockpit)
            if (parameter.id == index) { value = parameter.reading; return true; }
        return false;
    }
}


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
        F117::simInitialized = F117::ACTUATORS::initialized = false;
        F117::FLIGHTCONTROLS::reset_runtime_state();
        F117::alt_hold = F117::altroll_hold = F117::horiz_hold = 0;
        F117::airRefuelDoorOpen = false;
        F117::pitchTrim = F117::rollTrim = F117::yawTrim = 0.0;
        F117::pedInput = F117::WheelBrakeCommand = 0.0;
        F117::nwsAuthorityHeld = false;
        F117::noseWheelYawRange_RAD = F117::kNoseWheelYawLow_RAD;
        for (double* input : {&F117::FLIGHTCONTROLS::latStickInput,
            &F117::FLIGHTCONTROLS::longStickInput, &F117::FLIGHTCONTROLS::longStickForce}) *input = 0.0;
        F117::throttleInput = F117::throttle_state = F117::thrust_N = 0.0;
        F117::engineswitch = false;
        g_engineStarts.reset(false);
        for (int side = 0; side < 2; ++side)
        {
            F117::engines[side] = F117::ENGINE::EngineState();
            F117::engine_thrust_N[side] = F117::engine_fuel_flow_kg_s[side] = F117::engine_oil_psi[side] = 0.0;
            F117::engine_egt_C[side] = NAN;
        }
        F117::elev_pos = F117::rudder_pos = 0.0f;
        F117::elevator_DEG = F117::aileron_DEG = F117::rudder_DEG = 0.0;
        F117::elevator_DEG_commanded = F117::aileron_DEG_commanded = F117::rudder_DEG_commanded = 0.0;
        F117::elevator_PCT = F117::aileron_PCT = F117::rudder_PCT = 0.0;
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::tailhook_command = F117::tailhook_pos = 0.0;
        F117::dragchute_command = F117::dragchute = 0.0;
        for (int bay = 0; bay < kWeaponBayCount; ++bay) F117::bay_cmd[bay] = F117::bay_state[bay] = 0.0f;
        F117::surface_height_valid = F117::altitude_valid = false;
        for (bool& wow : F117::suspension_wow) wow = false;
        F117::ACTUATORS::reset_all();
        g_autothrottle.reset();
        g_paars.reset();
        g_paarsEngagedAutothrottle = false;
        g_paarsRollRateDegS = 0.0;
    }

    void apply_ground_start_state(bool enginesRunning)
    {
        reset_session_controls();
        F117::ACTUATORS::gear.set(kGearCommandDown);
        F117::gearDown = kGearCommandDown;
        F117::GearCommand = kGearCommandDown;
        F117::weight_on_wheels = F117::wow_from_draw_args = true;
        F117::throttleInput = 0.0;
        F117::engineswitch = enginesRunning;
        g_engineStarts.reset(enginesRunning);
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::engines[0].N2 = F117::engines[1].N2 = enginesRunning ? kGroundStartIdleN2 : 0.0;
        reset_damage_and_events();
        reset_weapon_release_sequence();
        reset_laser();
    }

    void apply_air_start_state()
    {
        reset_session_controls();
        F117::gearDown = kGearCommandUp;
        F117::GearCommand = kGearCommandUp;
        F117::weight_on_wheels = F117::wow_from_draw_args = false;
        F117::throttleInput = kAirStartThrottle;
        F117::throttle_state = kAirStartThrottle;
        g_engineStarts.reset(true);
        F117::engineswitch = g_engineStarts.any_running();
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::engines[0].N2 = F117::engines[1].N2 = kAirStartN2;
        reset_damage_and_events();
        reset_weapon_release_sequence();
        reset_laser();
    }
}

// Reset transient FM state when the aircraft is destroyed, restarted, or unloaded.

void ed_fm_release(void)
{
    reset_session_controls();
    // DCS will collect the next aircraft's initial fuel mass afresh.
    F117::fuel_mass_in_dcs = F117::DeltaTime = 0.0;
    F117::gearDown = F117::GearCommand = 0.0;
    F117::weight_on_wheels = F117::wow_from_draw_args = false;
    reset_damage_and_events();
    reset_weapon_release_sequence();
    reset_laser();
    CockpitInterop::ResetIRADSSession();
}


// Conditions to make the screen shake in first-person view.
double ed_fm_get_shake_amplitude(void)
{
    const double engineLoss = 1.0 - (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
    // Highest-priority active source wins, matching the aircraft's existing shake schedule.
    const struct { bool active; double amplitude; } sources[] = {
        {g_damage.leftWing < 0.2 || g_damage.rightWing < 0.2, 10.0},
        {g_damage.cockpit < 0.5, 2.0 * (1.0 - g_damage.cockpit)},
        {engineLoss > 0.3, engineLoss * 5.0},
        {F117::az > 80.0, F117::az / 150.0},
        {F117::mach >= 0.1 && F117::alpha_DEG > 45.0, F117::alpha_DEG / 100.0},
        {F117::mach >= 0.05 && F117::beta_DEG > 15.0, F117::beta_DEG / 50.0},
    };
    for (const auto& source : sources) if (source.active) return source.amplitude;
    return 0.0;
}

// Optional DCS easy-flight hook. This FM currently leaves game-mode behavior unchanged.
void ed_fm_set_easy_flight(bool /*enabled*/)
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
    CockpitInterop::InjectIRADSSensor_Late();
}


// Ground hot-start initialization.

void ed_fm_hot_start()
{
    apply_ground_start_state(true);
    log_damage_lifecycle_event("ed_fm_hot_start");
    CockpitInterop::InjectIRADSSensor_Late();
}


// Air-start initialization.

void ed_fm_hot_start_in_air()
{
    apply_air_start_state();
    log_damage_lifecycle_event("ed_fm_hot_start_in_air");
    CockpitInterop::InjectIRADSSensor_Late();
}

// Damage callback. DCS reports remaining integrity from 1.0 intact down to 0.1 destroyed.



// Restore repairable FM damage state and clear queued events.





// -----------------------------------------------------------------------
// New API functions (DCS 2.5.7+ / July 2025 SDK) - stubs
// DCS calls these via GetProcAddress; returning safely is sufficient if unused.
// -----------------------------------------------------------------------

// Called asynchronously with cloud/precipitation density at the aircraft position.
void ed_fm_set_clouds_density(const atmo_clouds_and_precipation & /*info*/)
{
}

// DCS requests the set of world points at which it should sample wind for this FM.
// Set in_out.field = nullptr to opt out.
void ed_fm_wind_vector_field_update_request(wind_vector_field & in_out)
{
	in_out.field = nullptr;
	in_out.field_points_count = 0;
}

// Called after DCS has filled all wind sample points requested above.
void ed_fm_wind_vector_field_done()
{
}

// DCS pushes in-game events (e.g. carrier cat/trap) to the FM.
bool ed_fm_push_simulation_event(const ed_fm_simulation_event & /*in*/)
{
	return false;
}

// DCS feeds back per-gear suspension state each frame.
// idx: 0 = nose gear, 1 = left main, 2 = right main.
void ed_fm_suspension_feedback(int idx, const ed_fm_suspension_info * info)
{
    if (!info || idx < 0 || idx > 2) return;
    F117::suspension_wow[idx] = (info->struct_compression > 0.001);
}
