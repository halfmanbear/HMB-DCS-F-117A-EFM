#include "stdafx.h"
#include "F_117_NightHawk.h"
#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include <Math.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <queue>
#include <fstream>
#include "UtilityFunctions.h"	// Utility help functions
#include "Inputs.h"

#include "include/Cockpit/CockpitAPI_Declare.h" // Provides param handle interfacing for use in lua
#include "include/FM/API_Declare.h"	// Provides all DCS related functions in this cpp file
// Model headers
#include "Actuators/Actuators.h"			//Actuators model functions
#include "Atmosphere/Atmosphere.h"			//Atmosphere model functions
#include "Aerodynamics/Aero.h"				//Aerodynamic model functions
#include "FlightControls/FlightControls.h"	//Flight Controls model functions
#include "Engine/Engine.h"					//Engine model functions
#include "param_functions.h"

using namespace F117;

//-----------------------------------------------------------------
// Per-frame accumulated body moments. Reset at the start of every simulation tick.
// Units = Newton * meter
//-----------------------------------------------------------------
Vec3	common_moment;							

Vec3	common_force;

Vec3    center_of_gravity;

Vec3	inertia;

Vec3	wind;

Vec3	velocity_world_cs;

//-------------------------------------------------------
// Aircraft simulation state
//-------------------------------------------------------
namespace F117 // Shared FM state. Many aero/control calculations still use the original imperial-unit tuning.
{
	double		meterToFoot	= 3.28084;					// Meter to foot conversion factor
	double		ambientTemperature_DegK = 0.0;			// Ambient temperature (kelvin)
	double		ambientDensity_KgPerM3	= 0.0;			// Ambient density (kg/m^3)
	double		wingSpan_FT				= 43.33;		// F-117A wing-span (ft)
	double		wingArea_FT2			= 912.7;		// F-117A wing area (ft^2)
	double		meanChord_FT			= 21.07;		// F-117A mean aerodynamic chord (ft)
	double		referenceCG_PCT			= 0.35;			// Reference center of mass as a % of wing chord
	double		actualCG_PCT			= 0.32;			// Actual center of mass as a % of wing chord
	double		Cm0						= 0.04;			// Pitch-up trim bias (positive = nose up) 0.0387 0.0287
	double		pi						= acos(-1.0);	// Pi (3.14159....)
	double		radiansToDegrees		= 180.0/pi;		// Conversion factor from radians to degrees
	double		inertia_Ix_KGM2			= 12874.0;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iy_KGM2			= 75673.6;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iz_KGM2			= 85552.1;		// Reference moment of inertia (kg/m^2)
	double		temp[9];								// Temporary array for holding look-up table results
	double		altitude_m				= 0.0;			// Absolute altitude above sea level (metres)
	double		altitude_FT				= 45000;		// Absolute altitude above sea level (ft)
	double		totalVelocity_FPS		= 0.0;			// Total velocity (always positive) (ft/s)
	double		alpha_DEG				= 0.0;			// Angle of attack (deg)
	double		beta_DEG				= 0.0;			// Slideslip angle (deg)
	double		rollRate_RPS			= 0.0;			// Body roll rate (rad/sec)
	double		pitchRate_RPS			= 0.0;			// Body pitch rate (rad/sec)
	double		yawRate_RPS				= 0.0;			// Body yaw rate (rad/sec)
	double		thrust_N				= 80415;		// Engine thrust (Newtons)

	double		elevator_DEG			= 0.0;			// Elevator deflection (deg)
	double		aileron_DEG				= 0.0;			// Aileron deflection (deg)
	double		rudder_DEG				= 0.0;			// Rudder deflection (deg)
	double		elevator_DEG_commanded	= 0.0;			// Commanded elevator deflection from control system (deg)
	double		aileron_DEG_commanded	= 0.0;			// Commanded aileron deflection from control system (deg)
	double		rudder_DEG_commanded	= 0.0;			// Commanded rudder deflection from control system (deg)
	double		pitchTrim				= 0.0;			// Pitch trim
	double		rollTrim				= 0.0;			// Roll trim
	double		yawTrim					= 0.0;			// Yaw trim
	double		roll_cmd				= 0.0;			// Aileron command
	double		throttle_state			= 0.1;			// Engine power state
	double		pedInput				= 0.0;			// Rudder pedal input command normalized (-1 to 1)
	double		throttleInput			= 0.1;			// Throttle input command normalized (-1 to 1)
	double		aileron_PCT				= 0.0;			// Aileron deflection as a percent of maximum (-1 to 1)
	double		rudder_PCT				= 0.0;			// Rudder deflection as a percent of maximum (-1 to 1)
	double		elevator_PCT			= 0.0;			// Elevator deflection as a percent of maximum (-1 to 1)
	float		elev_pos				= 0.0;			// Elevator/stabilator deflection
	double		leadingEdgeFlap_DEG		= 0.0;			// Leading edge slat deflection (deg)
	double		leadingEdgeFlap_PCT		= 0.0;			// Leading edge slat deflection as a percent of maximum (0 to 1)

	double		dynamicPressure_LBFT2	= 0.0;			// Dynamic pressure (lb/ft^2)
	double		mach					= 0.0;			// Air speed in Mach; 1 is the local speed of sound.
	double		ps_LBFT2				= 0.0;			// Ambient calculated pressure (lb/ft^2)
	bool		simInitialized			= false;		// Has the simulation gone through it's first run frame?
	double		gearDown				= 0.0;			// Is the gear currently down?
	double		az						= 0.0;			// This is the G force felt by the pilot, acting out the bottom of the aircraft (m/s^2), 1 is Earth's gravity.
	double		ay						= 0.0;			// Ay (per normal direction convention) out the right wing (m/s^2)
	double		weight_N				= 131222.538;	// Weight force of aircraft (N)
	double		ay_world				= 0.0;			// World referenced up/down acceleration (m/s^2)
	bool		weight_on_wheels		= false;		// Weight on wheels flag 
	bool		wow_from_draw_args		= false;		// Suspension contact state from DCS draw args

	double		rolling_friction		= 0.015;		// Relative wheel braking/rolling resistance reported to DCS.
	double		WheelBrakeCommand		= 0.0;			// Commanded wheel brake
	double		GearCommand				= 0.0;			// Commanded gear lever
	//double		airbrake_command		= 0.0;			// Air brakes/spoiler command
	//double		airbrakes				= 0.0;			// Are the air brakes/spoilers deployed?
	double		tailhook_command		= 0.0;			// Tail hook command
	double		tailhook_pos			= 0.0;			// Tailhook actuator position reported back to DCS.
	float		rudder_pos				= 0.0;			//Rudder(s) deflection
	float		misc_cmd				= 0.0;			// Generic misc actuator command used for the weapon bay animation.
	float		misc_state				= 0.0;
	float		misc_cmdH				= 0.0;			// Secondary misc actuator command used for the hook-related animation path.
	float		misc_stateH				= 0.0;

	// Integrity-based damage state: 1.0 = perfect, 0.0 = destroyed
	// Lua Damage table elements: [3]=COCKPIT, [10]=ENGINE001, [11]=MAIN
	struct DamageState {
		double leftWing    = 1.0;   // From Element 11 (MAIN)
		double rightWing   = 1.0;   // From Element 11 (MAIN)
		double leftEngine  = 1.0;   // From Element 10 (ENGINE001)
		double rightEngine = 1.0;   // From Element 10 (ENGINE001)
		double leftTail    = 1.0;   // From Element 11 (MAIN)
		double rightTail   = 1.0;   // From Element 11 (MAIN)
		double cockpit     = 1.0;   // From Element 3  (COCKPIT)

		// Derived values (updated each frame in ed_fm_simulate)
		double wingAsymmetry   = 0.0;  // leftWing - rightWing
		double totalWingLoss   = 0.0;  // average damage across both wings
		double totalTailLoss   = 0.0;  // average damage across both tails
		double engineAsymmetry = 0.0;  // leftEngine - rightEngine
	};

	double		pitch_angle = 0.0;		// Pitch angle relateive to the horizon in degrees, -90 to +90.
	double		roll_angle = 0.0;		// Roll angle relateive to the horizon in degrees, -90 to +90.
	double		vspeed = 0.0;			// Vertical speed in metres per second.

	int			alt_hold = 0;			// Altitude hold
	int			altroll_hold = 0;		// Altitude and bank hold
	int			horiz_hold = 0;			// Horizon hold
	
	double		DeltaTime				= 0.0;			// Delta time of the simulation, in seconds.
	bool		engineswitch			= true;			// Is the engine(s) on? If there are two engines they are treated as one.
	double		fuel_consumption_since_last_time = 0;
	double		internal_fuel;
	double		external_fuel;
	
	// Drag chute state
	double		dragchute_command = 0.0;			// dragChute command
	double		dragchute = 0.0;					// is the dragChute out?

	EDPARAM cockpitAPI;
	param_stuff param_class;
}

static F117::DamageState g_damage;
static std::queue<ed_fm_simulation_event> g_simEvents;


namespace
{
    std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath)
    {
        const char* userProfile = getenv("USERPROFILE");
        if (userProfile == nullptr)
        {
            return std::string();
        }

        return std::string(userProfile) + "\\Saved Games\\" + dcsFolderName + "\\" + relativePath;
    }

    constexpr double kGearCommandDown = 1.0;
    constexpr double kGearCommandUp = 0.0;
    constexpr double kGearToggleThreshold = 0.5;
    constexpr double kWeightOnWheelsDrawArgThreshold = 0.5;

    constexpr double kRollTrimStep = 0.02;
    constexpr double kPitchTrimStep = 0.05;
    constexpr double kYawTrimStep = 0.05;
    constexpr double kThrottleStep = 0.5;
    constexpr double kWheelBrakeOnFriction = 0.50;
    constexpr double kWheelBrakeOffFriction = 0.015;
    constexpr double kToggleLowThreshold = 0.25;
    constexpr double kToggleHighThreshold = 0.75;

    constexpr double kLuaDamageDose = 0.25;
    constexpr double kWingDamageDoseScale = 0.6;
    constexpr double kEngineDamageDoseScale = 0.4;
    constexpr double kTailDamageDoseScale = 0.3;
    constexpr double kCockpitDamageDoseScale = 0.2;
    constexpr double kFireDamageThreshold = 0.8;
    constexpr double kDamageScale = 5.0;

    constexpr size_t kDamageDrawArgRequiredSize = 245;
    constexpr size_t kMirroredGearDrawArgRequiredSize = 616;
    constexpr size_t kTailDamageDrawArgRequiredSize = 248;
    constexpr size_t kWingDamageDrawArgRequiredSize = 1015;
    constexpr int kInitialDrawArgLogFrames = 500;
    constexpr int kPeriodicDrawArgLogFrames = 1000;

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
    constexpr int kWeaponBayArg = 26;
    constexpr int kDragChuteArg = 35;
    constexpr int kMirroredGearArgNose = 611;
    constexpr int kMirroredGearArgLeft = 614;
    constexpr int kMirroredGearArgRight = 616;

    constexpr int kDamageArgFuselageBottom = 134;
    constexpr int kDamageArgFuselageTop = 135;
    constexpr int kDamageArgRightWingOuter = 213;
    constexpr int kDamageArgLeftWingOuter = 223;
    constexpr int kDamageArgLeftFlapOuter = 240;
    constexpr int kDamageArgRightFlapOuter = 241;
    constexpr int kDamageArgLeftFlapCenter = 244;
    constexpr int kDamageArgRightFlapCenter = 245;

    constexpr int kBrokenLeftWingStructureArg = 224;
    constexpr int kBrokenLeftRudderArg = 247;
    constexpr int kBrokenRightRudderArg = 248;
    constexpr int kBrokenLeftElevonOuterArg = 1010;
    constexpr int kBrokenLeftElevonCenterArg = 1011;
    constexpr int kBrokenRightElevonCenterArg = 1014;
    constexpr int kBrokenRightElevonOuterArg = 1015;

    constexpr int kFc3CockpitPitchArg = 1000;
    constexpr int kFc3CockpitRollArg = 1001;
    constexpr int kFc3CockpitThrottleArg = 1002;
    constexpr int kFc3CockpitRudderArg = 1003;

    constexpr double kGroundStartIdleN2 = 60.0;
    constexpr double kAirStartThrottle = 77.5;
    constexpr double kAirStartN2 = 92.0;
    constexpr double kMaxThrustNewtons = 92000.0;
    constexpr double kIdleEgtCelsius = 450.0;
    constexpr double kMaxEgtCelsius = 850.0;
    constexpr double kIdleFuelFlowPph = 1500.0;
    constexpr double kMaxFuelFlowPph = 6000.0;
    constexpr double kAirframeOxygenSupply = 1000.0;
    constexpr double kDefaultFlowVelocity = 100.0;
    constexpr float kVisualDamageThreshold = 0.01f;

    constexpr int kDamageElementCockpit = 3;
    constexpr int kDamageElementEngine = 10;
    constexpr int kDamageElementMain = 11;

    bool gear_retraction_blocked()
    {
        return F117::weight_on_wheels || F117::wow_from_draw_args;
    }

    double clamp_gear_command_for_weight_on_wheels(double requestedCommand)
    {
        if (gear_retraction_blocked() && requestedCommand < kGearCommandDown)
        {
            return kGearCommandDown;
        }

        return requestedCommand;
    }

    void set_gear_command(double requestedCommand)
    {
        F117::GearCommand = clamp_gear_command_for_weight_on_wheels(requestedCommand);
    }

    void update_weight_on_wheels(double normalForceY)
    {
        const bool physicsWeightOnWheels =
            (F117::ACTUATORS::gear_state >= 0.99) &&
            (F117::weight_N > normalForceY) &&
            (fabs(F117::ay_world) <= 0.5);

        F117::weight_on_wheels = F117::wow_from_draw_args || physicsWeightOnWheels;
        if (F117::weight_on_wheels)
        {
            F117::GearCommand = kGearCommandDown;
        }
    }
}

// World-axis state captured from DCS and reused during the simulation step.
double ax_world = 0;
double az_world = 0;
double vx_world = 0;
double vy_world = 0;
double vz_world = 0;

// Accumulate a body-axis force for the current frame.
// The position parameter is ignored here; forces are applied at the current center of mass.
void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
{
	common_force.x += Force.x;
	common_force.y += Force.y;
	common_force.z += Force.z;
}

// Accumulate a body-axis moment for the current frame.
void add_local_moment(const Vec3 & Moment)
{
	common_moment.x += Moment.x;
	common_moment.y += Moment.y;
	common_moment.z += Moment.z;
}


// Return the accumulated body-axis force to DCS after the current simulation step.
void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	x = common_force.x;
	y = common_force.y;
	z = common_force.z;
	pos_x = center_of_gravity.x;
	pos_y = center_of_gravity.y;
	pos_z = center_of_gravity.z;
}

// Not used
void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{}

// Not used
void ed_fm_add_global_moment(double & x,double &y,double &z)
{
}

// Return the accumulated body-axis moment to DCS after the current simulation step.
void ed_fm_add_local_moment(double & x,double &y,double &z)
{
	x = common_moment.x;
	y = common_moment.y;
	z = common_moment.z; 
}

float aoa_filter = 0.0;
float aos_filter = 0.0;
float roll_filter = 0.0;


// Forward declarations for damage debug logging
static FILE* g_damageLog = nullptr;
static void initDamageLog();
static double g_damageLogTimer = 0.0;

// Forward declaration for fire event helper
static void pushFireEvent(int handle, double x, double y, double z);

namespace
{
    bool handle_roll_command(int command, float value)
    {
        switch (command)
        {
        case JoystickRoll:
            F117::FLIGHTCONTROLS::latStickInput = limit(value, -1.0, 1.0);
            F117::roll_cmd = limit(value, -1.0, 1.0);
            return true;
        case RollLeft:
            F117::FLIGHTCONTROLS::latStickInput = (-value - 0.025) / 2.0 * 100.0;
            F117::roll_cmd = (-value - 0.025) / 2.0 * 100.0;
            return true;
        case RollLeftStop:
            F117::FLIGHTCONTROLS::latStickInput = 0.0;
            return true;
        case trimLeft:
            F117::rollTrim += kRollTrimStep;
            return true;
        case RollRight:
            F117::FLIGHTCONTROLS::latStickInput = (-value + 0.025) / 2.0 * 100.0;
            F117::roll_cmd = (-value + 0.025) / 2.0 * 100.0;
            return true;
        case RollRightStop:
            F117::FLIGHTCONTROLS::latStickInput = 0.0;
            return true;
        case trimRight:
            F117::rollTrim -= kRollTrimStep;
            return true;
        }

        return false;
    }

    bool handle_pitch_command(int command, float value)
    {
        switch (command)
        {
        case JoystickPitch:
            F117::FLIGHTCONTROLS::longStickInput = limit(-value, -1.0, 1.0);
            return true;
        case PitchUp:
            F117::FLIGHTCONTROLS::longStickInput = -1.0;
            return true;
        case PitchUpStop:
            F117::FLIGHTCONTROLS::longStickInput = 0.0;
            return true;
        case trimUp:
            F117::pitchTrim -= kPitchTrimStep;
            return true;
        case PitchDown:
            F117::FLIGHTCONTROLS::longStickInput = 1.0;
            return true;
        case PitchDownStop:
            F117::FLIGHTCONTROLS::longStickInput = 0.0;
            return true;
        case trimDown:
            F117::pitchTrim += kPitchTrimStep;
            return true;
        }

        return false;
    }

    bool handle_yaw_command(int command, float value)
    {
        switch (command)
        {
        case JoystickYaw:
            F117::pedInput = limit(-value * (501.0 / (beta_DEG * beta_DEG + 500.0)), -1.0, 1.0);
            return true;
        case rudderleft:
            F117::pedInput = -value + (101.0 / (beta_DEG * beta_DEG + 100.0));
            return true;
        case rudderleftend:
            F117::pedInput = 0.0;
            return true;
        case ruddertrimLeft:
            F117::yawTrim += kYawTrimStep;
            return true;
        case rudderright:
            F117::pedInput = -value - (101.0 / (beta_DEG * beta_DEG + 100.0));
            return true;
        case rudderrightend:
            F117::pedInput = 0.0;
            return true;
        case ruddertrimRight:
            F117::yawTrim -= kYawTrimStep;
            return true;
        }

        return false;
    }

    void apply_lua_damage_hit()
    {
        if (F117::param_class.invincible_value == 0)
        {
            return;
        }

        g_damage.leftWing = max(0.0, g_damage.leftWing - kLuaDamageDose * kWingDamageDoseScale);
        g_damage.rightWing = max(0.0, g_damage.rightWing - kLuaDamageDose * kWingDamageDoseScale);
        g_damage.leftEngine = max(0.0, g_damage.leftEngine - kLuaDamageDose * kEngineDamageDoseScale);
        g_damage.rightEngine = max(0.0, g_damage.rightEngine - kLuaDamageDose * kEngineDamageDoseScale);
        g_damage.leftTail = max(0.0, g_damage.leftTail - kLuaDamageDose * kTailDamageDoseScale);
        g_damage.rightTail = max(0.0, g_damage.rightTail - kLuaDamageDose * kTailDamageDoseScale);
        g_damage.cockpit = max(0.0, g_damage.cockpit - kLuaDamageDose * kCockpitDamageDoseScale);

        if (g_damage.leftEngine < kFireDamageThreshold)
        {
            pushFireEvent(8, -4.45, 0.08, -1.7);
        }
        if (g_damage.rightEngine < kFireDamageThreshold)
        {
            pushFireEvent(7, -4.45, 0.08, 1.7);
        }

        initDamageLog();
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "[LUA_DMG] Hit received! dose=%.2f | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                kLuaDamageDose,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit);
            fflush(g_damageLog);
        }
    }

    bool handle_engine_and_throttle_command(int command, float value)
    {
        switch (command)
        {
        case EnginesOff:
            F117::engineswitch = 0;
            F117::throttleInput = 0.0;
            return true;
        case EnginesOn:
            F117::engineswitch = 1;
            return true;
        case JoystickThrottle:
            if (F117::engineswitch == true)
            {
                F117::throttleInput = limit(((-value + 1.0) * 0.5) * 100.0, 0.0, 100.0);
            }
            return true;
        case ThrottleIncrease:
            if (F117::engineswitch == true && F117::internal_fuel >= 5.0)
            {
                F117::throttleInput += kThrottleStep;
                F117::throttleInput = limit(F117::throttleInput, 0.0, 100.0);
            }
            return true;
        case ThrottleDecrease:
            if (F117::engineswitch == true)
            {
                F117::throttleInput -= kThrottleStep;
                F117::throttleInput = limit(F117::throttleInput, 0.0, 100.0);
            }
            return true;
        }

        return false;
    }

    bool handle_actuator_toggle_command(int command)
    {
        switch (command)
        {
        case tailhook:
            if (F117::ACTUATORS::tailhook_state < kToggleLowThreshold)
            {
                F117::tailhook_command = 1.0;
            }
            else if (F117::ACTUATORS::tailhook_state > kToggleHighThreshold)
            {
                F117::tailhook_command = 0.0;
            }
            return true;
        case dragChute:
            if (F117::ACTUATORS::dragchute_state < kToggleLowThreshold)
            {
                F117::dragchute_command = 1.0;
            }
            else if (F117::ACTUATORS::dragchute_state > kToggleHighThreshold)
            {
                F117::dragchute_command = 0.0;
            }
            printf("Drag chute = %f \n", dragchute_command);
            return true;
        case geardown:
            set_gear_command(kGearCommandDown);
            return true;
        case gearup:
            set_gear_command(kGearCommandUp);
            return true;
        case geartoggle:
            if (F117::ACTUATORS::gear_state > kGearToggleThreshold)
            {
                set_gear_command(kGearCommandUp);
            }
            else if (F117::ACTUATORS::gear_state < kGearToggleThreshold)
            {
                set_gear_command(kGearCommandDown);
            }
            return true;
        case WheelBrakeOn:
            F117::rolling_friction = kWheelBrakeOnFriction;
            return true;
        case WheelBrakeOff:
            F117::rolling_friction = kWheelBrakeOffFriction;
            return true;
        case bombay:
            if (misc_state < 0.5)
            {
                F117::misc_cmd = 1.0;
            }
            if (misc_state > 0.5)
            {
                F117::misc_cmd = 0.0;
            }
            return true;
        case luaDamageHit:
            apply_lua_damage_hit();
            return true;
        }

        return false;
    }

    bool handle_autopilot_command(int command)
    {
        switch (command)
        {
        case autopilot_alt:
            horiz_hold = 0;
            altroll_hold = 0;
            alt_hold = (alt_hold < 0.5) ? 1 : 0;
            return true;
        case autopilot_horiz:
            alt_hold = 0;
            altroll_hold = 0;
            horiz_hold = (horiz_hold < 0.5) ? 1 : 0;
            return true;
        case autopilot_alt_roll:
            alt_hold = 0;
            horiz_hold = 0;
            altroll_hold = (altroll_hold < 0.5) ? 1 : 0;
            return true;
        case autopilot_reset:
            horiz_hold = 0;
            alt_hold = 0;
            altroll_hold = 0;
            return true;
        }

        return false;
    }

void log_simulation_frame(double dt)
{
    static int simFrameCount = 0;
    simFrameCount++;
    initDamageLog();
    if (g_damageLog)
    {
        g_damageLogTimer += dt;
        if (simFrameCount <= 500 || g_damageLogTimer >= 2.0)
        {
            g_damageLogTimer = 0.0;
            double accelMagLog = sqrt(ax_world * ax_world + F117::ay_world * F117::ay_world + az_world * az_world);
            fprintf(g_damageLog,
                "[SIM] frame=%d invincible=%d | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f | accel=%.1f events=%d WoW=%d\n",
                simFrameCount,
                (int)F117::param_class.invincible_value,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit,
                accelMagLog,
                (int)g_simEvents.size(),
                (int)F117::weight_on_wheels);
            fflush(g_damageLog);
        }
    }
}

void begin_simulation_frame()
{
    common_force = Vec3();
    common_moment = Vec3();
}

void update_derived_damage_values()
{
    g_damage.wingAsymmetry = g_damage.leftWing - g_damage.rightWing;
    g_damage.totalWingLoss = 1.0 - (g_damage.leftWing + g_damage.rightWing) / 2.0;
    g_damage.totalTailLoss = 1.0 - (g_damage.leftTail + g_damage.rightTail) / 2.0;
    g_damage.engineAsymmetry = g_damage.leftEngine - g_damage.rightEngine;
}

void update_total_velocity_from_airmass()
{
    Vec3 airspeed;
    airspeed.x = velocity_world_cs.x - wind.x;
    airspeed.y = velocity_world_cs.y - wind.y;
    airspeed.z = velocity_world_cs.z - wind.z;

    F117::totalVelocity_FPS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * F117::meterToFoot;
    if (F117::totalVelocity_FPS < 0.01)
    {
        F117::totalVelocity_FPS = 0.01;
    }
}

void update_atmosphere_from_total_velocity(double* temp)
{
    F117::ATMOS::atmos(F117::ambientTemperature_DegK, F117::ambientDensity_KgPerM3, F117::totalVelocity_FPS, temp);
    F117::dynamicPressure_LBFT2 = temp[0];
    F117::mach = temp[1];
}

void update_autopilot_hold_state()
{
    if (g_damage.cockpit > 0.7)
    {
        if (alt_hold == 1)
        {
            pitchTrim = limit(vspeed, -10, 10);
        }

        if (altroll_hold == 1)
        {
            pitchTrim = limit(vspeed, -10, 10);
            rollTrim = roll_angle / 10;
        }

        if (horiz_hold == 1)
        {
            pitchTrim = pitch_angle * 2;
            rollTrim = roll_angle / 10;
        }
    }
    else
    {
        alt_hold = 0;
        altroll_hold = 0;
        horiz_hold = 0;
    }
}

void update_fuel_system(double dt)
{
    F117::fuel_consumption_since_last_time = (F117::thrust_N / 36000) * dt;
    F117::internal_fuel -= F117::fuel_consumption_since_last_time * F117::param_class.fuelvalue;
}


double get_control_degradation()
{
    double controlDegradation = 1.0;
    if (g_damage.cockpit < 0.7)
    {
        controlDegradation = g_damage.cockpit / 0.7;
    }

    return controlDegradation;
}

void update_flight_control_commands(double dt, double controlDegradation)
{
    aoa_filter = 1;

    F117::elevator_DEG_commanded = -(F117::FLIGHTCONTROLS::fcs_pitch_controller(
        F117::FLIGHTCONTROLS::longStickInput * controlDegradation,
        0.0,
        F117::alpha_DEG,
        F117::pitchRate_RPS * F117::radiansToDegrees,
        (F117::az / 9.81),
        0.0,
        F117::dynamicPressure_LBFT2,
        dt,
        F117::roll_angle,
        F117::pitch_angle,
        F117::totalVelocity_FPS,
        F117::mach,
        F117::thrust_N,
        F117::AERO::Cx_total));
    F117::elevator_DEG = F117::elevator_DEG_commanded + F117::pitchTrim;
    if (alpha_DEG < 1)
    {
        F117::elevator_DEG = limit(F117::elevator_DEG, -25, 25.0);
    }
    if (alpha_DEG >= 1)
    {
        F117::elevator_DEG = limit(F117::elevator_DEG, -(25.0 / aoa_filter), 25.0);
    }
    if (alpha_DEG <= -5)
    {
        F117::elevator_DEG = limit(F117::elevator_DEG, -25.0, (15.0 / aoa_filter));
    }

    roll_filter = static_cast<float>(((rollRate_RPS * rollRate_RPS) / 2) / 4 * 5 + 0.5);

    F117::aileron_DEG_commanded = F117::FLIGHTCONTROLS::fcs_roll_controller(
        F117::FLIGHTCONTROLS::latStickInput * controlDegradation,
        F117::FLIGHTCONTROLS::longStickForce,
        F117::ay / 9.81,
        F117::rollRate_RPS * F117::radiansToDegrees,
        0.0,
        F117::dynamicPressure_LBFT2,
        dt);
    F117::aileron_DEG = F117::aileron_DEG_commanded + F117::rollTrim;
    F117::aileron_DEG = limit(F117::aileron_DEG, (-15.0 * roll_filter), (15.0 * roll_filter));

    F117::dragchute = F117::ACTUATORS::dragchute_actuator(
        F117::dragchute_command,
        dt,
        F117::totalVelocity_FPS,
        F117::gearDown,
        F117::weight_on_wheels);

    aos_filter = static_cast<float>(pedInput * (beta_DEG * beta_DEG) / 7500 * (1 + (yawRate_RPS * radiansToDegrees) / 90) + 1);

    F117::rudder_DEG_commanded = F117::FLIGHTCONTROLS::fcs_yaw_controller(
        F117::pedInput,
        0.0,
        F117::yawRate_RPS * (180.0 / 3.14159),
        ((F117::rollRate_RPS * F117::radiansToDegrees) / 45),
        F117::FLIGHTCONTROLS::alphaFiltered,
        F117::aileron_DEG_commanded,
        F117::ay / 1.56,
        dt);
    F117::rudder_DEG = F117::rudder_DEG_commanded + F117::yawTrim;
    F117::rudder_DEG = limit(F117::rudder_DEG, -15.0 / aos_filter, 15.0 / aos_filter);
}

double update_airframe_actuators(double dt)
{
    F117::elev_pos = F117::ACTUATORS::elev_actuator(
        static_cast<float>(F117::FLIGHTCONTROLS::longStickInput / aoa_filter + (F117::pitchTrim / 15)),
        dt);

    const double tailIntegrity = (g_damage.leftTail + g_damage.rightTail) / 2.0;
    F117::rudder_pos = static_cast<float>(F117::ACTUATORS::rudder_actuator(static_cast<float>(F117::pedInput), dt) * tailIntegrity);

    F117::GearCommand = clamp_gear_command_for_weight_on_wheels(F117::GearCommand);
    F117::gearDown = F117::ACTUATORS::gear_actuator(F117::GearCommand, dt, gear_retraction_blocked());

    F117::tailhook_pos = F117::ACTUATORS::tailhook_actuator(F117::tailhook_command, dt);
    F117::misc_state = F117::ACTUATORS::misc_actuator(F117::misc_cmd, dt);
    F117::misc_stateH = F117::ACTUATORS::misc_actuatorH(F117::misc_cmdH, dt);

    return tailIntegrity;
}

void update_propulsion_and_control_effectiveness(double dt, double tailIntegrity)
{
    const double avgEngineIntegrity = (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
    F117::throttle_state = F117::ACTUATORS::throttle_actuator(F117::throttleInput * avgEngineIntegrity, dt);

    const bool engineRunning = F117::engineswitch && (F117::internal_fuel >= 5.0) && (avgEngineIntegrity > 0.05);
    const double damagedThrottle = F117::throttleInput * avgEngineIntegrity;
    F117::thrust_N = F117::ENGINE::engine_dynamics(damagedThrottle, F117::mach, F117::altitude_FT, dt, engineRunning, F117::weight_on_wheels);

    const double wingIntegrity = 1.0 - g_damage.totalWingLoss;
    F117::aileron_PCT = (F117::aileron_DEG * wingIntegrity) / 25.5;
    F117::elevator_PCT = (F117::elevator_DEG * (1.0 - g_damage.totalTailLoss)) / 25.0;

    const double tailAsymmetry = g_damage.leftTail - g_damage.rightTail;
    F117::rudder_PCT = (F117::rudder_DEG * tailIntegrity) / 30.0 + tailAsymmetry * 0.1;
}


void update_aerodynamic_coefficients(double* temp)
{
    const double alpha1_DEG_Limited = limit(F117::alpha_DEG, -20.0, 90.0);
    const double beta1_DEG_Limited = limit(F117::beta_DEG, -30.0, 30.0);

    const double CDchute = 0.5 * F117::ACTUATORS::dragchute_state;
    const double Cxchute = CDchute * cos(F117::pi / 180.0);

    const double CDbay = 0.015 * F117::ACTUATORS::misc_pos;
    const double Czbay = -(CDbay * sin(F117::pi / 180.0));
    const double Cxbay = CDbay * cos(F117::pi / 180.0);

    const double CDGear = 0.027 * F117::gearDown * 1.5;
    const double CzGear = -(CDGear * sin(F117::pi / 180.0));
    const double CxGear = CDGear * cos(F117::pi / 180.0);

    F117::AERO::hifi_C(alpha1_DEG_Limited, beta1_DEG_Limited, F117::elevator_DEG, temp);

    const double Cz = temp[1];
    F117::AERO::Cz = Cz;

    const double Cd0 = 0.0155;
    const double K_induced = 0.18;
    const double Cd_induced = K_induced * Cz * Cz;

    double Cd_wave = 0.0;
    const double Mcrit = 0.87;
    const double Mdiv = 0.90;

    if (F117::mach > Mcrit)
    {
        if (F117::mach < 1.05)
        {
            const double mach_factor = (F117::mach - Mcrit) / (1.05 - Mcrit);
            Cd_wave = 0.05 * mach_factor * mach_factor;

            if (F117::mach > Mdiv)
            {
                const double barrier_factor = (F117::mach - Mdiv) / (1.05 - Mdiv);
                Cd_wave += 0.07 * barrier_factor * barrier_factor;
            }
        }
        else
        {
            Cd_wave = 0.12;
        }
    }

    F117::AERO::Cx = Cd0 + Cd_induced + Cd_wave;
    F117::AERO::Cm = temp[2];
    F117::AERO::Cy = temp[3];
    F117::AERO::Cn = temp[4];
    F117::AERO::Cl = temp[5];

    F117::AERO::hifi_damping(alpha1_DEG_Limited, temp);
    F117::AERO::Cxq = temp[0];
    F117::AERO::Cyr = temp[1];
    F117::AERO::Cyp = temp[2];
    F117::AERO::Czq = temp[3];
    F117::AERO::Clr = temp[4];
    F117::AERO::Clp = temp[5];
    F117::AERO::Cmq = temp[6];
    F117::AERO::Cnr = temp[7];
    F117::AERO::Cnp = temp[8];

    F117::AERO::hifi_rudder(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
    F117::AERO::Cy_delta_r30 = temp[0];
    F117::AERO::Cn_delta_r30 = temp[1];
    F117::AERO::Cl_delta_r30 = temp[2] * 1.2;

    F117::AERO::hifi_ailerons(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
    F117::AERO::Cy_delta_a20 = temp[0];
    F117::AERO::Cn_delta_a20 = temp[2];
    F117::AERO::Cl_delta_a20 = temp[4];

    F117::AERO::hifi_other_coeffs(alpha1_DEG_Limited, F117::elevator_DEG, temp);
    F117::AERO::Cn_delta_beta = temp[0];
    F117::AERO::Cl_delta_beta = temp[1];
    F117::AERO::Cm_delta = temp[2];
    F117::AERO::eta_el = temp[3];
    F117::AERO::Cm_delta_ds = 0;

    F117::AERO::dXdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cxq;
    F117::AERO::Cx_total = F117::AERO::Cx + F117::AERO::dXdQ * F117::pitchRate_RPS;
    F117::AERO::Cx_total += CxGear + Cxchute + Cxbay;

    F117::AERO::dZdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Czq;
    F117::AERO::Cz_total = F117::AERO::Cz + F117::AERO::dZdQ * F117::pitchRate_RPS;
    F117::AERO::Cz_total += CzGear + Czbay;

    F117::AERO::dMdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cmq;
    F117::AERO::Cm_total = F117::AERO::Cm * F117::AERO::eta_el + F117::AERO::Cz_total * (F117::referenceCG_PCT - F117::actualCG_PCT) + F117::AERO::dMdQ * F117::pitchRate_RPS + F117::AERO::Cm_delta + F117::AERO::Cm_delta_ds + F117::Cm0;

    F117::AERO::dYdail = F117::AERO::Cy_delta_a20;
    F117::AERO::dYdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cyr;
    F117::AERO::dYdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cyp;
    F117::AERO::Cy_total = F117::AERO::Cy + F117::AERO::dYdail * F117::aileron_PCT + F117::AERO::Cy_delta_r30 * F117::rudder_PCT + F117::AERO::dYdR * F117::yawRate_RPS + F117::AERO::dYdP * F117::rollRate_RPS;

    F117::AERO::dNdail = F117::AERO::Cn_delta_a20;
    F117::AERO::dNdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cnr;
    F117::AERO::dNdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cnp;
    F117::AERO::Cn_total = F117::AERO::Cn - F117::AERO::Cy_total * (F117::referenceCG_PCT - F117::actualCG_PCT) * (F117::meanChord_FT / F117::wingSpan_FT) + F117::AERO::dNdail * F117::aileron_PCT + F117::AERO::Cn_delta_r30 * F117::rudder_PCT + F117::AERO::dNdR * F117::yawRate_RPS + F117::AERO::dNdP * F117::rollRate_RPS + F117::AERO::Cn_delta_beta * F117::beta_DEG;

    F117::AERO::dLdail = F117::AERO::Cl_delta_a20;
    F117::AERO::dLdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Clr;
    F117::AERO::dLdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Clp;
    F117::AERO::Cl_total = F117::AERO::Cl + F117::AERO::dLdail * F117::aileron_PCT + F117::AERO::Cl_delta_r30 * F117::rudder_PCT + F117::AERO::dLdR * F117::yawRate_RPS + F117::AERO::dLdP * F117::rollRate_RPS + F117::AERO::Cl_delta_beta * F117::beta_DEG;
}

double apply_aerodynamic_forces_and_thrust()
{
    Vec3 cy_force(0.0, 0.0, F117::AERO::Cy_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825);
    Vec3 cy_force_pos(0.0, 0, 0);
    add_local_force(cy_force, cy_force_pos);

    Vec3 cx_force(-F117::AERO::Cx_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825, 0, 0);
    Vec3 cx_force_pos(0, 0.0, 0.0);
    add_local_force(cx_force, cx_force_pos);

    const double liftDamageFactor = 1.0 - g_damage.totalWingLoss;
    Vec3 cz_force(0.0, -F117::AERO::Cz_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825 * liftDamageFactor, 0.0);
    Vec3 cz_force_pos(0, 0, 0);
    add_local_force(cz_force, cz_force_pos);

    const double qS_b = F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795;
    double rollMoment = F117::AERO::Cl_total * qS_b;
    rollMoment += g_damage.wingAsymmetry * qS_b * 0.05;
    Vec3 cl_moment(rollMoment, 0.0, 0.0);
    add_local_moment(cl_moment);

    const double pitchDamageFactor = 1.0 - g_damage.totalTailLoss * 0.5;
    const double noseDownBias = -g_damage.totalWingLoss * qS_b * 0.08;
    Vec3 cm_moment(0.0, 0.0, F117::AERO::Cm_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 1.35581795 * F117::meanChord_FT * pitchDamageFactor + noseDownBias);
    add_local_moment(cm_moment);

    const double qS_span = F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795;
    double yawMoment = -F117::AERO::Cn_total * qS_span;
    const double tailAsymDamage = g_damage.leftTail - g_damage.rightTail;
    yawMoment += tailAsymDamage * qS_span * 0.03;
    Vec3 cn_moment(0.0, yawMoment, 0.0);
    add_local_moment(cn_moment);

    const double thrust_cant_up_deg = 0.1;
    const double thrust_toe_out_deg = 10;
    const double cant_up_rad = thrust_cant_up_deg * F117::pi / 180.0;
    const double toe_out_rad = thrust_toe_out_deg * F117::pi / 180.0;

    const double thrust_per_engine = F117::thrust_N * 0.5;
    const double leftThrust = thrust_per_engine * g_damage.leftEngine;
    const double rightThrust = thrust_per_engine * g_damage.rightEngine;

    const double thrust_x_L = leftThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_L = leftThrust * sin(cant_up_rad);
    const double thrust_z_L = leftThrust * cos(cant_up_rad) * sin(toe_out_rad);

    const double thrust_x_R = rightThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_R = rightThrust * sin(cant_up_rad);
    const double thrust_z_R = rightThrust * cos(cant_up_rad) * sin(toe_out_rad);

    Vec3 thrust_force_L(thrust_x_L, thrust_y_L, thrust_z_L);
    Vec3 thrust_force_pos_L(-4.604, 0.000, -1.427);
    add_local_force(thrust_force_L, thrust_force_pos_L);

    Vec3 thrust_force_R(thrust_x_R, thrust_y_R, -thrust_z_R);
    Vec3 thrust_force_pos_R(-4.604, 0.000, 1.427);
    add_local_force(thrust_force_R, thrust_force_pos_R);

    return cz_force.y;
}

}

//-----------------------------------------------------------------------
// Main FM update called once per DCS simulation step.
// `dt` is the current frame step in seconds.
//-----------------------------------------------------------------------
void ed_fm_simulate(double dt)
{
	F117::DeltaTime = dt;


log_simulation_frame(dt);

// Clear the per-frame force and moment accumulators.
begin_simulation_frame();
update_derived_damage_values();

// Compute air-relative speed using the current wind estimate.
// The downstream aero model still expects feet/second here.
update_total_velocity_from_airmass();

// Update Mach and dynamic pressure from the atmosphere model.
// The tuned aero model still uses its original imperial-unit convention here.
double* temp = F117::temp;
update_atmosphere_from_total_velocity(temp);

//---------------------------------------------
//-----CONTROL DYNAMICS------------------------
//---------------------------------------------
//
update_autopilot_hold_state();

// Fuel system
update_fuel_system(dt);
// Legacy experimental fuel-scaling hook left here for reference.

// Cockpit damage reduces effective control input below 0.7 integrity.
double controlDegradation = get_control_degradation();

// Run the tuned pitch/roll/yaw control laws and clamp the commanded surfaces.
update_flight_control_commands(dt, controlDegradation);

double tailIntegrity = update_airframe_actuators(dt);
update_propulsion_and_control_effectiveness(dt, tailIntegrity);
	// Engine and yaw debug logging
		{
			static std::ofstream engineDebugLog;
			static int engineLogCounter = 0;
			static bool engineLogInitialized = false;

			if (!engineLogInitialized)
			{
				const std::string savedGamesLogPath =
					build_saved_games_dcs_path("DCS", "Logs\\F117_Engine_Debug.csv");
				if (!savedGamesLogPath.empty())
				{
					engineDebugLog.open(savedGamesLogPath);
				}
				if (!engineDebugLog.is_open())
				{
					const std::string openBetaLogPath =
						build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Engine_Debug.csv");
					if (!openBetaLogPath.empty())
					{
						engineDebugLog.open(openBetaLogPath);
					}
				}
				if (!engineDebugLog.is_open())
				{
					engineDebugLog.open("F117_Engine_Debug.csv");
				}
				if (engineDebugLog.is_open())
				{
					engineDebugLog << "engine1_thrust_N,engine2_thrust_N,total_thrust_N,throttle_pct,N2_rpm_pct,mach,velocity_fps,altitude_ft,"
					               << "pedInput,rudder_DEG_cmd,rudder_DEG,rudder_PCT,rudder_pos,beta_DEG,yawRate_RPS,yawTrim,aileron_DEG,aileron_PCT\n";
				}
				engineLogInitialized = true;
			}

			engineLogCounter++;
			if (engineDebugLog.is_open() && (engineLogCounter % 100 == 0))
			{
				double perEngineThrust = F117::thrust_N * 0.5;
				engineDebugLog << perEngineThrust << ","
				               << perEngineThrust << ","
				               << F117::thrust_N << ","
				               << F117::throttleInput << ","
				               << F117::ENGINE::N2 << ","
				               << F117::mach << ","
				               << F117::totalVelocity_FPS << ","
				               << F117::altitude_FT << ","
				               << F117::pedInput << ","
				               << F117::rudder_DEG_commanded << ","
				               << F117::rudder_DEG << ","
				               << F117::rudder_PCT << ","
				               << F117::rudder_pos << ","
				               << F117::beta_DEG << ","
				               << F117::yawRate_RPS << ","
				               << F117::yawTrim << ","
				               << F117::aileron_DEG << ","
				               << F117::aileron_PCT << "\n";
				engineDebugLog.flush();
			}
		}


// Aerodynamic coefficient update and force accumulation
update_aerodynamic_coefficients(temp);
double weightOnWheelsReferenceForce = apply_aerodynamic_forces_and_thrust();
		// Mark subsystem initialization complete after the first successful simulation frame.
		F117::simInitialized = true;
		F117::ACTUATORS::simInitialized = true;
		F117::FLIGHTCONTROLS::simInitialized = true;

		update_weight_on_wheels(weightOnWheelsReferenceForce);

}

void ed_fm_set_atmosphere(	
	double h,//altitude above sea level			(meters)
	double t,//current atmosphere temperature   (Kelvin)
	double a,//speed of sound					(meters/sec)
	double ro,// atmosphere density				(kg/m^3)
	double p,// atmosphere pressure				(N/m^2)
	double wind_vx,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
	double wind_vy,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
	double wind_vz //components of velocity vector, including turbulence in world coordinate system (meters/sec)
	)
{
	F117::ambientTemperature_DegK = t;
	F117::ambientDensity_KgPerM3 = ro;
	F117::altitude_m = h;
	F117::altitude_FT = h * F117::meterToFoot;
	F117::ps_LBFT2 = p * 0.020885434273;
}

void ed_fm_set_current_mass_state ( double mass,
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	center_of_gravity.x  = center_of_mass_x;
	center_of_gravity.y  = center_of_mass_y;
	center_of_gravity.z  = center_of_mass_z;

	inertia.x = moment_of_inertia_x; // Stored so ed_fm_change_mass can compare against the current DCS inertia state.
	inertia.y = moment_of_inertia_y;
	inertia.z = moment_of_inertia_z;

	F117::weight_N = mass * 9.98665002864;
}
/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{
	ax_world = ax;
	F117::ay_world = ay;
	az_world = az;
	vx_world = vx;
	vy_world = vy;
	vz_world = vz;

	F117::vspeed = vy -3.25; // Bias retained to match the current autopilot tuning.
}

double ax_body = 0;
double ay_body = 0;
double az_body = 0;
double vx_body = 0;
double vy_body = 0;
double vz_body = 0;

void ed_fm_set_current_state_body_axis(	
	double ax,//linear acceleration component in body coordinate system (meters/sec^2)
	double ay,//linear acceleration component in body coordinate system (meters/sec^2)
	double az,//linear acceleration component in body coordinate system (meters/sec^2)
	double vx,//linear velocity component in body coordinate system (meters/sec)
	double vy,//linear velocity component in body coordinate system (meters/sec)
	double vz,//linear velocity component in body coordinate system (meters/sec)
	double wind_vx,//wind linear velocity component in body coordinate system (meters/sec)
	double wind_vy,//wind linear velocity component in body coordinate system (meters/sec)
	double wind_vz,//wind linear velocity component in body coordinate system (meters/sec)
	double omegadotx,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegadoty,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegadotz,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegax,//angular velocity components in body coordinate system (rad/sec)
	double omegay,//angular velocity components in body coordinate system (rad/sec)
	double omegaz,//angular velocity components in body coordinate system (rad/sec)
	double yaw,  //radians (rad)
	double pitch,//radians (rad)
	double roll, //radians (rad)
	double common_angle_of_attack, //AoA  (rad)
	double common_angle_of_slide   //AoS  (rad)
	)

{
	ax_body = ax;
	ay_body = ay;
	az_body = az;
	vx_body = vx;
	vy_body = vy;
	vz_body = vz;

	velocity_world_cs.x = vx;
	velocity_world_cs.y = vy;
	velocity_world_cs.z = vz;

	wind.x = wind_vx;
	wind.y = wind_vy;
	wind.z = wind_vz;

	pitch_angle = (pitch * radiansToDegrees); 
	roll_angle = (roll * radiansToDegrees);
	// Update body-axis aircraft state used by the control laws and aero model.
	F117::alpha_DEG	= (common_angle_of_attack * F117::radiansToDegrees);
	F117::beta_DEG	= (common_angle_of_slide * F117::radiansToDegrees);
	F117::rollRate_RPS = omegax;   // These rates feed the tuned damping/limiter schedules.
	F117::yawRate_RPS = -omegay;  // Sign convention matches the existing FM tuning.
	F117::pitchRate_RPS = omegaz;  // These rates act like dampers/limiters in several control paths.
	
	if (alpha_DEG > 20 && pitchRate_RPS > 0) { F117::pitchRate_RPS = omegaz * (((alpha_DEG * alpha_DEG + 10000) / (180.0 + g_damage.totalWingLoss * 20.0)) - 54.5); } // High-AoA pitch-rate shaping retained from the existing tune.

	F117::az = ay;
	F117::ay = az;
}


void ed_fm_set_command(int command, float value)	// Command = Command Index (See Export.lua), Value = Signal Value (-1 to 1 for Joystick Axis)
{
    if (handle_roll_command(command, value))
    {
        return;
    }

    if (handle_pitch_command(command, value))
    {
        return;
    }

    if (handle_yaw_command(command, value))
    {
        return;
    }

    if (handle_engine_and_throttle_command(command, value))
    {
        return;
    }

    if (handle_actuator_toggle_command(command))
    {
        return;
    }

    if (handle_autopilot_command(command))
    {
        return;
    }

    switch (command)
    {
    case resetTrim:
        F117::pitchTrim = 0.0;
        F117::rollTrim = 0.0;
        F117::yawTrim = 0.0;
        break;
    }
}


/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity,  and moments of inertia
	}
*/
bool ed_fm_change_mass  (double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	if((inertia.x != F117::inertia_Ix_KGM2) ||
	   (inertia.y != F117::inertia_Iz_KGM2) ||
	   (inertia.z != F117::inertia_Iy_KGM2))
	{
		delta_mass = 0.0;
		delta_mass_pos_x = 0.0;
		delta_mass_pos_y = 0.0;
		delta_mass_pos_z = 0.0;
		delta_mass_moment_of_inertia_x = F117::inertia_Ix_KGM2 - inertia.x;
		delta_mass_moment_of_inertia_y = F117::inertia_Ix_KGM2 - inertia.z;
		delta_mass_moment_of_inertia_z = F117::inertia_Ix_KGM2 - inertia.y;

		// Can't set to true...crashing right now :(
		return false;
	}
	else
	{
		return false;
	}
	if (F117::fuel_consumption_since_last_time > 0)
	{
		delta_mass		 = F117::fuel_consumption_since_last_time;
		delta_mass_pos_x = -1.0;
		delta_mass_pos_y =  1.0;
		delta_mass_pos_z =  0;

		delta_mass_moment_of_inertia_x	= 0;
		delta_mass_moment_of_inertia_y	= 0;
		delta_mass_moment_of_inertia_z	= 0;

		F117::fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
		// better to use stack like structure for mass changing 
		return true;
	}
	else 
	{
		return false;
	}
}

/*
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
*/
void ed_fm_set_internal_fuel(double fuel)
{
	internal_fuel = fuel;
}
/*
	get internal fuel volume 
*/
double ed_fm_get_internal_fuel()
{
	return internal_fuel + external_fuel;
}
/*
	set external fuel volume for each payload station , called for weapon init and on reload
*/
void  ed_fm_set_external_fuel (int	 station,
								double fuel,
								double x,
								double y,
								double z)
{
}
/*
	get external fuel volume 
*/

double ed_fm_get_external_fuel ()
{
	return external_fuel;
}

double ed_fm_refueling_add_fuel()
{
	return internal_fuel + 100;
}


namespace
{
    void update_damage_from_draw_args(float* drawargs, size_t size)
    {
        if (F117::param_class.invincible_value == 0 || size <= kDamageDrawArgRequiredSize)
        {
            return;
        }

        const float fuseBot = drawargs[kDamageArgFuselageBottom];
        const float fuseTop = drawargs[kDamageArgFuselageTop];
        const float wingLOut = drawargs[kDamageArgLeftWingOuter];
        const float wingROut = drawargs[kDamageArgRightWingOuter];
        const float flapLOut = drawargs[kDamageArgLeftFlapOuter];
        const float flapROut = drawargs[kDamageArgRightFlapOuter];
        const float flapLCtr = drawargs[kDamageArgLeftFlapCenter];
        const float flapRCtr = drawargs[kDamageArgRightFlapCenter];

        static int drawArgLogCounter = 0;
        drawArgLogCounter++;
        if (g_damageLog && (drawArgLogCounter <= kInitialDrawArgLogFrames || drawArgLogCounter % kPeriodicDrawArgLogFrames == 0))
        {
            fprintf(g_damageLog,
                "[DRAWARGS] frame=%d size=%zu fuseB=%.3f fuseT=%.3f wL=%.3f wR=%.3f fLO=%.3f fRO=%.3f fLC=%.3f fRC=%.3f\n",
                drawArgLogCounter, size, fuseBot, fuseTop, wingLOut, wingROut,
                flapLOut, flapROut, flapLCtr, flapRCtr);
            fflush(g_damageLog);
        }

        const bool anyDamage = (fuseBot > kVisualDamageThreshold || fuseTop > kVisualDamageThreshold ||
            wingLOut > kVisualDamageThreshold || wingROut > kVisualDamageThreshold ||
            flapLOut > kVisualDamageThreshold || flapROut > kVisualDamageThreshold ||
            flapLCtr > kVisualDamageThreshold || flapRCtr > kVisualDamageThreshold);

        if (!anyDamage)
        {
            return;
        }

        const float fuseWorst = max(fuseBot, fuseTop);
        if (fuseWorst > kVisualDamageThreshold)
        {
            const double integrity = 1.0 - static_cast<double>(fuseWorst);
            g_damage.leftEngine = min(g_damage.leftEngine, integrity);
            g_damage.rightEngine = min(g_damage.rightEngine, integrity);
            g_damage.cockpit = min(g_damage.cockpit, integrity);
            if (integrity < kFireDamageThreshold)
            {
                pushFireEvent(8, -4.45, 0.08, -1.7);
                pushFireEvent(7, -4.45, 0.08, 1.7);
            }
        }

        const float leftWorst = max(max(wingLOut, flapLOut), flapLCtr);
        if (leftWorst > kVisualDamageThreshold)
        {
            const double integrity = 1.0 - static_cast<double>(leftWorst);
            g_damage.leftWing = min(g_damage.leftWing, integrity);
            if (integrity < kFireDamageThreshold)
            {
                pushFireEvent(4, -0.82, 0.265, -2.774);
            }
        }

        const float rightWorst = max(max(wingROut, flapROut), flapRCtr);
        if (rightWorst > kVisualDamageThreshold)
        {
            const double integrity = 1.0 - static_cast<double>(rightWorst);
            g_damage.rightWing = min(g_damage.rightWing, integrity);
            if (integrity < kFireDamageThreshold)
            {
                pushFireEvent(3, -0.82, 0.265, 2.774);
            }
        }

        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "[DRAWARG_DMG] fuseB=%.3f fuseT=%.3f wL=%.3f wR=%.3f fLO=%.3f fRO=%.3f fLC=%.3f fRC=%.3f\n",
                fuseBot, fuseTop, wingLOut, wingROut, flapLOut, flapROut, flapLCtr, flapRCtr);
            fprintf(g_damageLog,
                "  -> State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit);
            fflush(g_damageLog);
        }
    }

    void sync_gear_draw_args(float* drawargs)
    {
        if (F117::simInitialized)
        {
            F117::ACTUATORS::gear_state = drawargs[kGearArgNose];
            F117::ACTUATORS::gear_state = drawargs[kGearArgLeft];
            F117::ACTUATORS::gear_state = drawargs[kGearArgRight];
        }
        else
        {
            drawargs[kGearArgNose] = static_cast<float>(F117::ACTUATORS::gear_state);
            drawargs[kGearArgLeft] = static_cast<float>(F117::ACTUATORS::gear_state);
            drawargs[kGearArgRight] = static_cast<float>(F117::ACTUATORS::gear_state);
        }
    }

    void update_weight_on_wheels_from_draw_args(float* drawargs)
    {
        F117::wow_from_draw_args = (drawargs[kWowArgNose] + drawargs[kWowArgLeft] + drawargs[kWowArgRight]) > kWeightOnWheelsDrawArgThreshold;
        if (F117::wow_from_draw_args)
        {
            F117::weight_on_wheels = true;
            F117::GearCommand = kGearCommandDown;
        }
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
        drawargs[kDragChuteArg] = static_cast<float>(ACTUATORS::dragchute_state);
        drawargs[kTailhookArg] = static_cast<float>(ACTUATORS::tailhook_state);
        drawargs[kWeaponBayArg] = static_cast<float>(limit(F117::misc_state, 0.0, 1.0));

        if (size > kMirroredGearDrawArgRequiredSize)
        {
            drawargs[kMirroredGearArgNose] = drawargs[kGearArgNose];
            drawargs[kMirroredGearArgLeft] = drawargs[kGearArgLeft];
            drawargs[kMirroredGearArgRight] = drawargs[kGearArgRight];
        }
    }

    void update_damage_animation_draw_args(float* drawargs, size_t size)
    {
        if (size > kTailDamageDrawArgRequiredSize)
        {
            drawargs[kBrokenLeftWingStructureArg] = (g_damage.leftWing < 0.2) ? 1.0f : 0.0f;
            drawargs[kBrokenLeftRudderArg] = (g_damage.leftTail < 0.5) ? 1.0f : 0.0f;
            drawargs[kBrokenRightRudderArg] = (g_damage.rightTail < 0.5) ? 1.0f : 0.0f;
        }
        if (size > kWingDamageDrawArgRequiredSize)
        {
            drawargs[kBrokenLeftElevonOuterArg] = (g_damage.leftWing < 0.7) ? 1.0f : 0.0f;
            drawargs[kBrokenLeftElevonCenterArg] = (g_damage.leftWing < 0.4) ? 1.0f : 0.0f;
            drawargs[kBrokenRightElevonCenterArg] = (g_damage.rightWing < 0.3) ? 1.0f : 0.0f;
            drawargs[kBrokenRightElevonOuterArg] = (g_damage.rightWing < 0.6) ? 1.0f : 0.0f;
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
    update_damage_from_draw_args(drawargs, size);
    sync_gear_draw_args(drawargs);
    update_weight_on_wheels_from_draw_args(drawargs);
    update_control_surface_draw_args(drawargs);
    update_misc_draw_args(drawargs, size);
    update_damage_animation_draw_args(drawargs, size);
}


// FC3 cockpit draw-arg callback used for stick, rudder pedal, throttle, and gear-handle animation.

void ed_fm_set_fc3_cockpit_draw_args_v2(float* drawargs, size_t size)
{
    update_fc3_cockpit_draw_args(drawargs);
}

void ed_fm_configure(const char * cfg_path)
{
	// Reserved DCS FM configuration hook; unused by this module.
}


namespace
{
    bool try_get_suspension_param(unsigned index, double& value)
    {
        switch (index)
        {
        case ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT:
            value = 0.0;
            return true;
        case ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT:
        case ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT:
            value = F117::rolling_friction;
            return true;
        case ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE:
        case ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE:
        case ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE:
            value = 0.0;
            return true;
        case ED_FM_SUSPENSION_0_WHEEL_YAW:
            value = limit(F117::rudder_pos, -0.3, 0.3);
            return true;
        case ED_FM_ANTI_SKID_ENABLE:
            value = true;
            return true;
        case ED_FM_SUSPENSION_0_GEAR_POST_STATE:
        case ED_FM_SUSPENSION_1_GEAR_POST_STATE:
        case ED_FM_SUSPENSION_2_GEAR_POST_STATE:
        case ED_FM_SUSPENSION_0_DOWN_LOCK:
            value = F117::ACTUATORS::gear_state;
            return true;
        case ED_FM_FC3_GEAR_HANDLE_POS:
            value = F117::GearCommand;
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

        const bool engineOn = (F117::engineswitch == true) && (F117::internal_fuel > 5.0);
        double normN2 = engineOn ? F117::ENGINE::N2 / 100.0 : 0.0;
        normN2 = limit(normN2, 0.0, 1.0);

        double normThrust = engineOn ? (F117::thrust_N / kMaxThrustNewtons) : 0.0;
        normThrust = limit(normThrust, 0.0, 1.0);

        switch (index)
        {
        case ED_FM_ENGINE_0_RPM:
        case ED_FM_ENGINE_0_RELATED_RPM:
        case ED_FM_ENGINE_0_THRUST:
        case ED_FM_ENGINE_0_RELATED_THRUST:
            value = 0.0;
            return true;
        case ED_FM_ENGINE_1_RPM:
        case ED_FM_ENGINE_1_RELATED_RPM:
        case ED_FM_ENGINE_1_CORE_RELATED_RPM:
        case ED_FM_ENGINE_2_RPM:
        case ED_FM_ENGINE_2_RELATED_RPM:
        case ED_FM_ENGINE_2_CORE_RELATED_RPM:
            value = normN2;
            return true;
        case ED_FM_ENGINE_1_THRUST:
        case ED_FM_ENGINE_2_THRUST:
            value = engineOn ? F117::thrust_N * 0.5 : 0.0;
            return true;
        case ED_FM_ENGINE_1_RELATED_THRUST:
        case ED_FM_ENGINE_2_RELATED_THRUST:
            value = normThrust;
            return true;
        case ED_FM_ENGINE_1_TEMPERATURE:
        case ED_FM_ENGINE_2_TEMPERATURE:
            value = kIdleEgtCelsius + normN2 * (kMaxEgtCelsius - kIdleEgtCelsius);
            return true;
        case ED_FM_ENGINE_1_OIL_PRESSURE:
        case ED_FM_ENGINE_2_OIL_PRESSURE:
            value = engineOn ? (20.0 + normN2 * 40.0) : 0.0;
            return true;
        case ED_FM_ENGINE_1_FUEL_FLOW:
        case ED_FM_ENGINE_2_FUEL_FLOW:
            value = engineOn ? (kIdleFuelFlowPph + normN2 * (kMaxFuelFlowPph - kIdleFuelFlowPph)) : 0.0;
            return true;
        case ED_FM_FC3_THROTTLE_LEFT:
        case ED_FM_FC3_THROTTLE_RIGHT:
            value = limit(F117::throttleInput / 100.0, 0.0, 1.0);
            return true;
        }

        return false;
    }

    bool try_get_misc_param(unsigned index, double& value)
    {
        switch (index)
        {
        case ED_FM_FUEL_INTERNAL_FUEL:
        case ED_FM_FUEL_TOTAL_FUEL:
            value = F117::internal_fuel + F117::external_fuel;
            return true;
        case ED_FM_OXYGEN_SUPPLY:
            value = kAirframeOxygenSupply;
            return true;
        case ED_FM_FLOW_VELOCITY:
            value = kDefaultFlowVelocity;
            return true;
        case ED_FM_FC3_STICK_PITCH:
            value = F117::FLIGHTCONTROLS::longStickInput;
            return true;
        case ED_FM_FC3_STICK_ROLL:
            value = F117::FLIGHTCONTROLS::latStickInput;
            return true;
        case ED_FM_FC3_RUDDER_PEDALS:
            value = F117::pedInput;
            return true;
        case ED_FM_FC3_AUTOPILOT_STATUS:
            value = F117::alt_hold;
            return true;
        }

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
    void clear_pending_sim_events()
    {
        while (!g_simEvents.empty())
        {
            g_simEvents.pop();
        }
    }

    void reset_damage_and_events()
    {
        g_damage = F117::DamageState();
        clear_pending_sim_events();
    }

    void log_damage_lifecycle_event(const char* label)
    {
        initDamageLog();
        if (g_damageLog)
        {
            fprintf(g_damageLog, "\n*** %s called ***\n", label);
            fflush(g_damageLog);
        }
    }

    void apply_ground_start_state(bool enginesRunning)
    {
        F117::gearDown = kGearCommandDown;
        F117::GearCommand = kGearCommandDown;
        F117::weight_on_wheels = true;
        F117::wow_from_draw_args = true;
        F117::throttleInput = 0.0;
        F117::WheelBrakeCommand = 0.0;
        F117::engineswitch = enginesRunning;
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::ENGINE::N2 = enginesRunning ? kGroundStartIdleN2 : 0.0;
        reset_damage_and_events();
    }

    void apply_air_start_state()
    {
        F117::gearDown = kGearCommandUp;
        F117::GearCommand = kGearCommandUp;
        F117::weight_on_wheels = false;
        F117::wow_from_draw_args = false;
        F117::throttleInput = kAirStartThrottle;
        F117::throttle_state = kAirStartThrottle;
        F117::WheelBrakeCommand = 0.0;
        F117::engineswitch = true;
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::ENGINE::N2 = kAirStartN2;
        reset_damage_and_events();
    }
}

// Reset transient FM state when the aircraft is destroyed, restarted, or unloaded.

void ed_fm_release()
{
    F117::DeltaTime = 0;
    F117::simInitialized = false;
    F117::ACTUATORS::simInitialized = false;
    F117::FLIGHTCONTROLS::simInitialized = false;
    F117::ENGINE::N2 = 0.0;

    reset_damage_and_events();

    F117::pedInput = 0;
    F117::throttleInput = 0.0;
    F117::elevator_DEG = 0;
    F117::aileron_DEG = 0;
    F117::rudder_DEG = 0;
    F117::elevator_DEG_commanded = 0;
    F117::rudder_DEG_commanded = 0;
    F117::throttle_state = 0;
    F117::rolling_friction = kWheelBrakeOffFriction;
    F117::WheelBrakeCommand = 0.0;
    F117::pitchTrim = 0.0;
    F117::rollTrim = 0.0;
    F117::yawTrim = 0.0;
    F117::tailhook_command = 0.0;
    F117::tailhook_pos = 0.0;
    F117::dragchute_command = 0.0;
    F117::dragchute = 0.0;
    F117::misc_cmd = 0.0;
    F117::misc_state = 0.0;
    F117::misc_cmdH = 0.0;
    F117::misc_stateH = 0.0;

    F117::alt_hold = 0;
    F117::horiz_hold = 0;
    F117::alt_hold = 0;

    F117::ACTUATORS::tailhook_state = 0.0;
    F117::ACTUATORS::tailhook_rate = 0.0;
    F117::ACTUATORS::throttle_state = 0.0;
    F117::ACTUATORS::throttle_rate = 0.0;
    F117::ACTUATORS::misc_pos = 0.0;
    F117::FLIGHTCONTROLS::latStickInput = 0.0;
    F117::FLIGHTCONTROLS::longStickInput = 0.0;
    F117::FLIGHTCONTROLS::longStickForce = 0.0;
}


// Conditions to make the screen shake in first-person view.
double ed_fm_get_shake_amplitude()
{
	// Severe wing damage - heavy shaking
	if (g_damage.leftWing < 0.2 || g_damage.rightWing < 0.2)
		return 10;

	// Cockpit damage - moderate shaking
	if (g_damage.cockpit < 0.5)
		return 2.0 * (1.0 - g_damage.cockpit);

	// Engine damage - proportional shaking
	double engineDamage = 1.0 - (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
	if (engineDamage > 0.3)
		return engineDamage * 5.0;

	// G-force shaking
	if (F117::az > 80.0)
		return F117::az / 150.0;

	// High AoA buffet
	if (F117::alpha_DEG > 45 && F117::mach >= 0.1)
		return F117::alpha_DEG / 100.0;

	// Sideslip shaking
	if (F117::beta_DEG > 15 && F117::mach >= 0.05)
		return F117::beta_DEG / 50.0;

	return 0;
}

// Optional DCS easy-flight hook. This FM currently leaves game-mode behavior unchanged.
void ed_fm_set_easy_flight(bool value) 
{}

// DCS hook for the unlimited-fuel option.
void ed_fm_unlimited_fuel(bool value) 
{
		F117::param_class.param_stuff::fuelparam(1-value);
}

// Cold-start initialization.

void ed_fm_cold_start()
{
    apply_ground_start_state(false);
    log_damage_lifecycle_event("ed_fm_cold_start");
}


// Ground hot-start initialization.

void ed_fm_hot_start()
{
    apply_ground_start_state(true);
    log_damage_lifecycle_event("ed_fm_hot_start");
}


// Air-start initialization.

void ed_fm_hot_start_in_air()
{
    apply_air_start_state();
    log_damage_lifecycle_event("ed_fm_hot_start_in_air");
}


// Push a fire event into the simulation event queue
// API layout (wHumanCustomPhysicsAPI.h):
//   [0] = handle, [1-3] = position XYZ, [4-6] = direction XYZ, [7] = speed, [8] = scale
static void pushFireEvent(int handle, double x, double y, double z)
{
	ed_fm_simulation_event evt = {};
	evt.event_type = ED_FM_EVENT_FIRE;
	evt.event_params[0] = (float)handle;  // fire control handle
	evt.event_params[1] = (float)x;       // fire origin X (body space)
	evt.event_params[2] = (float)y;       // fire origin Y
	evt.event_params[3] = (float)z;       // fire origin Z
	evt.event_params[4] = 0.0f;           // emitter direction X
	evt.event_params[5] = 1.0f;           // emitter direction Y (up)
	evt.event_params[6] = 0.0f;           // emitter direction Z
	evt.event_params[7] = 5.0f;           // particle speed
	evt.event_params[8] = 1.0f;           // scale (>0 = fire on, <=0 = fire off)
	g_simEvents.push(evt);
}

// DCS calls this to dequeue simulation events (fire, smoke, failures)
bool ed_fm_pop_simulation_event(ed_fm_simulation_event & out)
{
	if (g_simEvents.empty()) return false;
	out = g_simEvents.front();
	g_simEvents.pop();
	if (g_damageLog)
	{
		fprintf(g_damageLog, "ed_fm_pop_simulation_event: type=%u handle=%.0f pos=(%.1f,%.1f,%.1f) dir=(%.1f,%.1f,%.1f) speed=%.1f scale=%.1f\n",
			out.event_type, out.event_params[0],
			out.event_params[1], out.event_params[2], out.event_params[3],
			out.event_params[4], out.event_params[5], out.event_params[6],
			out.event_params[7], out.event_params[8]);
		fflush(g_damageLog);
	}
	return true;
}


namespace
{
    bool try_open_damage_log_path(const char* path)
    {
        g_damageLog = fopen(path, "w");
        if (!g_damageLog)
        {
            return false;
        }

        fprintf(g_damageLog, "=== F-117 Damage Debug Log ===\n");
        fprintf(g_damageLog, "Log opened at: %s\n", path);
        fprintf(g_damageLog, "invincible_value at init: %d\n\n", static_cast<int>(F117::param_class.invincible_value));
        fflush(g_damageLog);
        return true;
    }

    void log_damage_callback_header(int element, double element_integrity_factor)
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "ed_fm_on_damage called: Element=%d, integrity=%.4f, invincible=%d\n",
                element, element_integrity_factor, static_cast<int>(F117::param_class.invincible_value));
            fflush(g_damageLog);
        }
    }

    void log_damage_callback_skipped_immortal()
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog, "  -> SKIPPED (immortal)\n");
            fflush(g_damageLog);
        }
    }

    const char* apply_damage_for_element(int element, double integrity)
    {
        switch (element)
        {
        case kDamageElementCockpit:
            return "cockpit(ignored, derived from MAIN)";
        case kDamageElementEngine:
            g_damage.leftEngine = integrity;
            g_damage.rightEngine = integrity;
            if (integrity < kFireDamageThreshold)
            {
                pushFireEvent(8, -4.45, 0.08, -1.7);
                pushFireEvent(7, -4.45, 0.08, 1.7);
            }
            return "engines(both)";
        case kDamageElementMain:
            g_damage.leftWing = min(g_damage.leftWing, max(0.0, integrity - (1.0 - integrity) * 0.3));
            g_damage.rightWing = min(g_damage.rightWing, integrity);
            g_damage.leftTail = min(g_damage.leftTail, integrity);
            g_damage.rightTail = min(g_damage.rightTail, integrity);
            g_damage.cockpit = min(g_damage.cockpit, integrity);
            if (integrity < kFireDamageThreshold)
            {
                pushFireEvent(4, -0.82, 0.265, -2.774);
                pushFireEvent(3, -0.82, 0.265, 2.774);
            }
            return "main(wings+tails+cockpit)";
        default:
            return "IGNORED(unhandled)";
        }
    }

    void log_damage_mapping_result(const char* mapped)
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "  -> mapped to: %s | State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                mapped,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit);
            fflush(g_damageLog);
        }
    }
}


static void initDamageLog()
{
    if (g_damageLog) return;

    const std::string paths[] = {
        build_saved_games_dcs_path("DCS", "Logs\\F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS", "F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS.openbeta", "F117_Damage_Debug.log"),
        "F117_Damage_Debug.log"
    };

    for (const std::string& path : paths)
    {
        if (!path.empty() && try_open_damage_log_path(path.c_str()))
        {
            return;
        }
    }
}


// Damage callback. DCS reports remaining integrity where 1.0 is intact and 0.0 is destroyed.

void ed_fm_on_damage(int Element, double element_integrity_factor)
{
    initDamageLog();
    log_damage_callback_header(Element, element_integrity_factor);

    if (F117::param_class.invincible_value == 0)
    {
        log_damage_callback_skipped_immortal();
        return;
    }

    double rawDamage = 1.0 - limit(element_integrity_factor, 0.0, 1.0);
    double integrity = max(0.0, 1.0 - rawDamage * kDamageScale);
    const char* mapped = apply_damage_for_element(Element, integrity);
    log_damage_mapping_result(mapped);
}


// Restore repairable FM damage state and clear queued events.

void ed_fm_repair()
{
    reset_damage_and_events();
}


void ed_fm_set_immortal(bool value)
{
	F117::param_class.param_stuff::invincible(1 - value);
	initDamageLog();
	if (g_damageLog)
	{
		fprintf(g_damageLog, "ed_fm_set_immortal called: value=%d -> invincible_value=%d\n",
			(int)value, (int)F117::param_class.invincible_value);
		fflush(g_damageLog);
	}
}

// Returns true when any component has taken damage, triggering the repair process
bool ed_fm_need_to_be_repaired()
{
	return g_damage.leftWing    < 1.0
		|| g_damage.rightWing   < 1.0
		|| g_damage.leftEngine  < 1.0
		|| g_damage.rightEngine < 1.0
		|| g_damage.leftTail    < 1.0
		|| g_damage.rightTail   < 1.0
		|| g_damage.cockpit     < 1.0;
}

#pragma once
