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
#define EXPORT_ED_FM_PHYSICS_IMP extern "C" __declspec(dllexport)
#include "include/FM/wHumanCustomPhysicsAPI_ImplementationDeclare.h"	// Official DCS EFM API declarations
// Model headers
#include "Actuators/Actuators.h"			//Actuators model functions
#include "Atmosphere/Atmosphere.h"			//Atmosphere model functions
#include "Aerodynamics/Aero.h"				//Aerodynamic model functions
#include "FlightControls/FlightControls.h"	//Flight Controls model functions
#include "Engine/Engine.h"					//Engine model functions
#include "param_functions.h"
#include "Avionics/CockpitBase_Interop.h"

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

Vec3	position_world_cs;

Vec3	velocity_world_cs;

//-------------------------------------------------------
// Aircraft simulation state
//-------------------------------------------------------
namespace F117 // Shared FM state. Many aero/control calculations still use the original imperial-unit tuning.
{
	double		ambientTemperature_DegK = 0.0;			// Ambient temperature (kelvin)
	double		ambientDensity_KgPerM3	= 0.0;			// Ambient density (kg/m^3)
	double		aircraftQuaternionX		= 0.0;			// World-frame orientation quaternion
	double		aircraftQuaternionY		= 0.0;
	double		aircraftQuaternionZ		= 0.0;
	double		aircraftQuaternionW		= 1.0;
	double		wingSpan_M				= 13.205;		// F-117A wing-span (m)
	double		wingArea_M2				= 84.77;		// F-117A wing area (m²)
	double		meanChord_M				= 6.422;		// F-117A mean aerodynamic chord (m)
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
	double		totalVelocity_MPS		= 0.0;			// Total velocity (always positive) (m/s)
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

	double		dynamicPressure_PA		= 0.0;			// Dynamic pressure (Pa)
	double		mach					= 0.0;			// Air speed in Mach; 1 is the local speed of sound.
	bool		simInitialized			= false;		// Has the simulation gone through it's first run frame?
	double		gearDown				= 0.0;			// Is the gear currently down?
	bool		airRefuelDoorOpen	= false;		// Hook for AAR receptacle state when cockpit wiring is available.
	double		az						= 0.0;			// This is the G force felt by the pilot, acting out the bottom of the aircraft (m/s^2), 1 is Earth's gravity.
	double		ay						= 0.0;			// Ay (per normal direction convention) out the right wing (m/s^2)
	double		weight_N				= 131222.538;	// Weight force of aircraft (N)
	double		ay_world				= 0.0;			// World referenced up/down acceleration (m/s^2)
	bool		weight_on_wheels		= false;		// Weight on wheels flag 
	bool		wow_from_draw_args		= false;		// Suspension contact state from DCS draw args

	double		rolling_friction		= 0.015;		// Relative wheel braking/rolling resistance reported to DCS.
	double		WheelBrakeCommand		= 0.0;			// Commanded wheel brake
	double		GearCommand				= 0.0;			// Commanded gear lever
	double		tailhook_command		= 0.0;			// Tail hook command
	double		tailhook_pos			= 0.0;			// Tailhook actuator position reported back to DCS.
	float		rudder_pos				= 0.0;			//Rudder(s) deflection
	float		misc_cmd				= 0.0;			// Generic misc actuator command used for the weapon bay animation.
	float		misc_state				= 0.0;
	float		misc_cmdH				= 0.0;			// Secondary misc actuator command used for the hook-related animation path.
	float		misc_stateH				= 0.0;
    bool        weapon_release_pickle_held = false;
    bool        weapon_release_gate = false;
    bool        weapon_release_gate_fired = false;
    bool        weapon_bay_owned_by_release = false;
    bool        weapon_bay_close_pending = false;
    double      weapon_bay_close_delay = 0.0;

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

	double		surface_height_m = 1000.0;			// Height of terrain surface below aircraft centre (m). Default high to suppress ground effect until first DCS call.
	bool		suspension_wow[3] = { false, false, false }; // Per-gear weight-on-wheels from DCS suspension feedback (0=nose, 1=left main, 2=right main)

	EDPARAM cockpitAPI;
	param_stuff param_class;
}

static F117::DamageState g_damage;
static std::queue<ed_fm_simulation_event> g_simEvents;
static void* g_weaponReleaseGateParam = nullptr;
static bool g_weaponReleaseGateParamInitialized = false;
static void* g_iradsLockValidParam = nullptr;
static void* g_iradsLockWorldXParam = nullptr;
static void* g_iradsLockWorldYParam = nullptr;
static void* g_iradsLockWorldZParam = nullptr;
static void* g_iradsLockRangeParam = nullptr;
static void* g_iradsLockAzimuthParam = nullptr;
static void* g_iradsLockElevationParam = nullptr;
static bool g_iradsLockParamsInitialized = false;
static void* g_releaseCueValidParam = nullptr;
static void* g_releaseCueErrorParam = nullptr;
static void* g_releaseCueTimeToGoParam = nullptr;
static void* g_releaseCueRangeParam = nullptr;
static void* g_releaseCueInZoneParam = nullptr;
static bool g_releaseCueParamsInitialized = false;
static void* g_ccrpProfileParam = nullptr;
static bool g_ccrpProfileParamInitialized = false;


namespace
{
    struct CcrpStoreModel
    {
        const char* name;
        double massKg;
        double dragCd;
        double referenceAreaM2;
        double releaseWindowMeters;
        double minSpeedMPS;
        double minDropMeters;
    };

    constexpr CcrpStoreModel kCcrpStoreModels[] =
    {
        { "GBU-31(V)1/B",   934.0, 0.00264, 0.45,  75.0, 30.0,  10.0 },
        { "GBU-31(V)3/B",   981.0, 0.00170, 0.45,  90.0, 30.0,  10.0 },
        { "GBU-32(V)2/B",   467.0, 0.00035, 0.30,  65.0, 30.0,  10.0 },
        { "GBU-12",         277.0, 0.000413, 0.22,  55.0, 30.0,  10.0 },
        { "GBU-27",         991.0, 0.00071,  0.45,  80.0, 30.0,  10.0 },
    };

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
    constexpr double kWeaponBayOpenThreshold = 0.995;
    constexpr double kWeaponBayClosedThreshold = 0.005;
    constexpr double kWeaponBayHoldAfterReleaseSeconds = 1.0;
    constexpr char kWeaponReleaseGateParamName[] = "F117_WEAPON_RELEASE_GATE";
    constexpr char kIRADSLockValidParamName[] = "F117_IRADS_LOCK_VALID";
    constexpr char kIRADSLockWorldXParamName[] = "F117_IRADS_LOCK_X";
    constexpr char kIRADSLockWorldYParamName[] = "F117_IRADS_LOCK_Y";
    constexpr char kIRADSLockWorldZParamName[] = "F117_IRADS_LOCK_Z";
    constexpr char kIRADSLockRangeParamName[] = "F117_IRADS_LOCK_RANGE";
    constexpr char kIRADSLockAzimuthParamName[] = "F117_IRADS_LOCK_AZ";
    constexpr char kIRADSLockElevationParamName[] = "F117_IRADS_LOCK_EL";
    constexpr char kReleaseCueValidParamName[] = "F117_RELEASE_CUE_VALID";
    constexpr char kReleaseCueErrorParamName[] = "F117_RELEASE_CUE_ERROR";
    constexpr char kReleaseCueTimeToGoParamName[] = "F117_RELEASE_CUE_TTG";
    constexpr char kReleaseCueRangeParamName[] = "F117_RELEASE_CUE_RANGE";
    constexpr char kReleaseCueInZoneParamName[] = "F117_RELEASE_CUE_IN_ZONE";
    constexpr char kCcrpProfileParamName[] = "F117_CCRP_PROFILE";
    constexpr double kGravity_MPS2 = 9.81;
    constexpr double kReleaseCueMinSpeedMPS = 30.0;
    constexpr double kReleaseCueMinDropMeters = 10.0;
    constexpr double kAtmosphereScaleHeight_M = 8500.0;
    constexpr double kReleaseIntegrationStepSeconds = 0.02;
    constexpr double kReleaseIntegrationMaxSeconds = 180.0;

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

    constexpr double kGroundStartIdleN2 = 62.0;
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

        const bool suspensionWeightOnWheels = F117::suspension_wow[0] || F117::suspension_wow[1] || F117::suspension_wow[2];
        F117::weight_on_wheels = F117::wow_from_draw_args || physicsWeightOnWheels || suspensionWeightOnWheels;
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
        {
            const double rudderKeyBias = 101.0 / (beta_DEG * beta_DEG + 100.0);
            F117::pedInput = limit(value + rudderKeyBias, -1.0, 1.0);
            return true;
        }
        case rudderleftend:
            F117::pedInput = 0.0;
            return true;
        case ruddertrimLeft:
            F117::yawTrim += kYawTrimStep;
            return true;
        case rudderright:
        {
            const double rudderKeyBias = 101.0 / (beta_DEG * beta_DEG + 100.0);
            F117::pedInput = limit(-value - rudderKeyBias, -1.0, 1.0);
            return true;
        }
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
    void initialize_weapon_release_gate_param()
    {
        if (g_weaponReleaseGateParamInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_weaponReleaseGateParam = F117::cockpitAPI.getParamHandle(kWeaponReleaseGateParamName);
        g_weaponReleaseGateParamInitialized = (g_weaponReleaseGateParam != nullptr);
    }

    void publish_weapon_release_gate_param()
    {
        initialize_weapon_release_gate_param();
        if (g_weaponReleaseGateParam != nullptr)
        {
            F117::cockpitAPI.setParamNumber(g_weaponReleaseGateParam, F117::weapon_release_gate ? 1.0 : 0.0);
        }
    }

    void initialize_irads_lock_params()
    {
        if (g_iradsLockParamsInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_iradsLockValidParam = F117::cockpitAPI.getParamHandle(kIRADSLockValidParamName);
        g_iradsLockWorldXParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldXParamName);
        g_iradsLockWorldYParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldYParamName);
        g_iradsLockWorldZParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldZParamName);
        g_iradsLockRangeParam = F117::cockpitAPI.getParamHandle(kIRADSLockRangeParamName);
        g_iradsLockAzimuthParam = F117::cockpitAPI.getParamHandle(kIRADSLockAzimuthParamName);
        g_iradsLockElevationParam = F117::cockpitAPI.getParamHandle(kIRADSLockElevationParamName);
        g_iradsLockParamsInitialized =
            (g_iradsLockValidParam != nullptr) &&
            (g_iradsLockWorldXParam != nullptr) &&
            (g_iradsLockWorldYParam != nullptr) &&
            (g_iradsLockWorldZParam != nullptr) &&
            (g_iradsLockRangeParam != nullptr) &&
            (g_iradsLockAzimuthParam != nullptr) &&
            (g_iradsLockElevationParam != nullptr);
    }

    void initialize_release_cue_params()
    {
        if (g_releaseCueParamsInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_releaseCueValidParam = F117::cockpitAPI.getParamHandle(kReleaseCueValidParamName);
        g_releaseCueErrorParam = F117::cockpitAPI.getParamHandle(kReleaseCueErrorParamName);
        g_releaseCueTimeToGoParam = F117::cockpitAPI.getParamHandle(kReleaseCueTimeToGoParamName);
        g_releaseCueRangeParam = F117::cockpitAPI.getParamHandle(kReleaseCueRangeParamName);
        g_releaseCueInZoneParam = F117::cockpitAPI.getParamHandle(kReleaseCueInZoneParamName);
        g_releaseCueParamsInitialized =
            (g_releaseCueValidParam != nullptr) &&
            (g_releaseCueErrorParam != nullptr) &&
            (g_releaseCueTimeToGoParam != nullptr) &&
            (g_releaseCueRangeParam != nullptr) &&
            (g_releaseCueInZoneParam != nullptr);
    }

    void initialize_ccrp_profile_param()
    {
        if (g_ccrpProfileParamInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_ccrpProfileParam = F117::cockpitAPI.getParamHandle(kCcrpProfileParamName);
        g_ccrpProfileParamInitialized = (g_ccrpProfileParam != nullptr);
    }

    int get_ccrp_profile_index()
    {
        initialize_ccrp_profile_param();
        if (!g_ccrpProfileParamInitialized)
        {
            return 0;
        }

        const double rawIndex = F117::cockpitAPI.getParamNumber(g_ccrpProfileParam);
        int profileIndex = (int)floor(rawIndex + 0.5);
        const int profileCount = (int)(sizeof(kCcrpStoreModels) / sizeof(kCcrpStoreModels[0]));
        if (profileIndex < 0)
        {
            profileIndex = 0;
        }
        if (profileIndex >= profileCount)
        {
            profileIndex = profileCount - 1;
        }

        return profileIndex;
    }

    const CcrpStoreModel& get_ccrp_profile()
    {
        return kCcrpStoreModels[get_ccrp_profile_index()];
    }

    Vec3 quaternion_to_world(const Vec3& bodyVector)
    {
        const double qx = F117::aircraftQuaternionX;
        const double qy = F117::aircraftQuaternionY;
        const double qz = F117::aircraftQuaternionZ;
        const double qw = F117::aircraftQuaternionW;

        const double mag = sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw));
        if (mag < 1.0e-6)
        {
            return bodyVector;
        }

        const double x = qx / mag;
        const double y = qy / mag;
        const double z = qz / mag;
        const double w = qw / mag;

        Vec3 worldVector{};
        worldVector.x = ((1.0 - (2.0 * y * y) - (2.0 * z * z)) * bodyVector.x) + ((2.0 * x * y - 2.0 * z * w) * bodyVector.y) + ((2.0 * x * z + 2.0 * y * w) * bodyVector.z);
        worldVector.y = ((2.0 * x * y + 2.0 * z * w) * bodyVector.x) + ((1.0 - (2.0 * x * x) - (2.0 * z * z)) * bodyVector.y) + ((2.0 * y * z - 2.0 * x * w) * bodyVector.z);
        worldVector.z = ((2.0 * x * z - 2.0 * y * w) * bodyVector.x) + ((2.0 * y * z + 2.0 * x * w) * bodyVector.y) + ((1.0 - (2.0 * x * x) - (2.0 * y * y)) * bodyVector.z);
        return worldVector;
    }

    Vec3 get_world_wind_vector()
    {
        return quaternion_to_world(wind);
    }

    bool simulate_bomb_impact_point(const CcrpStoreModel& profile, const Vec3& releasePos, const Vec3& releaseVel, const Vec3& worldWind, double targetAltMeters, Vec3* outImpactPos, double* outImpactTime)
    {
        if (outImpactPos == nullptr || outImpactTime == nullptr || profile.massKg <= 0.0 || profile.referenceAreaM2 <= 0.0)
        {
            return false;
        }

        if (releasePos.y <= targetAltMeters)
        {
            return false;
        }

        const double altitudeReference = max(releasePos.y, 0.0);
        const double seaLevelDensity = max(F117::ambientDensity_KgPerM3, 0.05) * exp(altitudeReference / kAtmosphereScaleHeight_M);

        Vec3 pos = releasePos;
        Vec3 vel = releaseVel;
        Vec3 prevPos = pos;
        double elapsed = 0.0;

        while (elapsed < kReleaseIntegrationMaxSeconds)
        {
            prevPos = pos;

            Vec3 relVel{};
            relVel.x = vel.x - worldWind.x;
            relVel.y = vel.y - worldWind.y;
            relVel.z = vel.z - worldWind.z;

            const double relSpeed = sqrt((relVel.x * relVel.x) + (relVel.y * relVel.y) + (relVel.z * relVel.z));
            Vec3 acc{};
            acc.x = 0.0;
            acc.y = -kGravity_MPS2;
            acc.z = 0.0;

            if (relSpeed > 0.25)
            {
                const double rho = seaLevelDensity * exp(-max(pos.y, 0.0) / kAtmosphereScaleHeight_M);
                const double dragScale = 0.5 * rho * profile.dragCd * profile.referenceAreaM2 / profile.massKg;
                const double dragAccel = dragScale * relSpeed * relSpeed;
                const double invRelSpeed = 1.0 / relSpeed;
                acc.x -= dragAccel * relVel.x * invRelSpeed;
                acc.y -= dragAccel * relVel.y * invRelSpeed;
                acc.z -= dragAccel * relVel.z * invRelSpeed;
            }

            vel.x += acc.x * kReleaseIntegrationStepSeconds;
            vel.y += acc.y * kReleaseIntegrationStepSeconds;
            vel.z += acc.z * kReleaseIntegrationStepSeconds;

            pos.x += vel.x * kReleaseIntegrationStepSeconds;
            pos.y += vel.y * kReleaseIntegrationStepSeconds;
            pos.z += vel.z * kReleaseIntegrationStepSeconds;
            elapsed += kReleaseIntegrationStepSeconds;

            if (pos.y <= targetAltMeters)
            {
                const double denom = prevPos.y - pos.y;
                const double fraction = (fabs(denom) > 1.0e-6) ? ((prevPos.y - targetAltMeters) / denom) : 1.0;
                outImpactPos->x = prevPos.x + ((pos.x - prevPos.x) * fraction);
                outImpactPos->y = targetAltMeters;
                outImpactPos->z = prevPos.z + ((pos.z - prevPos.z) * fraction);
                *outImpactTime = max(0.0, elapsed - kReleaseIntegrationStepSeconds + (kReleaseIntegrationStepSeconds * fraction));
                return true;
            }
        }

        return false;
    }

    bool compute_release_solution(double* outErrorMeters, double* outTimeToGoSeconds, double* outRangeMeters, bool* outInZone)
    {
        if (outErrorMeters == nullptr || outTimeToGoSeconds == nullptr || outRangeMeters == nullptr || outInZone == nullptr)
        {
            return false;
        }

        double targetWorld[3] = {0.0, 0.0, 0.0};
        if (!CockpitInterop::GetIRADSLockWorldPoint(targetWorld))
        {
            return false;
        }

        const CcrpStoreModel& profile = get_ccrp_profile();
        const double dx = targetWorld[0] - position_world_cs.x;
        const double dz = targetWorld[2] - position_world_cs.z;
        const double horizSpeed = sqrt((velocity_world_cs.x * velocity_world_cs.x) + (velocity_world_cs.z * velocity_world_cs.z));
        const double horizRange = sqrt((dx * dx) + (dz * dz));
        const double dropMeters = position_world_cs.y - targetWorld[1];

        if (horizSpeed < max(kReleaseCueMinSpeedMPS, profile.minSpeedMPS) || dropMeters < max(kReleaseCueMinDropMeters, profile.minDropMeters))
        {
            return false;
        }

        const double vx = velocity_world_cs.x / horizSpeed;
        const double vz = velocity_world_cs.z / horizSpeed;
        const double alongTrack = (dx * vx) + (dz * vz);
        if (alongTrack <= 0.0)
        {
            return false;
        }

        const double crossTrackSq = (horizRange * horizRange) - (alongTrack * alongTrack);
        const double crossTrack = sqrt((crossTrackSq > 0.0) ? crossTrackSq : 0.0);
        const Vec3 worldWind = get_world_wind_vector();
        Vec3 impactPos{};
        double impactTime = 0.0;
        if (!simulate_bomb_impact_point(profile, position_world_cs, velocity_world_cs, worldWind, targetWorld[1], &impactPos, &impactTime))
        {
            return false;
        }
        (void)impactTime;

        const Vec3 impactDelta
        {
            impactPos.x - position_world_cs.x,
            impactPos.y - position_world_cs.y,
            impactPos.z - position_world_cs.z
        };
        const double impactAlongTrack = (impactDelta.x * vx) + (impactDelta.z * vz);
        const double errorMeters = alongTrack - impactAlongTrack;
        const double timeToGo = (errorMeters > 0.0) ? (errorMeters / max(horizSpeed, profile.minSpeedMPS)) : 0.0;
        const double acceptableWindow = max(profile.releaseWindowMeters, max(25.0, alongTrack * 0.02));
        const bool inZone = (fabs(errorMeters) <= acceptableWindow) && (crossTrack <= max(100.0, alongTrack * 0.15));

        *outErrorMeters = errorMeters;
        *outTimeToGoSeconds = timeToGo;
        *outRangeMeters = alongTrack;
        *outInZone = inZone;
        return true;
    }

    void publish_release_cue_params()
    {
        initialize_release_cue_params();
        if (!g_releaseCueParamsInitialized)
        {
            return;
        }

        double errorMeters = 0.0;
        double timeToGoSeconds = 0.0;
        double rangeMeters = 0.0;
        bool inZone = false;
        const bool valid = CockpitInterop::HasIRADSLock() &&
            compute_release_solution(&errorMeters, &timeToGoSeconds, &rangeMeters, &inZone);

        if (valid)
        {
            F117::cockpitAPI.setParamNumber(g_releaseCueValidParam, 1.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueErrorParam, errorMeters);
            F117::cockpitAPI.setParamNumber(g_releaseCueTimeToGoParam, timeToGoSeconds);
            F117::cockpitAPI.setParamNumber(g_releaseCueRangeParam, rangeMeters);
            F117::cockpitAPI.setParamNumber(g_releaseCueInZoneParam, inZone ? 1.0 : 0.0);
        }
        else
        {
            F117::cockpitAPI.setParamNumber(g_releaseCueValidParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueErrorParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueTimeToGoParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueRangeParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueInZoneParam, 0.0);
        }
    }

    void publish_irads_lock_params()
    {
        initialize_irads_lock_params();
        if (!g_iradsLockParamsInitialized)
        {
            return;
        }

        double world[3] = {0.0, 0.0, 0.0};
        double polar[3] = {0.0, 0.0, 0.0};
        const bool valid = CockpitInterop::HasIRADSLock() &&
            CockpitInterop::GetIRADSLockWorldPoint(world) &&
            CockpitInterop::GetIRADSLockPolar(polar);

        if (valid)
        {
            F117::cockpitAPI.setParamNumber(g_iradsLockValidParam, 1.0);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldXParam, world[0]);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldYParam, world[1]);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldZParam, world[2]);
            F117::cockpitAPI.setParamNumber(g_iradsLockRangeParam, polar[2]);
            F117::cockpitAPI.setParamNumber(g_iradsLockAzimuthParam, polar[0]);
            F117::cockpitAPI.setParamNumber(g_iradsLockElevationParam, polar[1]);
        }
        else
        {
            F117::cockpitAPI.setParamNumber(g_iradsLockValidParam, 0.0);
        }
    }

    bool weapon_bay_is_open()
    {
        return F117::misc_state >= kWeaponBayOpenThreshold;
    }

    bool weapon_bay_is_closed()
    {
        return F117::misc_state <= kWeaponBayClosedThreshold;
    }

    void command_weapon_bay_open()
    {
        F117::misc_cmd = 1.0f;
    }

    void command_weapon_bay_close()
    {
        F117::misc_cmd = 0.0f;
    }

    void reset_weapon_release_sequence()
    {
        F117::weapon_release_pickle_held = false;
        F117::weapon_release_gate = false;
        F117::weapon_release_gate_fired = false;
        F117::weapon_bay_owned_by_release = false;
        F117::weapon_bay_close_pending = false;
        F117::weapon_bay_close_delay = 0.0;
        publish_weapon_release_gate_param();
    }

    void handle_weapon_release_pressed()
    {
        F117::weapon_bay_close_pending = false;
        F117::weapon_bay_close_delay = 0.0;
        F117::weapon_release_pickle_held = true;
        F117::weapon_release_gate = false;
        F117::weapon_release_gate_fired = false;

        if (!weapon_bay_is_open())
        {
            command_weapon_bay_open();
            F117::weapon_bay_owned_by_release = true;
        }
        else
        {
            F117::weapon_bay_owned_by_release = false;
        }

        publish_weapon_release_gate_param();
    }

    void handle_weapon_release_released()
    {
        F117::weapon_release_pickle_held = false;
        F117::weapon_release_gate = false;
        F117::weapon_release_gate_fired = false;

        if (F117::weapon_bay_owned_by_release)
        {
            if (weapon_bay_is_open())
            {
                F117::weapon_bay_close_pending = true;
                F117::weapon_bay_close_delay = kWeaponBayHoldAfterReleaseSeconds;
            }
            else
            {
                command_weapon_bay_close();
                F117::weapon_bay_close_pending = false;
                F117::weapon_bay_close_delay = 0.0;
            }
        }
        else
        {
            F117::weapon_bay_close_pending = false;
            F117::weapon_bay_close_delay = 0.0;
        }

        publish_weapon_release_gate_param();
    }

    void update_weapon_release_sequence(double dt)
    {
        double errorMeters = 0.0;
        double timeToGoSeconds = 0.0;
        double rangeMeters = 0.0;
        bool inZone = false;
        const bool hasSolution = CockpitInterop::HasIRADSLock() &&
            compute_release_solution(&errorMeters, &timeToGoSeconds, &rangeMeters, &inZone);

        const bool releaseReady = F117::weapon_release_pickle_held &&
            weapon_bay_is_open() &&
            hasSolution &&
            inZone &&
            !F117::weapon_release_gate_fired;

        F117::weapon_release_gate = releaseReady;
        if (releaseReady)
        {
            F117::weapon_release_gate_fired = true;
        }

        if (!F117::weapon_release_pickle_held &&
            F117::weapon_bay_owned_by_release &&
            weapon_bay_is_closed())
        {
            F117::weapon_bay_owned_by_release = false;
            F117::weapon_bay_close_pending = false;
            F117::weapon_bay_close_delay = 0.0;
        }

        if (!F117::weapon_release_pickle_held && F117::weapon_bay_close_pending)
        {
            F117::weapon_bay_close_delay = max(0.0, F117::weapon_bay_close_delay - dt);
            if (F117::weapon_bay_close_delay <= 0.0)
            {
                command_weapon_bay_close();
                F117::weapon_bay_close_pending = false;
            }
        }

        publish_release_cue_params();
        publish_weapon_release_gate_param();
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
        case AirRefuel:
            F117::airRefuelDoorOpen = !F117::airRefuelDoorOpen;
            return true;
        case bombay:
            reset_weapon_release_sequence();
            if (misc_state < 0.5)
            {
                command_weapon_bay_open();
            }
            else if (misc_state > 0.5)
            {
                command_weapon_bay_close();
            }
            return true;
        case bombayOpen:
            reset_weapon_release_sequence();
            command_weapon_bay_open();
            return true;
        case bombayClose:
            reset_weapon_release_sequence();
            command_weapon_bay_close();
            return true;
        case pickleOn:
        case weaponReleaseHoldOn:
            handle_weapon_release_pressed();
            return true;
        case pickleOff:
        case weaponReleaseHoldOff:
            handle_weapon_release_released();
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

    F117::totalVelocity_MPS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);
    if (F117::totalVelocity_MPS < 0.003)
    {
        F117::totalVelocity_MPS = 0.003; // ~0.01 ft/s equivalent minimum
    }
}

void update_atmosphere_from_total_velocity(double* temp)
{
    F117::ATMOS::atmos(F117::ambientTemperature_DegK, F117::ambientDensity_KgPerM3, F117::totalVelocity_MPS, temp);
    F117::dynamicPressure_PA = temp[0];
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
    // SFC (cemax) calculation:
    // Thrust (Newtons) -> kgf (divide by 9.81)
    // kg/kgf/hr (1.24) -> kg/s (divide by 3600)
    const double cemax = 1.24;
    F117::fuel_consumption_since_last_time = (F117::thrust_N / 9.81) * (cemax / 3600.0) * dt;
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

    F117::FLIGHTCONTROLS::update_pitch_mode_auto_inputs(F117::gearDown, F117::airRefuelDoorOpen);

    F117::elevator_DEG_commanded = -(F117::FLIGHTCONTROLS::fcs_pitch_controller(
        F117::FLIGHTCONTROLS::longStickInput * controlDegradation,
        0.0,
        F117::alpha_DEG,
        F117::pitchRate_RPS * F117::radiansToDegrees,
        (F117::az / 9.81),
        0.0,
        F117::dynamicPressure_PA,
        dt,
        F117::roll_angle,
        F117::pitch_angle,
        F117::rollRate_RPS * F117::radiansToDegrees,
        F117::totalVelocity_MPS,
        F117::mach,
        F117::thrust_N,
        F117::AERO::Cx_total));
    // Pitch axis: Loschke specifies -25° TEU (nose-down) to +37.5° TED (nose-up) for pitch.
    // Physical actuator stop is ±45° — pitch authority does not reach the stop.
    F117::elevator_DEG = limit(F117::elevator_DEG_commanded + F117::pitchTrim, -25.0, 37.5);

    F117::aileron_DEG_commanded = F117::FLIGHTCONTROLS::fcs_roll_controller(
        F117::FLIGHTCONTROLS::latStickInput * controlDegradation,
        F117::FLIGHTCONTROLS::longStickForce,
        F117::ay / 9.81,
        F117::pedInput,
        F117::beta_DEG,
        F117::roll_angle,
        F117::rollRate_RPS * F117::radiansToDegrees,
        0.0,
        F117::dynamicPressure_PA,
        F117::gearDown > 0.5,
        F117::airRefuelDoorOpen,
        dt);

    // Real F-117 pitch/roll mixer: pitch has priority.
    // Each elevon physical stop is ±45°. Pitch occupies elevator_DEG of that travel.
    // Roll gets whatever remains: max roll deflection = 45° - |elevator_DEG|.
    // This means rolling pull-outs automatically have less roll authority at high pitch demand,
    // exactly as the real aircraft (and consistent with Loschke's mixer description).
    const double maxRollDEG = (std::max)(0.0, 45.0 - std::abs(F117::elevator_DEG));
    F117::aileron_DEG = limit(F117::aileron_DEG_commanded + F117::rollTrim, -maxRollDEG, maxRollDEG);

    F117::dragchute = F117::ACTUATORS::dragchute_actuator(
        F117::dragchute_command,
        dt,
        F117::totalVelocity_MPS,
        F117::gearDown,
        F117::weight_on_wheels);

    // Keep the high-beta/high-yaw-rate rudder limiter symmetric left vs right.
    const double betaMagnitude_DEG = std::abs(beta_DEG);
    const double yawRateMagnitude_DEG_s = std::abs(yawRate_RPS * radiansToDegrees);
    aos_filter = static_cast<float>(1.0 + (betaMagnitude_DEG * betaMagnitude_DEG) / 7500.0 * (1.0 + yawRateMagnitude_DEG_s / 90.0));

    F117::rudder_DEG_commanded = F117::FLIGHTCONTROLS::fcs_yaw_controller(
        F117::pedInput,
        0.0,
        F117::beta_DEG,
        F117::yawRate_RPS * F117::radiansToDegrees,
        F117::rollRate_RPS * F117::radiansToDegrees,
        F117::pitchRate_RPS * F117::radiansToDegrees,
        F117::FLIGHTCONTROLS::alphaFiltered,
        F117::dynamicPressure_PA,
        F117::gearDown > 0.5,
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
    F117::thrust_N = F117::ENGINE::engine_dynamics(damagedThrottle, F117::mach, F117::altitude_m, dt, engineRunning, F117::weight_on_wheels);

    const double wingIntegrity = 1.0 - g_damage.totalWingLoss;
    F117::aileron_PCT = (F117::aileron_DEG * wingIntegrity) / 25.5;
    F117::elevator_PCT = (F117::elevator_DEG * (1.0 - g_damage.totalTailLoss)) / 25.0;

    const double tailAsymmetry = g_damage.leftTail - g_damage.rightTail;
    F117::rudder_PCT = (F117::rudder_DEG * tailIntegrity) / 30.0 + tailAsymmetry * 0.1;
    F117::FLIGHTCONTROLS::update_yaw_debug_snapshot(
        F117::beta_DEG,
        F117::yawRate_RPS,
        F117::rudder_DEG_commanded,
        F117::rudder_DEG,
        F117::rudder_PCT);
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

    // Wieselberger ground effect: phi approaches 0 near the ground (reducing induced drag)
    // and approaches 1 in free air (no effect). h/b = height / wingspan.
    const double h_over_b    = F117::surface_height_m / F117::wingSpan_M;
    const double phi_GE      = (16.0 * h_over_b * 16.0 * h_over_b) / (1.0 + 16.0 * h_over_b * 16.0 * h_over_b);
    const double Cd_induced  = K_induced * Cz * Cz * phi_GE;

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

    F117::AERO::dXdQ = (F117::meanChord_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cxq;
    F117::AERO::Cx_total = F117::AERO::Cx + F117::AERO::dXdQ * F117::pitchRate_RPS;
    F117::AERO::Cx_total += CxGear + Cxchute + Cxbay;

    F117::AERO::dZdQ = (F117::meanChord_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Czq;
    F117::AERO::Cz_total = F117::AERO::Cz + F117::AERO::dZdQ * F117::pitchRate_RPS;
    F117::AERO::Cz_total += CzGear + Czbay;

    F117::AERO::dMdQ = (F117::meanChord_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cmq;
    F117::AERO::Cm_total = F117::AERO::Cm * F117::AERO::eta_el + F117::AERO::Cz_total * (F117::referenceCG_PCT - F117::actualCG_PCT) + F117::AERO::dMdQ * F117::pitchRate_RPS + F117::AERO::Cm_delta + F117::AERO::Cm_delta_ds + F117::Cm0;

    F117::AERO::dYdail = F117::AERO::Cy_delta_a20;
    F117::AERO::dYdR = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cyr;
    F117::AERO::dYdP = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cyp;
    F117::AERO::Cy_total = F117::AERO::Cy + F117::AERO::dYdail * F117::aileron_PCT + F117::AERO::Cy_delta_r30 * F117::rudder_PCT + F117::AERO::dYdR * F117::yawRate_RPS + F117::AERO::dYdP * F117::rollRate_RPS;

    F117::AERO::dNdail = F117::AERO::Cn_delta_a20;
    F117::AERO::dNdR = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cnr;
    F117::AERO::dNdP = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Cnp;
    F117::AERO::Cn_total = F117::AERO::Cn - F117::AERO::Cy_total * (F117::referenceCG_PCT - F117::actualCG_PCT) * (F117::meanChord_M / F117::wingSpan_M) + F117::AERO::dNdail * F117::aileron_PCT + F117::AERO::Cn_delta_r30 * F117::rudder_PCT + F117::AERO::dNdR * F117::yawRate_RPS + F117::AERO::dNdP * F117::rollRate_RPS + F117::AERO::Cn_delta_beta * F117::beta_DEG;

    F117::AERO::dLdail = F117::AERO::Cl_delta_a20;
    F117::AERO::dLdR = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Clr;
    F117::AERO::dLdP = (F117::wingSpan_M / (2 * F117::totalVelocity_MPS)) * F117::AERO::Clp;
    F117::AERO::Cl_total = F117::AERO::Cl + F117::AERO::dLdail * F117::aileron_PCT + F117::AERO::Cl_delta_r30 * F117::rudder_PCT + F117::AERO::dLdR * F117::yawRate_RPS + F117::AERO::dLdP * F117::rollRate_RPS + F117::AERO::Cl_delta_beta * F117::beta_DEG;

    F117::FLIGHTCONTROLS::update_aero_debug_snapshot(
        F117::AERO::Cy_delta_r30,
        F117::AERO::Cn_delta_r30,
        F117::AERO::Cl_delta_r30,
        F117::AERO::Cn_delta_beta,
        F117::AERO::Cl_delta_beta,
        F117::AERO::Cy_total,
        F117::AERO::Cn_total,
        F117::AERO::Cl_total);
}

double apply_aerodynamic_forces_and_thrust()
{
    const double qS = F117::wingArea_M2 * F117::dynamicPressure_PA; // N

    Vec3 cy_force(0.0, 0.0, F117::AERO::Cy_total * qS);
    Vec3 cy_force_pos(0.0, 0, 0);
    add_local_force(cy_force, cy_force_pos);

    Vec3 cx_force(-F117::AERO::Cx_total * qS, 0, 0);
    Vec3 cx_force_pos(0, 0.0, 0.0);
    add_local_force(cx_force, cx_force_pos);

    const double liftDamageFactor = 1.0 - g_damage.totalWingLoss;
    Vec3 cz_force(0.0, -F117::AERO::Cz_total * qS * liftDamageFactor, 0.0);
    Vec3 cz_force_pos(0, 0, 0);
    add_local_force(cz_force, cz_force_pos);

    const double qS_b = qS * F117::wingSpan_M; // N·m
    double rollMoment = F117::AERO::Cl_total * qS_b;
    rollMoment += g_damage.wingAsymmetry * qS_b * 0.05;
    Vec3 cl_moment(rollMoment, 0.0, 0.0);
    add_local_moment(cl_moment);

    const double pitchDamageFactor = 1.0 - g_damage.totalTailLoss * 0.5;
    const double noseDownBias = -g_damage.totalWingLoss * qS_b * 0.08;
    Vec3 cm_moment(0.0, 0.0, F117::AERO::Cm_total * qS * F117::meanChord_M * pitchDamageFactor + noseDownBias);
    add_local_moment(cm_moment);

    const double qS_span = qS_b; // N·m (same as qS_b for yaw)
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

	CockpitInterop::ForceIRADSIndicatorActive();
	CockpitInterop::UpdateIRADSSensorFrame(dt,
		position_world_cs.x, position_world_cs.y, position_world_cs.z,
		F117::aircraftQuaternionX, F117::aircraftQuaternionY,
		F117::aircraftQuaternionZ, F117::aircraftQuaternionW);
	publish_irads_lock_params();

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
update_weapon_release_sequence(dt);
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
					engineDebugLog << "engine1_thrust_N,engine2_thrust_N,total_thrust_N,throttle_pct,N2_rpm_pct,mach,velocity_mps,altitude_m,"
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
				               << F117::totalVelocity_MPS << ","
				               << F117::altitude_m << ","
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

void ed_fm_set_surface(double h, double /*h_obj*/, unsigned /*surface_type*/,
                       double /*normal_x*/, double /*normal_y*/, double /*normal_z*/)
{
    // h is the height of the terrain surface directly below the aircraft centre of mass.
    // Used for ground effect modelling in update_aerodynamic_coefficients.
    F117::surface_height_m = h;
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
	position_world_cs.x = px;
	position_world_cs.y = py;
	position_world_cs.z = pz;
	F117::aircraftQuaternionX = quaternion_x;
	F117::aircraftQuaternionY = quaternion_y;
	F117::aircraftQuaternionZ = quaternion_z;
	F117::aircraftQuaternionW = quaternion_w;

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
    if (CockpitInterop::HandleIRADSCommand(command, value))
    {
        return;
    }

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

void ed_fm_refueling_add_fuel(double fuel)
{
	F117::internal_fuel += fuel;
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
        reset_weapon_release_sequence();
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
        reset_weapon_release_sequence();
    }
}

// Reset transient FM state when the aircraft is destroyed, restarted, or unloaded.

void ed_fm_release()
{
    F117::DeltaTime = 0;
    F117::simInitialized = false;
    F117::ACTUATORS::simInitialized = false;
    F117::FLIGHTCONTROLS::reset_runtime_state();
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
    reset_weapon_release_sequence();

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

bool ed_fm_LERX_vortex_update(unsigned idx, LERX_vortex& out)
{
    if (idx > 1)
        return false;

    out.version = 0;

    if (F117::weight_on_wheels || F117::totalVelocity_MPS < 35.0)
    {
        out.spline = nullptr;
        out.spline_points_count = 0;
        return true;
    }

    const float aoa = (float)F117::alpha_DEG;
    const float beta = (float)F117::beta_DEG;
    const float vel = (float)F117::totalVelocity_MPS;
    float rho = (float)F117::ambientDensity_KgPerM3;
    if (rho < 0.1f) rho = 1.225f;

    // Only advance time on the first vortex call per frame (idx==0).
    // Both vortices share the same animation clock; advancing on every call
    // would double the rate since DCS calls this once for each idx per frame.
    static float vortex_time = 0.0f;
    if (idx == 0)
        vortex_time += (float)F117::DeltaTime;
    float time = vortex_time;

    // =========================================================================
    // AoA onset — sigmoid centred at (aoa_onset) deg with sharpness 1.4.  
    // =========================================================================
    const float aoa_onset = 5.0f;
    const float sharpness = 1.4f;
    float vortex_strength = 1.0f / (1.0f + expf(-(aoa - aoa_onset) * sharpness));
    vortex_strength = (std::max)(0.0f, (std::min)(1.0f, vortex_strength));

    if (vortex_strength < 0.02f)
    {
        out.spline = nullptr;
        out.spline_points_count = 0;
        return true;
    }

    // =========================================================================
    // ALTITUDE-DEPENDENT CONDENSATION FADE
    //
    // Visible wing vapor requires ambient moisture to nucleate in the vortex
    // core's low-pressure region.  Temperature lapse ~2 deg C / 1000 ft,
    // dew-point lapse ~0.5 deg C / 1000 ft — the spread widens with altitude
    // making condensation progressively harder.  Ambient density captures both
    // pressure and temperature effects via the standard atmosphere.
    //
    //   Sea level  (rho ~1.225):  humidity_factor ~ 1.0
    //   10,000 ft  (rho ~0.90):   humidity_factor ~ 0.85
    //   20,000 ft  (rho ~0.65):   humidity_factor ~ 0.40
    //   30,000 ft  (rho ~0.46):   humidity_factor ~ 0.10
    //   40,000 ft  (rho ~0.30):   humidity_factor ~ 0.02
    //
    // Additionally use altitude_m directly for a hard ceiling to handle any
    // non-standard atmosphere edge cases (hot day at altitude, etc).
    // =========================================================================
    const float rho_sea_level = 1.225f;
    float rho_ratio = rho / rho_sea_level;
    float humidity_factor = powf(rho_ratio, 3.5f);

    // Low-altitude moisture boost (below ~500 m / ~1500 ft)
    if (rho_ratio > 0.95f)
    {
        float low_alt_boost = (rho_ratio - 0.95f) / 0.05f;
        humidity_factor += low_alt_boost * 0.08f;
    }

    // Hard ceiling via actual altitude — zero above ~10,000 m (~33,000 ft)
    float alt_m = (float)F117::altitude_m;
    if (alt_m > 8000.0f)
    {
        float alt_fade = 1.0f - (alt_m - 8000.0f) / 2000.0f;
        humidity_factor *= (std::max)(0.0f, alt_fade);
    }

    humidity_factor = (std::max)(0.0f, (std::min)(1.0f, humidity_factor));

    if (humidity_factor < 0.01f || vortex_strength * humidity_factor < 0.02f)
    {
        out.spline = nullptr;
        out.spline_points_count = 0;
        return true;
    }

    // =========================================================================
    // Sideslip asymmetry — F-117 faceted geometry amplifies windward/leeward
    // vortex strength difference more than a smooth-skinned delta.
    // =========================================================================
    float side_bias = (idx == 1) ? beta : -beta;
    float asym = 1.0f + side_bias * 0.12f;
    asym = (std::max)(0.15f, (std::min)(1.85f, asym));

    // --- Density / speed scaling ---
    float reynolds = rho * vel;
    float density_scale = (std::min)(1.2f, (std::max)(0.6f, reynolds / 50000.0f));
    float speed_factor = (std::min)(1.3f, (std::max)(0.7f, vel / 120.0f));

    out.opacity = vortex_strength * 0.10f * asym * density_scale * humidity_factor;

    // =========================================================================
    // VORTEX BURST POSITION
    //
    // On a 67.5 deg sweep delta, burst sits well aft of trailing edge at low
    // alpha and migrates forward with increasing AoA.
    //
    // Experimental data for 70 deg delta (closest published sweep to 67.5):
    //   ~10 deg AoA: burst at ~120% root chord (downstream of TE)
    //   ~20 deg AoA: burst at ~70% root chord
    //   ~30 deg AoA: burst at ~30% root chord
    //   ~40 deg AoA: burst at ~4% (near apex)
    // =========================================================================
    const float spline_length = 8.7f;
    float burst_frac;
    if (aoa < 5.0f)
        burst_frac = 1.5f;
    else if (aoa < 30.0f)
        burst_frac = 1.5f - (aoa - 5.0f) * (1.2f / 25.0f);
    else
        burst_frac = 0.3f - (aoa - 30.0f) * 0.02f;
    burst_frac = (std::max)(0.05f, (std::min)(1.5f, burst_frac));

    out.explosion_start = burst_frac * spline_length * speed_factor;
    out.explosion_start = (std::max)(0.5f, out.explosion_start);

    // =========================================================================
    // PARAMETRIC PATH — F-117 specific geometry
    //
    // Vortex originates at the wing / inlet junction and tracks inboard of
    // the leading edge at ~15-20% local semi-span.
    // =========================================================================
    const float x_start = 4.20f, y_start = -0.08f, z_start = 2.10f;
    const float x_end = -4.50f, y_end = 0.50f, z_end = 5.20f;

    float curvature_y = aoa * 0.012f;
    float curvature_z = aoa * 0.004f;
    float r_scale = 0.95f + aoa * 0.025f;

    // Tangents: LE sweep = 67.5 deg, core tracks slightly inboard
    const float sweep_ratio = 1.8f;
    const float tx0 = -1.0f, ty0 = 0.04f + curvature_y * 0.25f;
    const float tz0 = sweep_ratio * 0.35f;
    const float tx1 = -1.0f, ty1 = 0.01f + curvature_y * 0.10f;
    const float tz1 = sweep_ratio * 0.15f;

    static constexpr int N = 8;
    static LERX_vortex_spline_point vortex_buffers[2][N];
    LERX_vortex_spline_point* sp = vortex_buffers[idx];

    const float zs = (idx == 0) ? -1.0f : 1.0f;

    // =========================================================================
    // Wing upper-surface clearance envelope
    // =========================================================================
    // Piecewise-linear upper surface height in body-axis Y (up = positive) along
    // the vortex path parameter t=[0,1].  Three stations:
    //   t=0 (apex/inlet junction): y = -0.20 m  (surface sits below body datum)
    //   t=0.5 (mid-chord):         y = -0.06 m
    //   t=1 (trailing edge):       y = +0.12 m  (aft body rises toward elevons)
    auto wing_surface_y = [](float t) -> float
        {
            float y_apex = -0.20f;
            float y_mid = -0.06f;
            float y_aft = 0.12f;
            if (t < 0.5f)
                return y_apex + (y_mid - y_apex) * (t / 0.5f);
            else
                return y_mid + (y_aft - y_mid) * ((t - 0.5f) / 0.5f);
        };

    const float clearance_margin = 0.03f;

    float burst_t = burst_frac;
    if (burst_t > 1.0f) burst_t = 1.0f;

    for (int i = 0; i < N; ++i)
    {
        float t = (float)i / (float)(N - 1);
        float t2 = t * t;
        float t3 = t2 * t;

        // --- Hermite basis ---
        float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
        float h10 = t3 - 2.0f * t2 + t;
        float h01 = -2.0f * t3 + 3.0f * t2;
        float h11 = t3 - t2;

        // --- Hermite basis derivatives (analytical tangent) ---
        float dh00 = 6.0f * t2 - 6.0f * t;
        float dh10 = 3.0f * t2 - 4.0f * t + 1.0f;
        float dh01 = -6.0f * t2 + 6.0f * t;
        float dh11 = 3.0f * t2 - 2.0f * t;

        // --- Position via cubic Hermite ---
        float px = h00 * x_start + h10 * tx0 + h01 * x_end + h11 * tx1;
        float py = h00 * y_start + h10 * ty0 + h01 * y_end + h11 * ty1;
        float pz = h00 * z_start + h10 * tz0 + h01 * z_end + h11 * tz1;

        // AoA-dependent curvature: vertical bow + lateral spread
        py += curvature_y * 4.0f * t * (1.0f - t);
        pz += curvature_z * 4.0f * t * (1.0f - t);

        // --- Analytical tangent (derivative of position w.r.t. t) ---
        float dpx = dh00 * x_start + dh10 * tx0 + dh01 * x_end + dh11 * tx1;
        float dpy = dh00 * y_start + dh10 * ty0 + dh01 * y_end + dh11 * ty1;
        float dpz = dh00 * z_start + dh10 * tz0 + dh01 * z_end + dh11 * tz1;
        dpy += curvature_y * 4.0f * (1.0f - 2.0f * t);
        dpz += curvature_z * 4.0f * (1.0f - 2.0f * t);

        // =================================================================
        // JITTER — two regimes:
        //   Pre-burst:  tight helical precession (coherent core)
        //   Post-burst: chaotic multi-frequency turbulence
        // =================================================================
        // Smooth transition across a ±0.05 window centred on the burst point.
        float blend = (t - (burst_t - 0.05f)) / 0.10f;
        blend = (std::max)(0.0f, (std::min)(1.0f, blend));
        float post_burst = blend;
        float pre_burst  = 1.0f - blend;

        float j_coherent = sinf(time * 6.0f + i * 1.9f) * 0.008f
            + sinf(time * 11.0f + i * 3.1f) * 0.004f;

        float downstream = (std::max)(0.0f, t - burst_t);
        float turb_amp = 0.04f * (1.0f + downstream * 3.0f);
        float j_turbulent = sinf(time * 17.0f + i * 2.3f) * turb_amp
            + sinf(time * 31.0f + i * 5.7f) * turb_amp * 0.5f
            + cosf(time * 23.0f + i * 4.1f) * turb_amp * 0.3f;

        float jitter_y = pre_burst * j_coherent + post_burst * j_turbulent;
        float jitter_z = pre_burst * j_coherent * 0.5f
            + post_burst * j_turbulent * 0.7f;

        // =================================================================
        // RADIUS — two regimes:
        //   Pre-burst:  linear growth (circulation scales with local semispan)
        //   Post-burst: rapid conical expansion (wake-type flow)
        // =================================================================
        float radius_pre = t * 0.80f * r_scale;
        float excess = (std::max)(0.0f, t - burst_t);
        float radius_post = (burst_t * 0.80f * r_scale) + excess * 3.2f * r_scale;

        float desired_radius = pre_burst * radius_pre + post_burst * radius_post;

        // Taper to zero at apex (adjusted for N=8 spacing)
        float apex_taper = (std::min)(1.0f, t * 5.0f);
        desired_radius *= apex_taper;

        // --- Wing surface clearance clamp ---
        float wing_y = wing_surface_y(t);
        float min_core_height = desired_radius + clearance_margin;
        if ((py - wing_y) < min_core_height)
            py = wing_y + min_core_height;

        float clearance = py - wing_y - clearance_margin;
        float max_radius = (std::max)(0.0f, clearance);

        float final_radius = desired_radius;
        if (desired_radius > 0.0f && max_radius < desired_radius)
        {
            float ratio = max_radius / desired_radius;
            final_radius = desired_radius * ratio * (2.0f - ratio);
        }

        // =================================================================
        // OPACITY:
        //   Pre-burst:  builds from zero, peaks at ~65% of burst distance
        //   Post-burst: exponential fade with residual turbulent haze
        // =================================================================
        float opacity;
        if (t < burst_t)
        {
            float peak_t = burst_t * 0.65f;
            float rise, fall;
            if (t < peak_t)
            {
                float s = t / peak_t;
                rise = s * s;
                fall = 1.0f;
            }
            else
            {
                rise = 1.0f;
                float s = (t - peak_t) / (burst_t - peak_t);
                fall = 1.0f - 0.3f * s;
            }
            opacity = 0.09f * rise * fall * vortex_strength * asym;
        }
        else
        {
            float decay = expf(-(t - burst_t) * 6.0f);
            float haze = 0.15f;
            opacity = (0.09f * decay + 0.09f * haze * (1.0f - decay))
                * vortex_strength * asym * 0.5f;
        }

        // =================================================================
        // DENSITY BREAKUP — three noise layers for streaky, wispy, non-uniform vapour 
        //   1. Large-scale patchiness  (whole sections thin/thick)
        //   2. Medium axial streaks
        //   3. Fine-grain shimmer
        // =================================================================
        float noise1 = sinf(t * 4.5f + time * 1.3f)
            * sinf(t * 7.2f - time * 0.9f);
        float noise2 = sinf(t * 13.0f + time * 3.7f + i * 0.8f)
            * cosf(t * 9.1f + time * 2.1f);
        float noise3 = sinf(t * 29.0f + time * 11.0f + i * 2.4f);

        float density_mod = 1.0f
            + 0.35f * noise1
            + 0.20f * noise2
            + 0.10f * noise3;

        // Extra breakup in the turbulent post-burst wake
        density_mod += post_burst * 0.25f * sinf(time * 19.0f + i * 3.3f);

        density_mod = (std::max)(0.05f, (std::min)(1.6f, density_mod));

        // Modulate radius slightly — wisps aren't constant width
        float radius_mod = 1.0f + 0.12f * noise1 + 0.06f * noise2;
        sp[i].radius = final_radius * radius_mod;

        // --- Final position ---
        sp[i].pos[0] = px;
        sp[i].pos[1] = py + jitter_y;
        sp[i].pos[2] = pz * zs + jitter_z * zs;

        // --- Normalised tangent from analytical derivative ---
        float mag = sqrtf(dpx * dpx + dpy * dpy + dpz * dpz);
        if (mag < 1e-6f) mag = 1.0f;
        float inv_mag = 1.0f / mag;

        sp[i].vel[0] = dpx * inv_mag;
        sp[i].vel[1] = dpy * inv_mag;
        sp[i].vel[2] = dpz * inv_mag * zs;

        // --- Composite opacity: base * clamp fade * density breakup * humidity ---
        float clamp_fade = (desired_radius > 0.01f)
            ? (final_radius / desired_radius) : 1.0f;
        sp[i].opacity = opacity * clamp_fade * density_mod * humidity_factor;
    }

    out.spline = sp;
    out.spline_points_count = N;
    out.spline_point_size_in_bytes = sizeof(LERX_vortex_spline_point);
    return true;
}
