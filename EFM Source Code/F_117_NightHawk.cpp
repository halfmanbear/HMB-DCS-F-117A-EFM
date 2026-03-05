#include "stdafx.h"
#include "F117FM.h"
#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include <Math.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <queue>
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
// This variable is very important.  Make sure you set this
// to 0 at the beginning of each frame time or else the moments
// will accumulate.  For each frame time, add up all the moments
// acting on the air vehicle with this variable using th
//
// Units = Newton * meter
//-----------------------------------------------------------------
Vec3	common_moment;							

Vec3	common_force;

Vec3    center_of_gravity;

Vec3	inertia;

Vec3	wind;

Vec3	velocity_world_cs;

//-------------------------------------------------------
// Start of aircraft Simulation Variables
//-------------------------------------------------------
namespace F117 // I tried to convert the imperial units to metric, but it resulted in very bizarre behaviour.
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

	double		rolling_friction		= 0.015;		// Wheel friction amount, I don't know what units. I don't know what this is exactly.
	double		WheelBrakeCommand		= 0.0;			// Commanded wheel brake
	double		GearCommand				= 0.0;			// Commanded gear lever
	//double		airbrake_command		= 0.0;			// Air brakes/spoiler command
	//double		airbrakes				= 0.0;			// Are the air brakes/spoilers deployed?
	double		tailhook_command		= 0.0;			// Tail hook command
	double		tailhook_pos			= 0.0;			// Not sure if needed yet, but whether the hook is down or not
	float		rudder_pos				= 0.0;			//Rudder(s) deflection
	float		misc_cmd				= 0.0;			//Misc actuator command either for tail hooks or weapon bays (F-22, Su-57, etc.)
	float		misc_state				= 0.0;
	float		misc_cmdH				= 0.0;			//Misc actuator command either for tail hooks or weapon bays (F-22, Su-57, etc.)
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
	
	// dragChute test
	double		dragchute_command = 0.0;			// dragChute command
	double		dragchute = 0.0;					// is the dragChute out?

	EDPARAM cockpitAPI;
	param_stuff param_class;
}

static F117::DamageState g_damage;
static std::queue<ed_fm_simulation_event> g_simEvents;

// World-axis state (declared here so ed_fm_simulate can use them for logging)
double ax_world = 0;
double az_world = 0;
double vx_world = 0;
double vy_world = 0;
double vz_world = 0;

// Very important! This function sum up all the forces acting on
// the aircraft for this run frame.  It currently assume the force
// is acting at the center of mass.
void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
{
	common_force.x += Force.x;
	common_force.y += Force.y;
	common_force.z += Force.z;
}

// Very important! This function sums up all the moments acting
// on the aircraft for this run frame.  It currently assumes the
// moment is acting at the center of mass.
void add_local_moment(const Vec3 & Moment)
{
	common_moment.x += Moment.x;
	common_moment.y += Moment.y;
	common_moment.z += Moment.z;
}


// This is where the simulation send the accumulated forces to the DCS Simulation
// after each run frame
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

// This is where the simulation send the accumulated moments to the DCS Simulation
// after each run frame
void ed_fm_add_local_moment(double & x,double &y,double &z)
{						// Figure out what these represent (R,P,Y)
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

//-----------------------------------------------------------------------
// The most important part of the entire EFM code.  This is where you code
// gets called for each run frame.  Each run frame last for a duration of
// "dt" (delta time).  This can be used to help time certain features such
// as filters and lags.
// dt is 6 milliseconds.
//-----------------------------------------------------------------------
void ed_fm_simulate(double dt)
{
	F117::DeltaTime = dt;

	// Init damage debug log on first frame
	static int simFrameCount = 0;
	simFrameCount++;
	initDamageLog();
	if (g_damageLog)
	{
		g_damageLogTimer += dt;
		// Log every frame for first 500 frames (3 seconds), then every 2 seconds
		if (simFrameCount <= 500 || g_damageLogTimer >= 2.0)
		{
			g_damageLogTimer = 0.0;
			double accelMagLog = sqrt(ax_world*ax_world + F117::ay_world*F117::ay_world + az_world*az_world);
			fprintf(g_damageLog, "[SIM] frame=%d invincible=%d | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f | accel=%.1f events=%d WoW=%d\n",
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

	// Very important! clear out the forces and moments before you start calculated
	// a new set for this run frame
	common_force = Vec3();
	common_moment = Vec3();

	// Update derived damage values
	g_damage.wingAsymmetry   = g_damage.leftWing - g_damage.rightWing;
	g_damage.totalWingLoss   = 1.0 - (g_damage.leftWing + g_damage.rightWing) / 2.0;
	g_damage.totalTailLoss   = 1.0 - (g_damage.leftTail + g_damage.rightTail) / 2.0;
	g_damage.engineAsymmetry = g_damage.leftEngine - g_damage.rightEngine;

	// Get the total absolute velocity acting on the aircraft with wind included
	// using imperial units so airspeed is in feet/second here
	Vec3 airspeed;
	airspeed.x = velocity_world_cs.x - wind.x;
	airspeed.y = velocity_world_cs.y - wind.y;
	airspeed.z = velocity_world_cs.z - wind.z;

	F117::totalVelocity_FPS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * F117::meterToFoot;
	if (F117::totalVelocity_FPS < 0.01)
	{
		F117::totalVelocity_FPS = 0.01;
	}

	// Call the atmosphere model to get mach and dynamic pressure
	// This was originally programmed with imperial units, so LB/FT^2 for the pressure.
	// I tried changing the units to the sensible system (metric), but the result was terrible.
	double* temp = F117::temp;
	F117::ATMOS::atmos(F117::ambientTemperature_DegK, F117::ambientDensity_KgPerM3, F117::totalVelocity_FPS, temp);
	F117::dynamicPressure_LBFT2 = temp[0];
	F117::mach = temp[1];

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------
	// 

	// Autopilot - disabled when cockpit integrity drops below 0.7
	if (g_damage.cockpit > 0.7)
	{
		if (alt_hold == 1) // Hold altitude
		{
			pitchTrim = limit((vspeed), -10, 10); // FIX ALTITUDE HOLD!!
		};

		if (altroll_hold == 1) // Keep level and altitude
		{
			pitchTrim = limit((vspeed), -10, 10);
			rollTrim = roll_angle / 10;
		}

		if (horiz_hold == 1) // Keep level to the horizon
		{
			pitchTrim = (pitch_angle * 2);
			rollTrim = roll_angle / 10;
		}
	}
	else
	{
		// Autopilot offline - reset holds
		alt_hold = 0;
		altroll_hold = 0;
		horiz_hold = 0;
	}

	//	Fuel system

	F117::fuel_consumption_since_last_time = ((F117::thrust_N / 36000) * dt);
	F117::internal_fuel -= (F117::fuel_consumption_since_last_time)*F117::param_class.fuelvalue;
	//F117::internal_fuel -= (F117::fuel_consumption_since_last_time) * F117::test_class.testvalue; //"testvalue" is just a temporary thing  I put to make infinite fuel work.

	// Cockpit damage degrades control inputs below 0.7 integrity (FBW system failing)
	double controlDegradation = 1.0;
	if (g_damage.cockpit < 0.7)
		controlDegradation = g_damage.cockpit / 0.7;  // Linear ramp: 0 at 0.0, 1.0 at 0.7

	// Longitudinal (pitch) controller.  Takes the following inputs:
	// -Normalized longitudinal stick input
	// -Trimmed G offset
	// -Angle of attack (deg)
	// -Pitch rate (rad/sec)
	// -Experimental hard input limiter.

	//aoa_filter = ((alpha_DEG * alpha_DEG / 2.5) / 270) + 1; // aoa limit redundant?

	aoa_filter = 1;

	F117::elevator_DEG_commanded = -(F117::FLIGHTCONTROLS::fcs_pitch_controller(F117::FLIGHTCONTROLS::longStickInput * controlDegradation, 0.0, F117::alpha_DEG, F117::pitchRate_RPS * F117::radiansToDegrees, (F117::az / 9.81), 0.0, F117::dynamicPressure_LBFT2, dt, F117::roll_angle, F117::pitch_angle, F117::totalVelocity_FPS, F117::mach, F117::thrust_N, F117::AERO::Cx_total));
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

	roll_filter = (float)(((rollRate_RPS * rollRate_RPS) / 2) / 4 * 5 + 0.5);

	F117::aileron_DEG_commanded = (F117::FLIGHTCONTROLS::fcs_roll_controller(F117::FLIGHTCONTROLS::latStickInput * controlDegradation, F117::FLIGHTCONTROLS::longStickForce, F117::ay / 9.81, F117::rollRate_RPS * F117::radiansToDegrees, 0.0, F117::dynamicPressure_LBFT2, dt));
	F117::aileron_DEG = F117::aileron_DEG_commanded + F117::rollTrim; //F117::ACTUATORS::aileron_actuator(F117::aileron_DEG_commanded,dt);
	F117::aileron_DEG = limit(F117::aileron_DEG, (-15.0 * roll_filter), (15.0 * roll_filter));

	//dragChute
	F117::dragchute = F117::ACTUATORS::dragchute_actuator(F117::dragchute_command, dt, F117::totalVelocity_FPS,
	F117::gearDown, F117::weight_on_wheels);

	aos_filter = (float)(pedInput * (beta_DEG * beta_DEG) / 7500 * (1 + (yawRate_RPS * radiansToDegrees) / 90) + 1);

	F117::rudder_DEG_commanded = F117::FLIGHTCONTROLS::fcs_yaw_controller(F117::pedInput, 0.0, F117::yawRate_RPS * (180.0 / 3.14159), ((F117::rollRate_RPS * F117::radiansToDegrees) / 45),
	F117::FLIGHTCONTROLS::alphaFiltered, F117::aileron_DEG_commanded, F117::ay / 1.56, dt);
	F117::rudder_DEG = F117::rudder_DEG_commanded + F117::yawTrim;
	F117::rudder_DEG = limit(F117::rudder_DEG, -15.0 / aos_filter, 15.0 / aos_filter);

		F117::elev_pos = F117::ACTUATORS::elev_actuator((float)(F117::FLIGHTCONTROLS::longStickInput / aoa_filter + (F117::pitchTrim / 15)), dt);

		// Tail integrity reduces rudder authority (average of both V-tails)
		double tailIntegrity = (g_damage.leftTail + g_damage.rightTail) / 2.0;
		F117::rudder_pos = (float)(F117::ACTUATORS::rudder_actuator((float)F117::pedInput, dt) * tailIntegrity);

		F117::gearDown = F117::ACTUATORS::gear_actuator(F117::GearCommand, dt);

		F117::tailhook_pos = F117::ACTUATORS::tailhook_actuator(F117::tailhook_command, dt);

		F117::misc_state = F117::ACTUATORS::misc_actuator(F117::misc_cmd, dt);

		F117::misc_stateH = F117::ACTUATORS::misc_actuatorH(F117::misc_cmdH, dt); //hook

		// Throttle and thrust - average engine integrity affects throttle response
		double avgEngineIntegrity = (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
		F117::throttle_state = F117::ACTUATORS::throttle_actuator(F117::throttleInput * avgEngineIntegrity, dt);

		// Engine running state - either engine below 0.05 integrity counts as destroyed
		bool engineRunning = F117::engineswitch && (F117::internal_fuel >= 5.0) && (avgEngineIntegrity > 0.05);

		// F404-F1D2 thrust scaled by average engine integrity
		double damagedThrottle = F117::throttleInput * avgEngineIntegrity;
		F117::thrust_N = F117::ENGINE::engine_dynamics(damagedThrottle, F117::mach, F117::altitude_FT, dt, engineRunning, F117::weight_on_wheels);

		// Wing damage: reduce aileron authority and add asymmetric roll bias
		double wingIntegrity = 1.0 - g_damage.totalWingLoss;
		F117::aileron_PCT = (F117::aileron_DEG * wingIntegrity) / 25.5;

		// V-tail damage reduces pitch authority
		F117::elevator_PCT = (F117::elevator_DEG * (1.0 - g_damage.totalTailLoss)) / 25.0;

		// Rudder authority reduced by tail damage, asymmetric tail adds yaw bias
		double tailAsymmetry = g_damage.leftTail - g_damage.rightTail;
		F117::rudder_PCT = (F117::rudder_DEG * tailIntegrity) / 30.0 + tailAsymmetry * 0.1;

		// ---- Engine & Yaw debug logging ----
		{
			static std::ofstream engineDebugLog;
			static int engineLogCounter = 0;
			static bool engineLogInitialized = false;

			if (!engineLogInitialized)
			{
				const char* userProfile = getenv("USERPROFILE");
				std::string desktopPath;
				if (userProfile != nullptr)
				{
					desktopPath = std::string(userProfile) + "\\Desktop\\F117_Engine_Debug.csv";
				}
				else
				{
					desktopPath = "C:\\Users\\Fraser\\Desktop\\F117_Engine_Debug.csv";
				}
				engineDebugLog.open(desktopPath);
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

		// Aerodynamics stuff

		double alpha1_DEG_Limited = limit(F117::alpha_DEG, -20.0, 90.0);
		double beta1_DEG_Limited = limit(F117::beta_DEG, -30.0, 30.0);

		// dragChute aero
		double CDchute = 0.5 * F117::ACTUATORS::dragchute_state; //Drag influence
		double Cxchute = (CDchute * cos(F117::pi / 180.0));

		// Bomb Bay aero
		double CDbay = 0.015 * F117::ACTUATORS::misc_pos; //Drag influence
		double Czbay = -(CDbay * sin(F117::pi / 180.0));
		double Cxbay = (CDbay * cos(F117::pi / 180.0));

		// Gear aero
		double CDGear = 0.027 * F117::gearDown * 1.5;
		double CzGear = -(CDGear * sin(F117::pi / 180.0));
		double CxGear = (CDGear * cos(F117::pi / 180.0));

		//When multiplied, these act a bit like limiters and dampers sometimes.

		F117::AERO::hifi_C(alpha1_DEG_Limited, beta1_DEG_Limited, F117::elevator_DEG, temp);

		// Lift coefficient from tables (used for induced drag calc)
		double Cz = temp[1];
		F117::AERO::Cz = Cz;

		// Drag model: parasitic + induced + wave drag
		// Real F-117 max speed: Mach 0.92 at altitude, cruise Mach 0.8

		// Base parasitic drag (Cd0)
		double Cd0 = 0.0155;

		// Induced drag: Cdi = K * Cz^2, where K = 1/(pi*AR*e)
		double K_induced = 0.18;
		double Cd_induced = K_induced * Cz * Cz;

		// Wave drag (transonic drag rise) - limits top speed to ~Mach 0.92
		double Cd_wave = 0.0;
		double Mcrit = 0.87;  // Critical Mach - wave drag onset
		double Mdiv = 0.90;   // Drag divergence Mach

		if (F117::mach > Mcrit)
		{
			if (F117::mach < 1.05)
			{
				// Transonic drag rise
				double mach_factor = (F117::mach - Mcrit) / (1.05 - Mcrit);
				Cd_wave = 0.05 * mach_factor * mach_factor;

				// Steeper rise at drag divergence (Mach 0.91+)
				if (F117::mach > Mdiv)
				{
					double barrier_factor = (F117::mach - Mdiv) / (1.05 - Mdiv);
					Cd_wave += 0.07 * barrier_factor * barrier_factor;
				}
			}
			else
			{
				// Supersonic - high drag wall
				Cd_wave = 0.12;
			}
		}

		F117::AERO::Cx = Cd0 + Cd_induced + Cd_wave;

		F117::AERO::Cm = temp[2]; // Pitch moment from tables
		F117::AERO::Cy = temp[3]; // I have no idea what this does.
		F117::AERO::Cn = temp[4]; // I think this is yaw momentum??.
		F117::AERO::Cl = temp[5]; // This has something to do with roll.

		F117::AERO::hifi_damping(alpha1_DEG_Limited, temp);
		F117::AERO::Cxq = temp[0]; // This one's weird. It seems to turn angular momentum into speed.
		F117::AERO::Cyr = temp[1]; // I don't know what this does...
		F117::AERO::Cyp = temp[2]; // I think this has something to do with yaw and speed.

		F117::AERO::Czq = temp[3]; // This is related to lift (force up from the dorsal side)
		F117::AERO::Clr = temp[4]; // I think this is roll?
		F117::AERO::Clp = temp[5]; // This also has something to with roll multplying is less movement
		F117::AERO::Cmq = temp[6]; // This is pitch moment damping basically.
		//F117::AERO::Cnr = temp[7] * ((beta_DEG * beta_DEG)/10); // I'm not really sure what this does...
		F117::AERO::Cnr = temp[7]; // I'm not really sure what this does...
		F117::AERO::Cnp = temp[8]; // This seems to translate roll into yaw.

		F117::AERO::hifi_rudder(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
		F117::AERO::Cy_delta_r30 = temp[0];
		F117::AERO::Cn_delta_r30 = temp[1]; // This seems to be the damping effect 
		F117::AERO::Cl_delta_r30 = temp[2] * 1.2; // This seems to translate yaw into roll

		F117::AERO::hifi_ailerons(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
		F117::AERO::Cy_delta_a20 = temp[0];
		F117::AERO::Cn_delta_a20 = temp[2];
		F117::AERO::Cl_delta_a20 = temp[4];

		F117::AERO::hifi_other_coeffs(alpha1_DEG_Limited, F117::elevator_DEG, temp);
		F117::AERO::Cn_delta_beta = temp[0];
		F117::AERO::Cl_delta_beta = temp[1];
		F117::AERO::Cm_delta = temp[2];
		F117::AERO::eta_el = temp[3];
		F117::AERO::Cm_delta_ds = 0;        // ignore deep-stall effect
	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_total
	(as on NASA report p37-40)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* XXXXXXXX Cx_tot XXXXXXXX */

		F117::AERO::dXdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cxq;

		F117::AERO::Cx_total = F117::AERO::Cx + F117::AERO::dXdQ * F117::pitchRate_RPS; // Cx is the x axis (rearward) drag coefficient
		F117::AERO::Cx_total += CxGear + Cxchute + Cxbay; // add x asis drag from gear, chute and bomb bay doors
		/* ZZZZZZZZ Cz_tot ZZZZZZZZ */

		F117::AERO::dZdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Czq;

		F117::AERO::Cz_total = F117::AERO::Cz + F117::AERO::dZdQ * F117::pitchRate_RPS; // Cz is the positive lift coefficient
		F117::AERO::Cz_total += CzGear + Czbay; // add lift modifiers from gear and bomb bay doors
		/* MMMMMMMM Cm_tot MMMMMMMM */

		F117::AERO::dMdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cmq;

		F117::AERO::Cm_total = F117::AERO::Cm * F117::AERO::eta_el + F117::AERO::Cz_total * (F117::referenceCG_PCT - F117::actualCG_PCT) + F117::AERO::dMdQ * F117::pitchRate_RPS + F117::AERO::Cm_delta + F117::AERO::Cm_delta_ds + F117::Cm0;

		/* YYYYYYYY Cy_tot YYYYYYYY */


		F117::AERO::dYdail = F117::AERO::Cy_delta_a20;

		F117::AERO::dYdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cyr;

		F117::AERO::dYdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cyp;

		F117::AERO::Cy_total = F117::AERO::Cy + F117::AERO::dYdail * F117::aileron_PCT + F117::AERO::Cy_delta_r30 * F117::rudder_PCT + F117::AERO::dYdR * F117::yawRate_RPS + F117::AERO::dYdP * F117::rollRate_RPS;

		/* NNNNNNNN Cn_tot NNNNNNNN */

		F117::AERO::dNdail = F117::AERO::Cn_delta_a20;

		F117::AERO::dNdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cnr;

		F117::AERO::dNdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cnp;

		F117::AERO::Cn_total = F117::AERO::Cn - F117::AERO::Cy_total * (F117::referenceCG_PCT - F117::actualCG_PCT) * (F117::meanChord_FT / F117::wingSpan_FT) + F117::AERO::dNdail * F117::aileron_PCT + F117::AERO::Cn_delta_r30 * F117::rudder_PCT + F117::AERO::dNdR * F117::yawRate_RPS + F117::AERO::dNdP * F117::rollRate_RPS + F117::AERO::Cn_delta_beta * F117::beta_DEG;

		/* LLLLLLLL Cl_total LLLLLLLL */

		F117::AERO::dLdail = F117::AERO::Cl_delta_a20;

		F117::AERO::dLdR = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Clr;

		F117::AERO::dLdP = (F117::wingSpan_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Clp;

		F117::AERO::Cl_total = F117::AERO::Cl + F117::AERO::dLdail * F117::aileron_PCT + F117::AERO::Cl_delta_r30 * F117::rudder_PCT + F117::AERO::dLdR * F117::yawRate_RPS + F117::AERO::dLdP * F117::rollRate_RPS + F117::AERO::Cl_delta_beta * F117::beta_DEG;

		//----------------------------------------------------------------
		// All prior forces calculated in lbs, needs to be converted
		// to units.  All prior forces calculated in lb*ft, needs
		// to be converted into N*m
		//----------------------------------------------------------------

		// Cy	(force out the right wing)
		Vec3 cy_force(0.0, 0.0, F117::AERO::Cy_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825);		// Output force in Newtons
		Vec3 cy_force_pos(0.0, 0, 0); //0.01437
		add_local_force(cy_force, cy_force_pos);

		// Cx (drag force - opposes forward motion, so negative X)
		Vec3 cx_force(-F117::AERO::Cx_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825, 0, 0);		// Output force in Newtons
		Vec3 cx_force_pos(0, 0.0, 0.0);
		add_local_force(cx_force, cx_force_pos);

		// Cz (lift force) - reduced by wing damage
		double liftDamageFactor = 1.0 - g_damage.totalWingLoss;
		Vec3 cz_force(0.0, -F117::AERO::Cz_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825 * liftDamageFactor, 0.0);
		Vec3 cz_force_pos(0, 0, 0);
		add_local_force(cz_force, cz_force_pos);

		// Cl (roll moment) - includes asymmetric wing damage roll bias
		double qS_b = F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795;
		double rollMoment = F117::AERO::Cl_total * qS_b;
		// Asymmetric wing damage: more damage on left wing -> roll left (negative moment)
		rollMoment += g_damage.wingAsymmetry * qS_b * 0.05;
		Vec3 cl_moment(rollMoment, 0.0, 0.0);
		add_local_moment(cl_moment);

		// Cm (pitch moment) - V-tail damage reduces pitch effectiveness
		// Wing damage causes nose-down tendency (CG shifts aft of reduced lift center)
		double pitchDamageFactor = 1.0 - g_damage.totalTailLoss * 0.5;
		double noseDownBias = -g_damage.totalWingLoss * qS_b * 0.08;  // pitch down with wing damage
		Vec3 cm_moment(0.0, 0.0, F117::AERO::Cm_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 1.35581795 * F117::meanChord_FT * pitchDamageFactor + noseDownBias);
		add_local_moment(cm_moment);

		// Cn (yaw moment) - includes asymmetric tail damage yaw bias
		double qS_span = F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795;
		double yawMoment = -F117::AERO::Cn_total * qS_span;
		// Asymmetric tail damage adds yaw bias
		double tailAsymDamage = g_damage.leftTail - g_damage.rightTail;
		yawMoment += tailAsymDamage * qS_span * 0.03;
		Vec3 cn_moment(0.0, yawMoment, 0.0);
		add_local_moment(cn_moment);

		// Thrust - F-117 has two F404 engines with slight upward and outward cant
		// Upward cant helps with pitch trim, outward toe matches real aircraft geometry
		double thrust_cant_up_deg = 0.1;    // Degrees upward was 1.0
		double thrust_toe_out_deg = 10;    // Degrees outward per engine
		double cant_up_rad = thrust_cant_up_deg * F117::pi / 180.0;
		double toe_out_rad = thrust_toe_out_deg * F117::pi / 180.0;

		double thrust_per_engine = F117::thrust_N * 0.5;

		// Per-engine thrust scaled by individual engine integrity
		double leftThrust  = thrust_per_engine * g_damage.leftEngine;
		double rightThrust = thrust_per_engine * g_damage.rightEngine;

		// Left engine thrust vector components
		double thrust_x_L = leftThrust * cos(cant_up_rad) * cos(toe_out_rad);
		double thrust_y_L = leftThrust * sin(cant_up_rad);
		double thrust_z_L = leftThrust * cos(cant_up_rad) * sin(toe_out_rad);

		// Right engine thrust vector components
		double thrust_x_R = rightThrust * cos(cant_up_rad) * cos(toe_out_rad);
		double thrust_y_R = rightThrust * sin(cant_up_rad);
		double thrust_z_R = rightThrust * cos(cant_up_rad) * sin(toe_out_rad);

		// Left engine - platypus nozzle spreads exhaust outward (-Z/left)
		// Exhaust goes left, so reaction thrust pushes right (+Z)
		Vec3 thrust_force_L(thrust_x_L, thrust_y_L, thrust_z_L);
		Vec3 thrust_force_pos_L(-4.604, 0.000, -1.427);  // Behind, left of CG (meters)
		add_local_force(thrust_force_L, thrust_force_pos_L);

		// Right engine - platypus nozzle spreads exhaust outward (+Z/right)
		// Exhaust goes right, so reaction thrust pushes left (-Z)
		Vec3 thrust_force_R(thrust_x_R, thrust_y_R, -thrust_z_R);
		Vec3 thrust_force_pos_R(-4.604, 0.000, 1.427);   // Behind, right of CG (meters)
		add_local_force(thrust_force_R, thrust_force_pos_R);

		// Tell the simulation that it has gone through the first frame
		F117::simInitialized = true;
		F117::ACTUATORS::simInitialized = true;
		F117::FLIGHTCONTROLS::simInitialized = true;

		F117::weight_on_wheels = false;
		if ((F117::weight_N > cz_force.y) && (abs(F117::ay_world) >= -0.5) && (F117::ACTUATORS::gear_state == 1.0))
		{
			F117::weight_on_wheels = true;
		}

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

	inertia.x = moment_of_inertia_x; // These don't seem to do anything.
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

	F117::vspeed = vy -3.25; // Vertical speed, with minor adjustments to make autopilot work.
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
	//-------------------------------
	// Start of setting plane states
	//-------------------------------
	F117::alpha_DEG	= (common_angle_of_attack * F117::radiansToDegrees);
	F117::beta_DEG	= (common_angle_of_slide * F117::radiansToDegrees);
	F117::rollRate_RPS = omegax;   // When these values are multiplied, 
	F117::yawRate_RPS = -omegay;  // Higher values mean less movement in that axis.
	F117::pitchRate_RPS = omegaz;  // these act as limiters or dampers.
	
	if (alpha_DEG > 15 && pitchRate_RPS > 0) { F117::pitchRate_RPS = omegaz * (((alpha_DEG * alpha_DEG + 10000) / (180.0 + g_damage.totalWingLoss * 20.0)) - 54.5); } // aoa tuning

	F117::az = ay;
	F117::ay = az;
}

void ed_fm_set_command(int command, float value)	// Command = Command Index (See Export.lua), Value = Signal Value (-1 to 1 for Joystick Axis)
{
	//----------------------------------
	// Set Raw Inputs
	//----------------------------------
	switch (command)
	{
	//Flight contols
		//Roll
	case JoystickRoll:
		F117::FLIGHTCONTROLS::latStickInput = limit(value, -1.0, 1.0);
		F117::roll_cmd = limit(value, -1.0, 1.0);
		break;

	case RollLeft:
		F117::FLIGHTCONTROLS::latStickInput = (-value - 0.025) / 2.0 * 100.0;
		F117::roll_cmd = (-value - 0.025) / 2.0 * 100.0;
		break;

	case RollLeftStop:
		F117::FLIGHTCONTROLS::latStickInput = 0.0;
		break;

	case trimLeft:
		F117::rollTrim += 0.02;
		break;

	case RollRight:
		F117::FLIGHTCONTROLS::latStickInput = (-value + 0.025) / 2.0 * 100.0;
		F117::roll_cmd = (-value + 0.025) / 2.0 * 100.0;
		break;

	case RollRightStop:
		F117::FLIGHTCONTROLS::latStickInput = 0.0;
		break;

	case trimRight:
		F117::rollTrim -= 0.02;
		break;

	case JoystickPitch:
		F117::FLIGHTCONTROLS::longStickInput = limit(-value, -1.0, 1.0);
		break;

	case PitchUp:
			// Keyboard pitch up - full authority (value is 0 for keyboard, so use fixed deflection)
			F117::FLIGHTCONTROLS::longStickInput = -1.0;  // Full pull
		break;
	case PitchUpStop:
		F117::FLIGHTCONTROLS::longStickInput = 0;
		break;
	case trimUp:
		F117::pitchTrim -= 0.05;
		break;

	case PitchDown:
		// Keyboard pitch down - full authority (value is 0 for keyboard, so use fixed deflection)
		F117::FLIGHTCONTROLS::longStickInput = 1.0;  // Full push
		break;

	case PitchDownStop:
		F117::FLIGHTCONTROLS::longStickInput = 0;
		break;
	case trimDown:
		F117::pitchTrim += 0.05;
		break;

		//Yaw
	case JoystickYaw:
		F117::pedInput = limit(-value * (501.0 / (beta_DEG * beta_DEG + 500.0)), -1.0, 1.0);
		break;

	case rudderleft:
		F117::pedInput = -value + (101.0 / (beta_DEG * beta_DEG + 100.0));
		break;

	case rudderleftend:
		F117::pedInput = 0.0;
		break;

	case ruddertrimLeft:
		F117::yawTrim += 0.05;
		break;

	case rudderright:
		F117::pedInput = -value - (101.0 / (beta_DEG * beta_DEG + 100.0));
		break;

	case rudderrightend:
		F117::pedInput = 0.0;
		break;

	case ruddertrimRight:
		F117::yawTrim -= 0.05;
		break;

	//Engine and throttle commands
	case EnginesOff:
		F117::engineswitch = 0;
		F117::throttleInput = 0.0;
		break;

	case EnginesOn:
		F117::engineswitch = 1;
		break;

	case JoystickThrottle:
		if (F117::engineswitch == true)
		{
			// Joystick value: -1..+1 -> Throttle: 0..100
			F117::throttleInput =
				limit(((-value + 1.0) * 0.5) * 100.0, 0.0, 100.0);
		}
		break;

	case ThrottleIncrease:
	{
		if (F117::engineswitch == true && F117::internal_fuel >= 5.0)
		{
			F117::throttleInput += 0.5;
			F117::throttleInput = limit(F117::throttleInput, 0.0, 100.0);
		}
		break;
	}

	case ThrottleDecrease:
	{
		if (F117::engineswitch == true)
		{
			F117::throttleInput -= 0.5;
			F117::throttleInput = limit(F117::throttleInput, 0.0, 100.0);
		}
		break;
	}

	case tailhook:
		if (F117::ACTUATORS::tailhook_state < 0.25)
			F117::tailhook_command = 1.0;
		else if (F117::ACTUATORS::tailhook_state > 0.75)
			F117::tailhook_command = 0.0;
		break;

		//Dragchute
	case dragChute: //toggle
		if (F117::ACTUATORS::dragchute_state < 0.25)
			F117::dragchute_command = 1.0;
		else if (F117::ACTUATORS::dragchute_state > 0.75)
			F117::dragchute_command = 0.0;
		printf("Drag chute = %f \n", dragchute_command);
		break;

		// Gear commands
	case geardown:
		F117::GearCommand = 1.0;
		break; 
	case gearup:
		F117::GearCommand = 0.0;
		break;
	case geartoggle:
		if (F117::ACTUATORS::gear_state > 0.5) F117::GearCommand = 0.0;
		else if (F117::ACTUATORS::gear_state < 0.5) F117::GearCommand = 1.0;
	case WheelBrakeOn:
		F117::rolling_friction = 0.50;
		break;
	case WheelBrakeOff:
		F117::rolling_friction = 0.015;
		break;

		//Other commands

	case bombay:
		if (misc_state < 0.5) F117::misc_cmd = 1.0;
		if (misc_state > 0.5) F117::misc_cmd = 0.0;
		break;

	// Lua damage monitor: applies proportional damage to all components
	// value = damage draw arg delta (0.0-1.0 range, higher = more damage)
	case luaDamageHit:
	{
		if (F117::param_class.invincible_value == 0) break; // immortal, skip

		// Apply a fixed damage dose per hit event
		// Each hit reduces integrity by 0.25 (so ~4 hits to destroy)
		double dmgDose = 0.25;
		g_damage.leftWing    = max(0.0, g_damage.leftWing    - dmgDose * 0.6);
		g_damage.rightWing   = max(0.0, g_damage.rightWing   - dmgDose * 0.6);
		g_damage.leftEngine  = max(0.0, g_damage.leftEngine  - dmgDose * 0.4);
		g_damage.rightEngine = max(0.0, g_damage.rightEngine - dmgDose * 0.4);
		g_damage.leftTail    = max(0.0, g_damage.leftTail    - dmgDose * 0.3);
		g_damage.rightTail   = max(0.0, g_damage.rightTail   - dmgDose * 0.3);
		g_damage.cockpit     = max(0.0, g_damage.cockpit     - dmgDose * 0.2);

		// Fire effects when engines critically damaged (handles match fires_pos)
		if (g_damage.leftEngine < 0.8)  pushFireEvent(8, -4.45, 0.08, -1.7);
		if (g_damage.rightEngine < 0.8) pushFireEvent(7, -4.45, 0.08, 1.7);

		initDamageLog();
		if (g_damageLog)
		{
			fprintf(g_damageLog, "[LUA_DMG] Hit received! dose=%.2f | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
				dmgDose,
				g_damage.leftWing, g_damage.rightWing,
				g_damage.leftEngine, g_damage.rightEngine,
				g_damage.leftTail, g_damage.rightTail,
				g_damage.cockpit);
			fflush(g_damageLog);
		}
		break;
	}

		//Autopilot
	case autopilot_alt:

		horiz_hold = 0;
		altroll_hold = 0;

		if (alt_hold < 0.5) alt_hold = 1;
		else if (alt_hold > 0.5) alt_hold = 0;
		break;

	case autopilot_horiz:

		alt_hold = 0;
		altroll_hold = 0;

		if (horiz_hold < 0.5) horiz_hold = 1;
		else if (horiz_hold > 0.5) horiz_hold = 0;
		break;

	case autopilot_alt_roll:

		alt_hold = 0;
		horiz_hold = 0;

		if (altroll_hold < 0.5) altroll_hold = 1;
		else if (altroll_hold > 0.5) altroll_hold = 0;
		break;

	case autopilot_reset:

		horiz_hold = 0;
		alt_hold = 0;
		altroll_hold = 0;

		break;

	case resetTrim:
		F117::pitchTrim = 0.0;
		F117::rollTrim = 0.0;
		F117::yawTrim = 0.0;
		break;
	};
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

void ed_fm_set_draw_args_v2(float* drawargs, size_t size) //The things that move on the model, use the model viewer to learn what each "arg" corresponds to.
{
	// Draw-arg based damage detection (workaround for limited EDM collision geometry)
	// DCS pre-populates drawargs with visual damage state before calling this function.
	// We read the damage-related args to detect hits that don't trigger ed_fm_on_damage.
	if (F117::param_class.invincible_value != 0 && size > 245) // not immortal, array large enough
	{
		// Read damage draw args from Lua damage table
		// DCS damage args: 0.0 = intact, >0.0 = damaged (visual damage progression)
		float fuseBot    = drawargs[134]; // FUSELAGE_BOTTOM
		float fuseTop    = drawargs[135]; // FUSELAGE_TOP
		float wingLOut   = drawargs[223]; // WING_L_OUT
		float wingROut   = drawargs[213]; // WING_R_OUT
		float flapLOut   = drawargs[240]; // FLAP_L_OUT
		float flapROut   = drawargs[241]; // FLAP_R_OUT
		float flapLCtr   = drawargs[244]; // FLAP_L_ CENTER
		float flapRCtr   = drawargs[245]; // FLAP_R_ CENTER

		// Log every call for first 500 frames, then every 1000 frames
		static int drawArgLogCounter = 0;
		drawArgLogCounter++;
		if (g_damageLog && (drawArgLogCounter <= 500 || drawArgLogCounter % 1000 == 0))
		{
			fprintf(g_damageLog, "[DRAWARGS] frame=%d size=%zu fuseB=%.3f fuseT=%.3f wL=%.3f wR=%.3f fLO=%.3f fRO=%.3f fLC=%.3f fRC=%.3f\n",
				drawArgLogCounter, size, fuseBot, fuseTop, wingLOut, wingROut,
				flapLOut, flapROut, flapLCtr, flapRCtr);
			fflush(g_damageLog);
		}

		// Convert draw arg damage (0=intact, 1=destroyed) to integrity (1=intact, 0=destroyed)
		// and apply to damage state if the draw arg shows damage
		bool anyDamage = (fuseBot > 0.01f || fuseTop > 0.01f ||
		                  wingLOut > 0.01f || wingROut > 0.01f ||
		                  flapLOut > 0.01f || flapROut > 0.01f ||
		                  flapLCtr > 0.01f || flapRCtr > 0.01f);

		if (anyDamage)
		{
			// Fuselage damage -> engines + cockpit
			float fuseWorst = max(fuseBot, fuseTop);
			if (fuseWorst > 0.01f)
			{
				double integrity = 1.0 - (double)fuseWorst;
				g_damage.leftEngine  = min(g_damage.leftEngine,  integrity);
				g_damage.rightEngine = min(g_damage.rightEngine, integrity);
				g_damage.cockpit     = min(g_damage.cockpit,     integrity);
				if (integrity < 0.8) { pushFireEvent(8, -4.45, 0.08, -1.7); pushFireEvent(7, -4.45, 0.08, 1.7); }
			}

			// Left wing damage
			float leftWorst = max(max(wingLOut, flapLOut), flapLCtr);
			if (leftWorst > 0.01f)
			{
				double integrity = 1.0 - (double)leftWorst;
				g_damage.leftWing = min(g_damage.leftWing, integrity);
				if (integrity < 0.8) pushFireEvent(4, -0.82, 0.265, -2.774);
			}

			// Right wing damage
			float rightWorst = max(max(wingROut, flapROut), flapRCtr);
			if (rightWorst > 0.01f)
			{
				double integrity = 1.0 - (double)rightWorst;
				g_damage.rightWing = min(g_damage.rightWing, integrity);
				if (integrity < 0.8) pushFireEvent(3, -0.82, 0.265, 2.774);
			}

			if (g_damageLog)
			{
				fprintf(g_damageLog, "[DRAWARG_DMG] fuseB=%.3f fuseT=%.3f wL=%.3f wR=%.3f fLO=%.3f fRO=%.3f fLC=%.3f fRC=%.3f\n",
					fuseBot, fuseTop, wingLOut, wingROut, flapLOut, flapROut, flapLCtr, flapRCtr);
				fprintf(g_damageLog, "  -> State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
					g_damage.leftWing, g_damage.rightWing,
					g_damage.leftEngine, g_damage.rightEngine,
					g_damage.leftTail, g_damage.rightTail,
					g_damage.cockpit);
				fflush(g_damageLog);
			}
		}
	}

	if (F117::simInitialized)
	{
		F117::ACTUATORS::gear_state = drawargs[0];
		F117::ACTUATORS::gear_state = drawargs[3];
		F117::ACTUATORS::gear_state = drawargs[5];
	}
	else {
		drawargs[0] = (float)F117::ACTUATORS::gear_state;
		drawargs[3] = (float)F117::ACTUATORS::gear_state;
		drawargs[5] = (float)F117::ACTUATORS::gear_state;
	}
	F117::weight_on_wheels = (drawargs[1] + drawargs[4] + drawargs[6]) > 0.5f;

	// Ailerons
	drawargs[11] = (float)limit((-aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75);
	drawargs[12] = (float)limit((aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75);

	// Elevators or stabilators
	drawargs[15] = (float)limit(-elev_pos / (mach + 1), -0.6, 0.6);
	drawargs[16] = (float)limit(-elev_pos / (mach + 1), -0.6, 0.6);

	// Rudder(s)
	drawargs[17] = (float)limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75);
	drawargs[18] = (float)limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75);

	// Air brakes or spoilers
	//drawargs[21] = (float)ACTUATORS::airbrake_state;

	// Drag Chute
	drawargs[35] = (float)ACTUATORS::dragchute_state;

	// Weapon bay doors or tail hook
	drawargs[25] = (float)ACTUATORS::tailhook_state; // This is usually the tail hook for most planes with them.
	drawargs[26] = (float)limit((F117::misc_state), 0.0, 1.0); // This is usually weapon bays for planes with them.

	if (size > 616)
	{
		drawargs[611] = drawargs[0];
		drawargs[614] = drawargs[3];
		drawargs[616] = drawargs[5];
	}

	// Damage animations - binary (0=normal, 1=broken)
	// Staggered thresholds: outer surfaces break first, then inner, then structural
	// Left wing takes 30% extra damage (asymmetric bias) so left breaks before right
	if (size > 248)
	{
		drawargs[224] = (g_damage.leftWing < 0.2) ? 1.0f : 0.0f;    // left wing structure (severe)
		drawargs[247] = (g_damage.leftTail < 0.5) ? 1.0f : 0.0f;    // left rudder
		drawargs[248] = (g_damage.rightTail < 0.5) ? 1.0f : 0.0f;   // right rudder
	}
	if (size > 1015)
	{
		drawargs[1010] = (g_damage.leftWing < 0.7) ? 1.0f : 0.0f;   // left elevon outer (first to break)
		drawargs[1011] = (g_damage.leftWing < 0.4) ? 1.0f : 0.0f;   // left elevon center
		drawargs[1014] = (g_damage.rightWing < 0.3) ? 1.0f : 0.0f;  // right elevon center
		drawargs[1015] = (g_damage.rightWing < 0.6) ? 1.0f : 0.0f;  // right elevon outer (first to break)
	}
}

// Cockpit controls (stick, rudder pedals, throttle) don't animate for some reason.
//void ed_fm_set_fc3_cockpit_draw_args(double* drawargs, size_t size)
void ed_fm_set_fc3_cockpit_draw_args_v2(float* drawargs, size_t size)
{
	drawargs[1001] = (float)limit((F117::FLIGHTCONTROLS::latStickInput), -1.0, 1.0);
	drawargs[1000] = (float)limit((-F117::FLIGHTCONTROLS::longStickInput), -1.0, 1.0);
	drawargs[1002] = (float)limit((F117::throttleInput), -1.0, 1.0);
	drawargs[1003] = (float)limit((F117::pedInput), -1.0, 1.0);
	//drawargs[105] = (float)limit((F117::throttleInput), 0.0, 1.0);

	//F117::FLIGHTCONTROLS::latStickInput = drawargs[1001];
	//F117::FLIGHTCONTROLS::longStickInput = drawargs[1000];
	//F117::throttleInput = drawargs[1002];
	//F117::throttleInput = drawargs[105];

	drawargs[0] = (float)F117::GearCommand;
	drawargs[3] = (float)F117::GearCommand;
	drawargs[5] = (float)F117::GearCommand;
};

void ed_fm_configure(const char * cfg_path)
{
	// I'm not too sure what this does.
}

double ed_fm_get_param(unsigned index)
{	
	// Gear stuff
	switch (index)
	{
	case ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT:
		return 0.0;
	case ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT:
		return F117::rolling_friction;
	case ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT:
		return F117::rolling_friction;
		//break;
	case ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE:
		return 0.0;

	case ED_FM_SUSPENSION_0_WHEEL_YAW:
		return limit(F117::rudder_pos, -0.3, 0.3);

	case ED_FM_ANTI_SKID_ENABLE:
		return true;
	case ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		return F117::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_1_GEAR_POST_STATE:
		return F117::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE:
		return 0.0;
	
	case ED_FM_SUSPENSION_2_GEAR_POST_STATE:
		return F117::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE:
		return 0.0;

	case ED_FM_SUSPENSION_0_DOWN_LOCK:
		return F117::ACTUATORS::gear_state;
		
	case ED_FM_FC3_GEAR_HANDLE_POS:
		return F117::GearCommand;
	}


	if (index <= ED_FM_END_ENGINE_BLOCK)
	{
		// ------------------------------------------------------------
		// Engine availability
		// ------------------------------------------------------------
		bool engineOn = (F117::engineswitch == true) && (F117::internal_fuel > 5.0);

		// ------------------------------------------------------------
		// Core RPM (N2) normalization
		// N2 runs 0-100%, report directly as 0.0-1.0
		// ------------------------------------------------------------
		double normN2 = engineOn
			? F117::ENGINE::N2 / 100.0
			: 0.0;

		normN2 = limit(normN2, 0.0, 1.0);

		// ------------------------------------------------------------
		// Thrust normalization
		// Use known F-117 total max thrust = 80,400 N
		// ------------------------------------------------------------
		const double MAX_THRUST_N = 92000.0;

		double normThrust = engineOn
			? (F117::thrust_N / MAX_THRUST_N)
			: 0.0;

		normThrust = limit(normThrust, 0.0, 1.0);

		switch (index)
		{
			// ============================================================
			// ENGINE 0 (unused / APU)
			// ============================================================
		case ED_FM_ENGINE_0_RPM:
		case ED_FM_ENGINE_0_RELATED_RPM:
		case ED_FM_ENGINE_0_THRUST:
		case ED_FM_ENGINE_0_RELATED_THRUST:
			return 0.0;

			// ============================================================
			// ENGINE 1
			// ============================================================
		case ED_FM_ENGINE_1_RPM:
		case ED_FM_ENGINE_1_RELATED_RPM:
		case ED_FM_ENGINE_1_CORE_RELATED_RPM:
			return normN2;

		case ED_FM_ENGINE_1_THRUST:
			return engineOn ? F117::thrust_N * 0.5 : 0.0;

		case ED_FM_ENGINE_1_RELATED_THRUST:
			return normThrust;

		case ED_FM_ENGINE_1_TEMPERATURE:
		{
			// EGT-style approximation (C)
			double idleEGT = 450.0;
			double maxEGT = 850.0;
			return idleEGT + normN2 * (maxEGT - idleEGT);
		}

		case ED_FM_ENGINE_1_OIL_PRESSURE:
			return engineOn ? (20.0 + normN2 * 40.0) : 0.0;

		case ED_FM_ENGINE_1_FUEL_FLOW:
		{
			// lb/hr equivalent
			double idleFF = 1500.0;
			double maxFF = 6000.0;
			return engineOn
				? idleFF + normN2 * (maxFF - idleFF)
				: 0.0;
		}

		case ED_FM_FC3_THROTTLE_LEFT:
			return limit(F117::throttleInput / 100.0, 0.0, 1.0);

			// ============================================================
			// ENGINE 2 (mirrors engine 1)
			// ============================================================
		case ED_FM_ENGINE_2_RPM:
		case ED_FM_ENGINE_2_RELATED_RPM:
		case ED_FM_ENGINE_2_CORE_RELATED_RPM:
			return normN2;

		case ED_FM_ENGINE_2_THRUST:
			return engineOn ? F117::thrust_N * 0.5 : 0.0;

		case ED_FM_ENGINE_2_RELATED_THRUST:
			return normThrust;

		case ED_FM_ENGINE_2_TEMPERATURE:
		{
			double idleEGT = 450.0;
			double maxEGT = 850.0;
			return idleEGT + normN2 * (maxEGT - idleEGT);
		}

		case ED_FM_ENGINE_2_OIL_PRESSURE:
			return engineOn ? (20.0 + normN2 * 40.0) : 0.0;

		case ED_FM_ENGINE_2_FUEL_FLOW:
		{
			double idleFF = 1500.0;
			double maxFF = 6000.0;
			return engineOn
				? idleFF + normN2 * (maxFF - idleFF)
				: 0.0;
		}

		case ED_FM_FC3_THROTTLE_RIGHT:
			return limit(F117::throttleInput / 100.0, 0.0, 1.0);
		}
	}

	// Other params (fuel, controls, autopilot)
	switch (index)
	{
	case ED_FM_FUEL_INTERNAL_FUEL:
		return (F117::internal_fuel)+(F117::external_fuel);
	case ED_FM_FUEL_TOTAL_FUEL:
		return (F117::internal_fuel) + (F117::external_fuel);

	case ED_FM_OXYGEN_SUPPLY:
		return 1000;
	case ED_FM_FLOW_VELOCITY:
		return 100;
	//case ED_FM_FC3_SPEED_BRAKE_HANDLE_POS:
	//	return F117::airbrake_command*100;
	case ED_FM_FC3_STICK_PITCH:
		return F117::FLIGHTCONTROLS::longStickInput;
	case ED_FM_FC3_STICK_ROLL:
		return F117::FLIGHTCONTROLS::latStickInput;
	case ED_FM_FC3_RUDDER_PEDALS:
		return F117::pedInput;

	// Autopilot
	case ED_FM_FC3_AUTOPILOT_STATUS:
		return F117::alt_hold;
	//case ED_FM_FC3_AUTOPILOT_FAILURE_ATTITUDE_STABILIZATION:
	}

	return 0;
}

// This defines what is reset when the plane is destroyed or the player restarts or quits the mission.
void ed_fm_release()
{
	F117::DeltaTime = 0;
	F117::simInitialized = false;
	F117::ACTUATORS::simInitialized = false;
	F117::FLIGHTCONTROLS::simInitialized = false;
	F117::ENGINE::N2 = 0.0;

	// Reset damage state and clear pending events
	g_damage = F117::DamageState();
	while (!g_simEvents.empty()) g_simEvents.pop();

	F117::pedInput = 0;
	F117::throttleInput = 0.0;
	F117::elevator_DEG = 0;
	F117::aileron_DEG = 0;
	F117::rudder_DEG = 0;
	F117::elevator_DEG_commanded = 0;
	F117::rudder_DEG_commanded = 0;
	F117::throttle_state = 0;
	F117::rolling_friction = 0.015;
	F117::WheelBrakeCommand = 0.0;
	F117::pitchTrim = 0.0;
	F117::rollTrim = 0.0;
	F117::yawTrim = 0.0;
	//F117::airbrake_command = 0.0;
	//F117::airbrakes = 0.0;
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
	//F117::ACTUATORS::flapPosition_DEG = 0.0;
	//F117::ACTUATORS::flapRate_DEGPERSEC = 0.0;
	F117::ACTUATORS::throttle_state = 0.0;
	F117::ACTUATORS::throttle_rate = 0.0;
	F117::ACTUATORS::misc_pos = 0.0;
	//F117::ENGINE::percentPower = 0.0;
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

// What parameters should change when easy flight mode is on/off?
void ed_fm_set_easy_flight(bool value) 
{} // Nothing here yet. Flight behaviour for this FM mimics "game" flight mode.

// This defines what happens when the unlimited fuel option is on or off.
void ed_fm_unlimited_fuel(bool value) 
{
		F117::param_class.param_stuff::fuelparam(1-value);
}

// What parameters should be set to what in a cold start?
void ed_fm_cold_start()
{
	F117::gearDown = 1;
	F117::GearCommand = 1;
	F117::throttleInput = 0;
	F117::WheelBrakeCommand = 0.0;
	F117::engineswitch = false;
	F117::rolling_friction = 0.015;
	F117::ENGINE::N2 = 0.0;
	g_damage = F117::DamageState();
	while (!g_simEvents.empty()) g_simEvents.pop();
	initDamageLog();
	if (g_damageLog) { fprintf(g_damageLog, "\n*** ed_fm_cold_start called ***\n"); fflush(g_damageLog); }
}

// What parameters should be set to what in a hot start on the ground?
void ed_fm_hot_start()
{
	// Aircraft state
	F117::gearDown = 1;
	F117::GearCommand = 1;
	F117::WheelBrakeCommand = 0.0;
	// Pilot throttle at idle stop
	F117::throttleInput = 0.0;
	// Engines already running at ground idle
	F117::engineswitch = true;
	F117::ENGINE::N2 = 60.0;
	F117::rolling_friction = 0.015;
	g_damage = F117::DamageState();
	while (!g_simEvents.empty()) g_simEvents.pop();
	initDamageLog();
	if (g_damageLog) { fprintf(g_damageLog, "\n*** ed_fm_hot_start called ***\n"); fflush(g_damageLog); }
}

// What parameters should be set to what in a hot start in the air?
void ed_fm_hot_start_in_air()
{
	F117::gearDown = 0;
	F117::GearCommand = 0;
	F117::throttleInput = 77.5;
	F117::throttle_state = 77.5;
	F117::WheelBrakeCommand = 0.0;
	F117::engineswitch = true;
	F117::rolling_friction = 0.015;
	F117::ENGINE::N2 = 92.0;
	g_damage = F117::DamageState();
	while (!g_simEvents.empty()) g_simEvents.pop();
	initDamageLog();
	if (g_damageLog) { fprintf(g_damageLog, "\n*** ed_fm_hot_start_in_air called ***\n"); fflush(g_damageLog); }
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

static void initDamageLog()
{
	if (g_damageLog) return;

	// Try multiple paths in case of permission issues
	const char* paths[] = {
		"E:\\Saved Games\\DCS\\Logs\\F117_Damage_Debug.log",
		"E:\\Saved Games\\DCS\\F117_Damage_Debug.log",
		"C:\\Users\\Fraser\\Desktop\\F117_Damage_Debug.log",
		"C:\\F117_Damage_Debug.log",
		nullptr
	};

	for (int i = 0; paths[i] != nullptr; i++)
	{
		g_damageLog = fopen(paths[i], "w");
		if (g_damageLog)
		{
			fprintf(g_damageLog, "=== F-117 Damage Debug Log ===\n");
			fprintf(g_damageLog, "Log opened at: %s\n", paths[i]);
			fprintf(g_damageLog, "invincible_value at init: %d\n\n", (int)F117::param_class.invincible_value);
			fflush(g_damageLog);
			return;
		}
	}
}

// Damage callback - DCS sends remaining integrity (1.0 = intact, 0.0 = destroyed)
void ed_fm_on_damage(int Element, double element_integrity_factor)
{
	initDamageLog();
	if (g_damageLog)
	{
		fprintf(g_damageLog, "ed_fm_on_damage called: Element=%d, integrity=%.4f, invincible=%d\n",
			Element, element_integrity_factor, (int)F117::param_class.invincible_value);
		fflush(g_damageLog);
	}

	// Skip damage when immortal (invincible_value == 0 means immortal)
	if (F117::param_class.invincible_value == 0)
	{
		if (g_damageLog) { fprintf(g_damageLog, "  -> SKIPPED (immortal)\n"); fflush(g_damageLog); }
		return;
	}

	// Amplify damage: DCS sends conservative integrity values, scale them up
	// Factor of 5.0: 1 rocket -> ~0.17 integrity (fire+uncontrollable), 2 rockets -> destroyed
	const double DMG_SCALE = 5.0;
	double rawDamage = 1.0 - limit(element_integrity_factor, 0.0, 1.0);
	double integrity = max(0.0, 1.0 - rawDamage * DMG_SCALE);
	const char* mapped = "UNHANDLED";

	switch (Element)
	{
	// ---- Cockpit (Damage table: [3] critical_damage=100 to keep pilot alive) ----
	// Ignored here - cockpit damage is derived from MAIN (case 11) instead
	// High critical_damage in Lua prevents DCS from killing the pilot / blacking out
	case 3:
		mapped = "cockpit(ignored, derived from MAIN)";
		break;

	// ---- Engine (Damage table: [10] ENGINE001) ----
	// Single engine cell covers both F404 engines
	// fires_pos handles (0-indexed into Lua fires_pos table):
	//  0={-0.865,1.01,1}      top       4={-0.82,0.265,-2.774}  left mid-wing
	//  1={-0.37,-0.23,3.01}   right wing 5={-0.82,0.255,4.274}  right outer wing
	//  2={-0.37,-0.23,-3.01}  left wing  6={-0.82,0.255,-4.274} left outer wing
	//  3={-0.82,0.265,2.774}  right mid  7={-4.45,0.08,1.7}     right engine
	//  8={-4.45,0.08,-1.7}    left engine 9={2,-0.56,-1}         nose
	// 10={-4.08,0.22,0}       rear center
	case 10:
		g_damage.leftEngine = integrity;
		g_damage.rightEngine = integrity;
		if (integrity < 0.8) { pushFireEvent(8, -4.45, 0.08, -1.7); pushFireEvent(7, -4.45, 0.08, 1.7); }
		mapped = "engines(both)";
		break;

	// ---- Main structure (Damage table: [11] MAIN) ----
	// Overall airframe: wings, tails, fuselage, and cockpit systems (FBW/controls)
	// Cockpit derived from MAIN so [3] has high critical_damage (prevents DCS pilot kill)
	// Asymmetric bias: left wing takes 30% more damage
	case 11:
		g_damage.leftWing  = min(g_damage.leftWing,  max(0.0, integrity - (1.0 - integrity) * 0.3));
		g_damage.rightWing = min(g_damage.rightWing, integrity);
		g_damage.leftTail  = min(g_damage.leftTail,  integrity);
		g_damage.rightTail = min(g_damage.rightTail, integrity);
		g_damage.cockpit   = min(g_damage.cockpit,   integrity);
		if (integrity < 0.8) { pushFireEvent(4, -0.82, 0.265, -2.774); pushFireEvent(3, -0.82, 0.265, 2.774); }
		mapped = "main(wings+tails+cockpit)";
		break;

	default:
		// Ignore unhandled elements (82, 99, 101, 102 = ground contact, etc.)
		// Only elements 3, 10, 11 from our Damage table should apply damage
		mapped = "IGNORED(unhandled)";
		break;
	}

	if (g_damageLog)
	{
		fprintf(g_damageLog, "  -> mapped to: %s | State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
			mapped,
			g_damage.leftWing, g_damage.rightWing,
			g_damage.leftEngine, g_damage.rightEngine,
			g_damage.leftTail, g_damage.rightTail,
			g_damage.cockpit);
		fflush(g_damageLog);
	}
}

// What should be fixed when repairs are complete?
void ed_fm_repair()
{
	g_damage = F117::DamageState();
	while (!g_simEvents.empty()) g_simEvents.pop();
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
