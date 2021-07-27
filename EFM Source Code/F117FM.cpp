#include "stdafx.h"
#include "F117FM.h"
#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include <Math.h>
#include <stdio.h>
#include <string>
#include <math.h>
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
	double		wingSpan_FT				= 43.4;		// F-16 wing-span (ft)
	double		wingArea_FT2			= 780.0;		// F-16 wing area (ft^2)
	double		meanChord_FT			= 25.0;		// F-16 mean aerodynamic chord (ft)
	double		referenceCG_PCT			= 0.35;			// Reference center of mass as a % of wing chord
	double		actualCG_PCT			= 0.30;			// Actual center of mass as a % of wing chord
	double		pi						= acos(-1.0);	// Pi (3.14159....)
	double		radiansToDegrees		= 180.0/pi;		// Conversion factor from radians to degrees
	double		inertia_Ix_KGM2			= 12874.0;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iy_KGM2			= 75673.6;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iz_KGM2			= 85552.1;		// Reference moment of inertia (kg/m^2)
	double		temp[9];								// Temporary array for holding look-up table results
	double		altitude_m				= 0.0;			// Absolute altitude above sea level (metres)
	double		altitude_FT				= 45000;			// Absolute altitude above sea level (ft)
	double		totalVelocity_FPS		= 0.0;			// Total velocity (always positive) (ft/s)
	double		alpha_DEG				= 0.0;			// Angle of attack (deg)
	double		beta_DEG				= 0.0;			// Slideslip angle (deg)
	double		rollRate_RPS			= 0.0;			// Body roll rate (rad/sec)
	double		pitchRate_RPS			= 0.0;			// Body pitch rate (rad/sec)
	double		yawRate_RPS				= 0.0;			// Body yaw rate (rad/sec)
	double		thrust_N				= 80423;			// Engine thrust (Newtons)

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
	double		throttle_state			= 0.2;			// Engine power state
	double		pedInput				= 0.0;			// Rudder pedal input command normalized (-1 to 1)
	double		throttleInput			= 0.2;			// Throttle input command normalized (-1 to 1)
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
	double		weight_N				= 131222.538;			// Weight force of aircraft (N)
	double		ay_world				= 0.0;			// World referenced up/down acceleration (m/s^2)
	double		weight_on_wheels		= 0.0;			// Weight on wheels flag (not used right now)

	double		rolling_friction		= 0.015;			// Wheel friction amount, I don't know what units. I don't know what this is exactly.
	double		WheelBrakeCommand		= 0.0;			// Commanded wheel brake
	double		GearCommand				= 0.0;			// Commanded gear lever
	double		airbrake_command		= 0.0;			// Air brakes/spoiler command
	double		airbrakes				= 0.0;			// Are the air brakes/spoilers deployed?
	double		tailhook_CMD			= 0.0;			// Tail hook command
	double		hook					= 0.0;			// Not sure if needed yet, but whether the hook is down or not
	float		rudder_pos				= 0.0;			//Rudder(s) deflection
	float		misc_cmd				= 0.0;			//Misc actuator command either for tail hooks or weapon bays (F-22, Su-57, etc.)
	float		misc_state				= 0.0;
	float		misc_cmdH				= 0.0;			//Misc actuator command either for tail hooks or weapon bays (F-22, Su-57, etc.)
	float		misc_stateH				= 0.0;

	double engine_damage = 0;			// Combined left and right engine damage
	double Lwing_damage = 0;			// Left wing damage
	double Rwing_damage = 0;			// Right wing damage
	double wing_damage = (Rwing_damage + Lwing_damage); // Combined wing damage
	double tail_damage = 0;				// Tail(s) and rudder(s) damage
	double cockpit_damage = 0;			// Cockpit avionics damage	

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
	
	// Dragshute test
	double		dragshute_command = 0.0;			// Dragshute command
	double		dragshute = 0.0;					// is the Dragshute out?

	EDPARAM cockpitAPI;
	param_stuff param_class;
}

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

//Fuel consumption
void simulate_fuel_consumption(double dt) // This doesn't seem to work. 

{
	F117::fuel_consumption_since_last_time =  10 * (F117::throttleInput * 1000) * engine_damage * dt; //10 kg persecond
		if (F117::fuel_consumption_since_last_time > internal_fuel)
			F117::fuel_consumption_since_last_time = internal_fuel;
		internal_fuel -= F117::fuel_consumption_since_last_time +10 +(10* throttleInput);
} // This doesn't seem to work. 

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


	/*
	if(F117::weight_on_wheels)
	{
		F117::alpha_DEG = 0.0;
		F117::az = 0.0;
	}
	*/

	// Very important! clear out the forces and moments before you start calculated
	// a new set for this run frame
	common_force = Vec3();
	common_moment = Vec3();

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
	double* temp;
	temp = (double*)malloc(9 * sizeof(double));
	F117::ATMOS::atmos(F117::ambientTemperature_DegK, F117::ambientDensity_KgPerM3, F117::totalVelocity_FPS, temp);
	F117::dynamicPressure_LBFT2 = temp[0];
	F117::mach = temp[1];

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------
	// 

	// Autopilot
	if (cockpit_damage < 1)
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
	if (cockpit_damage > 1)
	{
		F117::engine_damage = 0;
		F117::Lwing_damage = 0;
		F117::Rwing_damage = 0;
		F117::tail_damage = 0;
	}

	//	Fuel system

	F117::fuel_consumption_since_last_time = ((F117::thrust_N / 20000) * dt);
	F117::internal_fuel -= (F117::fuel_consumption_since_last_time)*F117::param_class.fuelvalue;
	//F117::internal_fuel -= (F117::fuel_consumption_since_last_time) * F117::test_class.testvalue; //"testvalue" is just a temporary thing  I put to make infinite fuel work.

	// Longitudinal (pitch) controller.  Takes the following inputs:
	// -Normalized longitudinal stick input
	// -Trimmed G offset
	// -Angle of attack (deg)
	// -Pitch rate (rad/sec)
	// -Experimental hard input limiter.

	aoa_filter = ((alpha_DEG * alpha_DEG / 2.5) / 270) + 1;

	F117::elevator_DEG_commanded = -(F117::FLIGHTCONTROLS::fcs_pitch_controller(F117::FLIGHTCONTROLS::longStickInput, -1.0, F117::alpha_DEG, F117::pitchRate_RPS * F117::radiansToDegrees, (F117::az / 9.81), 0.0, F117::dynamicPressure_LBFT2, dt));
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

	//roll_filter = (((rollRate_RPS * rollRate_RPS) / 2) / 4 * 5 + 0.5);
	roll_filter = ((rollRate_RPS * rollRate_RPS) *100)+0.1;

	F117::aileron_DEG_commanded = (F117::FLIGHTCONTROLS::fcs_roll_controller(F117::FLIGHTCONTROLS::latStickInput, F117::FLIGHTCONTROLS::longStickForce, F117::ay / 9.81, F117::rollRate_RPS * F117::radiansToDegrees, 0.0, F117::dynamicPressure_LBFT2, dt));
	F117::aileron_DEG = F117::aileron_DEG_commanded + F117::rollTrim; //F117::ACTUATORS::aileron_actuator(F117::aileron_DEG_commanded,dt);
	F117::aileron_DEG = limit(F117::aileron_DEG, (-15.0 * roll_filter), (15.0 * roll_filter));

	//Dragshute
	F117::dragshute = F117::ACTUATORS::dragshute_actuator(F117::dragshute_command, dt);

	aos_filter = pedInput * (beta_DEG * beta_DEG) / 7500 * (1 + (yawRate_RPS * radiansToDegrees) / 90) + 1;

	F117::rudder_DEG_commanded = F117::FLIGHTCONTROLS::fcs_yaw_controller(F117::pedInput, 0.0, F117::yawRate_RPS * (180.0 / 3.14159), ((F117::rollRate_RPS * F117::radiansToDegrees) / 45),
	F117::FLIGHTCONTROLS::alphaFiltered, F117::aileron_DEG_commanded, F117::ay / 1.56, dt);
	F117::rudder_DEG = F117::rudder_DEG_commanded + F117::yawTrim;
	F117::rudder_DEG = limit(F117::rudder_DEG, -15.0 / aos_filter, 15.0 / aos_filter);

		F117::elev_pos = F117::ACTUATORS::elev_actuator(F117::FLIGHTCONTROLS::longStickInput / aoa_filter + (F117::pitchTrim / 15), dt);

		F117::rudder_pos = F117::ACTUATORS::rudder_actuator(F117::pedInput, dt) / (1+tail_damage);

		F117::gearDown = F117::ACTUATORS::gear_actuator(F117::GearCommand, dt);

		F117::airbrakes = F117::ACTUATORS::airbrake_actuator(F117::airbrake_command, dt);

		F117::hook = F117::ACTUATORS::tailhook_actuator(F117::tailhook_CMD, dt);

		F117::misc_state = F117::ACTUATORS::misc_actuator(F117::misc_cmd, dt);

		F117::misc_stateH = F117::ACTUATORS::misc_actuatorH(F117::misc_cmdH, dt); //hook

		//Throttle and thrust
		F117::throttle_state = F117::ACTUATORS::throttle_actuator(F117::throttleInput / (1 + engine_damage), dt);

		if (F117::throttleInput <= 54.9) //This is to make sure the plane holds still while at idle on the ground.
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics((F117::throttleInput - 25), F117::mach, F117::altitude_FT, dt) / 1.2;
		}

		if (F117::throttle_state > 55.0 && F117::throttle_state < 74.9)
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics((F117::throttleInput - 25) - 25, F117::mach, F117::altitude_FT, dt);
		}

		if (F117::throttle_state > 75.0 && F117::throttle_state < 89.9)
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics((F117::throttleInput - 25), F117::mach, F117::altitude_FT, dt) * 1.15;
		}

		if (F117::throttle_state >= 90.0 && F117::gearDown >= 0.25) // I can't think of a better way of getting ground acceleration more realistic.
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics(F117::throttleInput, F117::mach, F117::altitude_FT, dt) / 1.2;
		}

		if (F117::throttle_state >= 90.0 && F117::gearDown <= 0.26) //simulating the increased thrust from afterburners.
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics(F117::throttleInput, F117::mach, F117::altitude_FT, dt) * 1.25;
		}
		if (F117::throttle_state >= 95.0 && F117::gearDown <= 0.26) //simulating the increased thrust from afterburners.
		{
			F117::thrust_N = F117::ENGINE::engine_dynamics(F117::throttleInput, F117::mach, F117::altitude_FT, dt) * 1.5;
		}
		if (F117::internal_fuel < 5.0)
		{
			F117::thrust_N = 0;
		}
		if (F117::engine_damage > 5.0)
		{
			F117::thrust_N = 0;
		}

		F117::aileron_PCT = (F117::aileron_DEG + ((Lwing_damage * 18) + (Rwing_damage * 20))) / (25.5 + wing_damage);
		F117::elevator_PCT = F117::elevator_DEG / 25.0;
		F117::rudder_PCT = (F117::rudder_DEG - (tail_damage)) / (30.0 + (tail_damage*10));

		// Aerodynamics stuff

		double alpha1_DEG_Limited = limit(F117::alpha_DEG, -20.0, 90.0);
		double beta1_DEG_Limited = limit(F117::beta_DEG, -30.0, 30.0);

		// Air brakes aero 
		double CDbrakes = 0.05 * F117::ACTUATORS::airbrake_state;
		double Cxbrakes = -(CDbrakes * cos(F117::pi / 180.0));

		// Drag Shute aero
		double CDshute = 1.0 * F117::ACTUATORS::dragshute_state; //Drag influence
		double Cxshute = -(CDshute * cos(F117::pi / 180.0));

		// Gear aero
		//double CDGear = 0.027 * F117::gearDown * 1.125;
		double CDGear = 0.027 * F117::gearDown * 1.5;
		double CzGear = -(CDGear * sin(F117::pi / 180.0));
		double CxGear = -(CDGear * cos(F117::pi / 180.0));
		if (mach < 0.5)
		{
			CxGear *= 2;
		};
		//When multiplied, these act a bit like limiters and dampers sometimes.

		F117::AERO::hifi_C(alpha1_DEG_Limited, beta1_DEG_Limited, F117::elevator_DEG, temp);
		F117::AERO::Cx = temp[0] * 1+(wing_damage); // I think this is drag when AOA is low.
		F117::AERO::Cz = temp[1]; // This is related to pitch.
		F117::AERO::Cm = temp[2]; // This is related to lift, I think.
		F117::AERO::Cy = temp[3]; // I have no idea what this does.
		F117::AERO::Cn = temp[4]; // I think this is yaw momentum??.
		F117::AERO::Cl = temp[5]; // This has something to do with roll.

		F117::AERO::hifi_damping(alpha1_DEG_Limited, temp);
		F117::AERO::Cxq = temp[0]; // This one's weird. It seems to turn angular momentum into speed.
		F117::AERO::Cyr = temp[1]; // I don't know what this does...
		F117::AERO::Cyp = temp[2]; // I think this has something to do with yaw and speed.
		if (alpha_DEG > 0.01 || Lwing_damage < 2 || Rwing_damage < 2)
		{					// This might be a "brute force" approach to stability, but I couldn't get anything else to work.
			F117::AERO::Czq = temp[3] * limit(((((alpha_DEG * alpha_DEG) / 12) + 1) * mach), 1, 10);
		}
		else
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
	// Cx is drag
		F117::AERO::dXdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cxq;

		F117::AERO::Cx_total = F117::AERO::Cx + F117::AERO::dXdQ * F117::pitchRate_RPS;
		F117::AERO::Cx_total += CxGear + Cxbrakes;

		/* ZZZZZZZZ Cz_tot ZZZZZZZZ */
		F117::AERO::dZdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Czq;

		F117::AERO::Cz_total = F117::AERO::Cz * F117::AERO::dZdQ * F117::pitchRate_RPS;
		F117::AERO::Cz_total += CzGear;

		/* MMMMMMMM Cm_tot MMMMMMMM */
		F117::AERO::dMdQ = (F117::meanChord_FT / (2 * F117::totalVelocity_FPS)) * F117::AERO::Cmq;

		F117::AERO::Cm_total = F117::AERO::Cm * F117::AERO::eta_el + F117::AERO::Cz_total * (F117::referenceCG_PCT - F117::actualCG_PCT) * F117::AERO::dMdQ * F117::pitchRate_RPS + F117::AERO::Cm_delta + F117::AERO::Cm_delta_ds;

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
		// Cl is lift
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

		// Cx (force out the nose)
		Vec3 cx_force(F117::AERO::Cx_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825, 0, 0);		// Output force in Newtons
		Vec3 cx_force_pos(0, 0.0, 0.0);
		add_local_force(cx_force, cx_force_pos);

		// Cz (force down the bottom of the aircraft)
		Vec3 cz_force(0.0, -F117::AERO::Cz_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 4.44822162825, 0.0);	// Output force in Newtons
		Vec3 cz_force_pos(0, 0, 0);
		add_local_force(cz_force, cz_force_pos);

		// Cl	(Output force in N/m)
		Vec3 cl_moment(F117::AERO::Cl_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795, 0.0, 0.0);
		add_local_moment(cl_moment);

		// Cm	(Output force in N/m)
		Vec3 cm_moment(0.0, 0.0, F117::AERO::Cm_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * 1.35581795 * F117::meanChord_FT);
		add_local_moment(cm_moment);

		// Cn	(Output force in N/m)
		Vec3 cn_moment(0.0, -F117::AERO::Cn_total * F117::wingArea_FT2 * F117::dynamicPressure_LBFT2 * F117::wingSpan_FT * 1.35581795, 0.0);
		add_local_moment(cn_moment);

		// Thrust	
		Vec3 thrust_force(F117::thrust_N, 0.0, 0.0);	// Output force in Newtons
		Vec3 thrust_force_pos(0, 0, 0);
		add_local_force(thrust_force, thrust_force_pos);

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
double ax_world = 0;
double ay_world = 0;
double az_world = 0;
double vx_world = 0;
double vy_world = 0;
double vz_world = 0;

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

	pitch_angle = (pitch * radiansToDegrees)-1;
	roll_angle = (roll * radiansToDegrees) - 1;
	//-------------------------------
	// Start of setting plane states
	//-------------------------------
	F117::alpha_DEG	= (common_angle_of_attack * F117::radiansToDegrees);
	F117::beta_DEG	= (common_angle_of_slide * F117::radiansToDegrees);
	F117::rollRate_RPS = omegax;   // When these values are multiplied, 
	F117::yawRate_RPS = -omegay;  // Higher values mean less movement in that axis.
	F117::pitchRate_RPS = omegaz;  // these act as limiters or dampers.
	
	if (alpha_DEG > 5 && pitchRate_RPS > 0 ) { F117::pitchRate_RPS = omegaz * (((alpha_DEG * alpha_DEG + 10000) / (180 + wing_damage)) - 54.5); }
	
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
			F117::FLIGHTCONTROLS::longStickInput = -value - (((alpha_DEG) / (alpha_DEG * alpha_DEG + 5 * 5) / alpha_DEG) * 120);
		break;
	case PitchUpStop:
		F117::FLIGHTCONTROLS::longStickInput = 0;
		break;
	case trimUp:
		F117::pitchTrim -= 0.05;
		break;

	case PitchDown:
		F117::FLIGHTCONTROLS::longStickInput = -value + ((alpha_DEG) / (alpha_DEG * alpha_DEG + 5 * 10) / alpha_DEG) *1250;
		break;

	case PitchDownStop:
		F117::FLIGHTCONTROLS::longStickInput = 0;
		break;
	case trimDown:
		F117::pitchTrim += 0.05;
		break;

		//Yaw
	case JoystickYaw:
		F117::pedInput = limit(-value * (((beta_DEG) / (beta_DEG * beta_DEG + 100 * 5) / beta_DEG) * 501), -1.0, 1.0);
		break;

	case rudderleft:
		F117::pedInput = -value + (((beta_DEG) / (beta_DEG * beta_DEG + 5 * 20) / beta_DEG) * 101);
		break;

	case rudderleftend:
		F117::pedInput = 0.0;
		break;

	case ruddertrimLeft:
		F117::yawTrim += 0.05;
		break;

	case rudderright:
		F117::pedInput = -value - (((beta_DEG) / (beta_DEG * beta_DEG + 5 * 20) / beta_DEG) * 101);
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
		F117::throttleInput = -100 + limit((value), 0.0, 0.0);
		break;

	case EnginesOn:
		F117::engineswitch = 1;
		F117::throttleInput += 50;
		F117::throttleInput = (limit((value), 0.0, 100.0));
		break;

	case JoystickThrottle:
		if (F117::engineswitch = true)
		{
			F117::throttleInput = limit(((-value + 1.0) / 2.0) * 100.0, 25.0, 100.0);
		}
		break;

	case ThrottleIncrease:
		if (F117::engineswitch == 1)
		{
			if (F117::internal_fuel >= 5.0)
			{
				if (F117::throttleInput < 100)
				{
					F117::throttleInput += 0.40;
				}
				if (F117::throttleInput <= 24.9)
				{
					F117::throttleInput = 25.0;
				}
			}
			if (F117::internal_fuel < 5.0)
			{
				F117::throttleInput -= 100.0;
				F117::throttleInput = 0.0;
				F117::throttleInput = 0 + limit((value), 0.0, 0.01);
			}
		}
		else
		{
			F117::throttleInput = 0;
		}
		break;

	case ThrottleDecrease:
		if (F117::engineswitch == 1)
		{
			if (F117::internal_fuel >= 5.0)
			{
				if (F117::throttleInput <= 24.9)
				{
					F117::throttleInput += 0.01;
				}
				if (F117::throttleInput > 25.0)
				{
					F117::throttleInput -= 0.50;
				}
			}
			else
			{
				F117::throttleInput -= 100.0;
				F117::throttleInput = 0.0;
				F117::throttleInput = 0 + limit((value), 0.0, 0.01);
			}
		}
		else
		{
			F117::throttleInput = 0;
		}
		break;

		//flaps
//	case flapsdown:
//		F117::flap_command = 1.0 + (F117::AERO::Cx += 10.0) + (F117::AERO::Cl *= 2.0);
//		break;

//	case flapsup:
//		F117::flap_command = 0.0;
//		break;

//	case flapstoggle: //toggle
//		if (F117::ACTUATORS::flapPosition_DEG < 0.5)
//		F117::flap_command = 1.0;
//		else if (F117::ACTUATORS::flapPosition_DEG > 0.51)
//		F117::flap_command = 0.0;
//		break;


		//Air brakes
	case AirBrakes: //toggle
		if (F117::ACTUATORS::airbrake_state < 0.25) 
			F117::airbrake_command = 1.0;
		else if (F117::ACTUATORS::airbrake_state > 0.75) 
			F117::airbrake_command = 0.0;
		break;
	case AirBrakesOff:
		F117::airbrake_command = 0.0;
	case AirBrakesOn:
		F117::airbrake_command = 1.0;
		break;

	case tailhook:
		if (F117::ACTUATORS::tailhook_state < 0.25)
			F117::tailhook_CMD = 1.0;
		else if (F117::ACTUATORS::tailhook_state > 0.75)
			F117::tailhook_CMD = 0.0;
		break;

		//Dragshute
	case DragShute: //toggle
		if (F117::ACTUATORS::dragshute_state < 0.25)
			F117::dragshute_command = 1.0;
		else if (F117::ACTUATORS::dragshute_state > 0.75)
			F117::dragshute_command = 0.0;
		printf("Drag Shute = %f \n", dragshute_command);
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
		F117::rolling_friction = 0.16;
		break;
	case WheelBrakeOff:
		F117::rolling_friction = 0.015;
		break;

		//Other commands

	case bombay:
		if (misc_state < 0.5) F117::misc_cmd = 1.0;
		if (misc_state > 0.5) F117::misc_cmd = 0.0;
		break;

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


//	case FBW_override: // This doesn't work for now
//		F117::FBW = 1;
//		break;

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
	F117::weight_on_wheels = limit(drawargs[1] + drawargs[4] + drawargs[6], -1.0, 1.0);

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
	drawargs[21] = (float)ACTUATORS::airbrake_state;

	// Drag Chute
	drawargs[35] = (float)ACTUATORS::dragshute_state;

	// Weapon bay doors or tail hook
	drawargs[25] = (float)ACTUATORS::tailhook_state; // This is usually the tail hook for most planes with them.
	drawargs[26] = (float)limit((F117::misc_state), 0.0, 1.0); // This is usually weapon bays for planes with them.

	if (size > 616)
	{
		drawargs[611] = drawargs[0];
		drawargs[614] = drawargs[3];
		drawargs[616] = drawargs[5];
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

	drawargs[0] = F117::GearCommand;
	drawargs[3] = F117::GearCommand;
	drawargs[5] = F117::GearCommand;
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
		switch (index)
		{
		case ED_FM_ENGINE_0_RPM:			
		case ED_FM_ENGINE_0_RELATED_RPM:	
		case ED_FM_ENGINE_0_THRUST:			
		case ED_FM_ENGINE_0_RELATED_THRUST:	
			return 0; // APU

		case ED_FM_ENGINE_1_RPM:

			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((F117::throttle_state - 35) / 50.0), 0.0, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_RELATED_RPM:
			//return limit((F117::throttleInput), 0.0, 100.0);
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((F117::throttle_state - 25) / 75), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_THRUST:
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return F117::thrust_N;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_RELATED_THRUST: // This determines the heat blur effect and exhaust sound
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((F117::throttle_state - 25) / 75), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_CORE_RELATED_RPM: // This shows up as "RPM" in-game.
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((((F117::throttle_state - 25) / 75) + 1) / (2 + engine_damage)), 0.5, 1.0);
				//return limit((F117::ACTUATORS::throttle_state / 100), 0.4, 1.05);
			if (F117::internal_fuel < 5)
				return limit((F117::throttleInput), 0.0, 0.001) + (F117::throttleInput - 100);
			if (F117::engine_damage > 5)
				return limit((F117::throttleInput), 0.0, 0.001) + (F117::throttleInput - 100);

		case	ED_FM_ENGINE_1_TEMPERATURE:
			return limit((F117::ACTUATORS::throttle_state / 100), 0.5, 1.05)*500;
		case	ED_FM_ENGINE_1_OIL_PRESSURE:
			return F117::ACTUATORS::throttle_state * 10;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);
		case	ED_FM_ENGINE_1_FUEL_FLOW:
			if (F117::internal_fuel > 5)
				return limit((F117::ACTUATORS::throttle_state / 100), 0.25, 1.05) * 100;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_FC3_THROTTLE_LEFT:
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
			return limit((F117::throttleInput), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);


		case ED_FM_ENGINE_2_RPM:

			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit((F117::throttleInput), 0.3, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_RELATED_RPM:
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((F117::throttle_state - 25) / 75), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_THRUST:
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return F117::thrust_N;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_RELATED_THRUST: // This determines the heat blur effect and exhaust sound
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((F117::throttle_state - 25) / 75), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_CORE_RELATED_RPM: // This shows up as "RPM" in-game.
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit(((((F117::throttle_state - 25) / 75) + 1) / (2 + engine_damage)), 0.5, 1.0);
			if (F117::internal_fuel < 5)
				return limit((F117::throttleInput), 0.0, 0.001) + (F117::throttleInput - 100);
			if (F117::engine_damage > 5)
				return limit((F117::throttleInput), 0.0, 0.001) + (F117::throttleInput - 100);

		case	ED_FM_ENGINE_2_TEMPERATURE:
			return limit((F117::ACTUATORS::throttle_state / 100), 0.5, 1.05) * 500;
		case	ED_FM_ENGINE_2_OIL_PRESSURE:
			return F117::ACTUATORS::throttle_state * 10;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);
		case	ED_FM_ENGINE_2_FUEL_FLOW:
			if (F117::internal_fuel > 5)
				return limit((F117::ACTUATORS::throttle_state / 100), 0.25, 1.05) * 100;
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);

		case ED_FM_FC3_THROTTLE_RIGHT:
			if (F117::engineswitch == false)
				return limit((F117::throttleInput), 0.0, 0.0);
			if (F117::engineswitch == true)
				return limit((F117::throttleInput), 0.25, 1.0);
			if (F117::internal_fuel < 5)
				return 0 + limit((F117::throttleInput), 0.0, 0.001);
		}

	//other stuff
	switch (index)
	{

	case ED_FM_FUEL_INTERNAL_FUEL:
		return (F117::internal_fuel)+(F117::external_fuel);
	case ED_FM_FUEL_TOTAL_FUEL:
		return (F117::internal_fuel) + (F117::external_fuel);

	// These don't seem to do anything.
	case ED_FM_OXYGEN_SUPPLY:
		return 1000;
	case ED_FM_FLOW_VELOCITY:
		return 100;
	case ED_FM_FC3_SPEED_BRAKE_HANDLE_POS:
		return F117::airbrake_command*100;
	case ED_FM_FC3_STICK_PITCH:
		return F117::FLIGHTCONTROLS::longStickInput;
	case ED_FM_FC3_STICK_ROLL:
		return F117::FLIGHTCONTROLS::latStickInput;
	case ED_FM_FC3_RUDDER_PEDALS:
		return F117::pedInput;
	}
	}
	return 0;	

	// Autopilot stuff, work in progress
	switch (index)
	{
	case ED_FM_FC3_AUTOPILOT_STATUS:
		return F117::alt_hold;
	//case ED_FM_FC3_AUTOPILOT_FAILURE_ATTITUDE_STABILIZATION:
	}
}

// This defines what is reset when the plane is destroyed or the player restarts or quits the mission.
void ed_fm_release()
{
	F117::DeltaTime = 0;
	F117::simInitialized = false;
	F117::ACTUATORS::simInitialized = false;
	F117::FLIGHTCONTROLS::simInitialized = false;

	F117::engine_damage = 0;
	F117::Lwing_damage = 0;
	F117::Rwing_damage = 0;
	F117::tail_damage = 0;
	F117::cockpit_damage = 0;
	//FIX INERTIA!!!!

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
	F117::airbrake_command = 0.0;
	F117::airbrakes = 0.0;
	F117::tailhook_CMD = 0.0;
	F117::hook = 0.0;
	F117::dragshute_command = 0.0;
	F117::dragshute = 0.0;
	F117::misc_cmd = 0.0;
	F117::misc_state = 0.0;
	F117::misc_cmdH = 0.0;
	F117::misc_stateH = 0.0;

	F117::alt_hold = 0;
	F117::horiz_hold = 0;
	F117::alt_hold = 0;

	F117::ACTUATORS::tailhook_state = 0.0;
	F117::ACTUATORS::tailhook_rate = 0.0;
	F117::ACTUATORS::flapPosition_DEG = 0.0;
	F117::ACTUATORS::flapRate_DEGPERSEC = 0.0;
	F117::ACTUATORS::throttle_state;
	F117::ACTUATORS::throttle_rate;
	F117::ACTUATORS::misc_pos = 0.0;
	F117::ENGINE::percentPower = 0.0;
	F117::FLIGHTCONTROLS::latStickInput = 0.0;
	F117::FLIGHTCONTROLS::longStickInput = 0.0;
	F117::FLIGHTCONTROLS::longStickForce = 0.0;
	
}

// Conditions to make the screen shake in first-person view. This is very useful for debugging.
double ed_fm_get_shake_amplitude() 
{
	if (Rwing_damage >= 10 || Lwing_damage >= 10)
		return 10;

	if (cockpit_damage > 1)
	{ return 2; }

	if (engine_damage > 1)
	{
		return engine_damage;
	}

	if (F117::az > 80.0)  // If the plane is under 7 gs or more, the screen shakes.
	{return (F117::az / 150);}

	if (F117::ACTUATORS::airbrake_state > 0.2 && F117::mach >= 0.1)
	{
		return F117::ACTUATORS::airbrake_state * (mach / 6);
	}
	if (F117::ACTUATORS::flapPosition_DEG > 0.2 && F117::mach >= 0.1)
	{
		return F117::ACTUATORS::flapPosition_DEG * (mach / 10);
	}
	if (F117::alpha_DEG > 45 && F117::mach >= 0.1)
	{
		return F117::alpha_DEG / 100;
	}
	if (F117::beta_DEG > 15 && F117::mach >= 0.05)
	{
		return F117::beta_DEG / 50;
	}
	else
	{
		return 0;
	};
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
} 

// What parameters should be set to what in a hot start on the ground?
void ed_fm_hot_start() 
{
	F117::gearDown = 1;
	F117::GearCommand = 1;
	F117::throttleInput = 25.0;
	F117::WheelBrakeCommand = 0.0;
	F117::engineswitch = true;
	F117::rolling_friction = 0.015;
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
}

//What should be fixed when repairs are complete?
void ed_fm_repair()
{
	engine_damage = 0;
	Lwing_damage = 0;
	Rwing_damage = 0;
	tail_damage = 0;
	cockpit_damage = 0;
}

void ed_fm_set_immortal(bool value)
{
	F117::param_class.param_stuff::invincible(1 - value);
}

// Damage stuff, still a work in progress.
void ed_fm_on_damage(int Element, double element_integrity_factor)
{
	if (F117::param_class.invincible_value = 1)
	{ 
	if (Element == 103 || Element == 104 || Element == 11 || Element == 12) // Engines 1 and 2
	{ engine_damage += element_integrity_factor;}

	if (Element == 23 || Element == 25 || Element == 29 || Element == 35) // Left wing damage
	{ Lwing_damage += (element_integrity_factor);}

	if (Element == 24 || Element == 26 || Element == 30 || Element == 36) // Right wing damage
	{ Rwing_damage += (element_integrity_factor);}

	if (Element >= 53 && Element <= 58 || Element == 100) // Tail damage
	{ tail_damage += (element_integrity_factor);}

	if (Element >= 0 && Element <= 6) // Cockpit/Avionics damage
	{ cockpit_damage += (element_integrity_factor / 10);}
	}
	else
	{
		engine_damage = 0;
		Lwing_damage = 0;
		Rwing_damage = 0;
		tail_damage = 0;
		cockpit_damage = 0;
	}
}

double test()
{
	return 10.0;
}

#pragma once
