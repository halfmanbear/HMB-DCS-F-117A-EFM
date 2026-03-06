#ifndef __ACTUATORS__
#define __ACTUATORS__

#include "../stdafx.h"
#include <stdio.h>
#include <string>
#include <math.h>
#include "../UtilityFunctions.h"

namespace F117
{
	namespace ACTUATORS
	{
		double	elevatorPosition_DEG = 0.0;
		double	elevatorRate_DEGPERSEC = 0.0;
		double	aileronPosition_DEG = 0.0;
		double	aileronRate_DEGPERSEC = 0.0;
		//double	flapPosition_DEG = 0.0;
		//double	flapRate_DEGPERSEC = 0.0;
		double	throttle_state = 0.0;
		double	throttle_rate = 0.0;
		double	gear_state = 0.0;
		double	gear_rate = 0.0;
		//double	airbrake_state = 0.0;
		//double	airbrake_rate = 0.0;
		double	tailhook_state = 0.0;
		double	tailhook_rate = 0.0;
		double	dragchute_state = 0.0;
		double	dragchute_rate = 0.0;
		double  internal_fuel;

		bool	simInitialized = false;

		double  elevator_actuator(double elevator_DEG_commanded, double frameTime)
		{
			if (!simInitialized)
			{
				elevatorPosition_DEG = elevator_DEG_commanded;
				return elevatorPosition_DEG;
			}

			elevatorRate_DEGPERSEC = 20.2 * (elevator_DEG_commanded - elevatorPosition_DEG);

			elevatorRate_DEGPERSEC = limit(elevatorRate_DEGPERSEC, -0.5, 0.5);

			elevatorPosition_DEG += (elevatorRate_DEGPERSEC * frameTime);

			elevatorPosition_DEG = limit(elevatorPosition_DEG, -20.0, 20.0);

			return elevatorPosition_DEG;
		}

		double  aileron_actuator(double roll_cmd, double frameTime)
		{
			if (!simInitialized)
			{
				aileronPosition_DEG = roll_cmd;
				return aileronPosition_DEG;
			}

			aileronRate_DEGPERSEC = 15.0 * (roll_cmd - aileronPosition_DEG);

			aileronRate_DEGPERSEC = limit(aileronRate_DEGPERSEC, -20.0, 20.0);

			aileronPosition_DEG += (aileronRate_DEGPERSEC * frameTime);

			aileronPosition_DEG = limit(aileronPosition_DEG, -20.0, 20.0);

			return aileronPosition_DEG;
		}

		float	rudderPosition_DEG = 0.0;
		float	rudderRate_DEGPERSEC = 0.0;

		float  rudder_actuator(float rudderCommanded_DEG, double frameTime)
		{
			if (!simInitialized)
			{
				rudderPosition_DEG = rudderCommanded_DEG;
				return rudderPosition_DEG;
			}

			rudderRate_DEGPERSEC = (float)(20.2 * (rudderCommanded_DEG - rudderPosition_DEG));

			rudderRate_DEGPERSEC = (float)limit(rudderRate_DEGPERSEC, -3.0, 3.0);

			rudderPosition_DEG += (float)(rudderRate_DEGPERSEC * frameTime);

			rudderPosition_DEG = (float)limit(rudderPosition_DEG, -1.0, 1.0);

			return rudderPosition_DEG;
		}

		double  throttle_actuator(double throttleInput, double frameTime)
		{
			if (!simInitialized)
					{
						throttle_state = throttleInput;
						return throttle_state;
					}
					throttle_rate = 20.2 * (throttleInput - throttle_state);
					throttle_rate = limit(throttle_rate, -25.0, 20.0);
					throttle_state += (throttle_rate * frameTime);
					throttle_state = limit(throttle_state, 0.0, 100.0);
			return throttle_state;
		}
		double  gear_actuator(double GearCommand, double frameTime, bool weight_on_wheels)
		{
			if (weight_on_wheels && GearCommand < 1.0)
			{
				GearCommand = 1.0;
			}

			if (!simInitialized)
			{
				gear_state = GearCommand;
				return gear_state;
			}
			gear_rate = 20.2 * (GearCommand - gear_state);
			gear_rate = limit(gear_rate, -0.5, 0.5);
			gear_state += (gear_rate * frameTime);
			gear_state = limit(gear_state, 0.0, 100.0);

			return gear_state;
		}
		//double  flaps_actuator(double flap_command, double frameTime)
		//{
		//	if (!simInitialized)
		//	{
		//		flapPosition_DEG = flap_command;
		//		return flapPosition_DEG;
		//	}
		//	flapRate_DEGPERSEC = 20.2 * (flap_command - flapPosition_DEG);
		//	flapRate_DEGPERSEC = limit(flapRate_DEGPERSEC, -1.0, 1.0);
		//	flapPosition_DEG += (flapRate_DEGPERSEC * frameTime);
		//	flapPosition_DEG = limit(flapPosition_DEG, 0.0, 1.0);
//
	//		return flapPosition_DEG;
	//	}

		float elev_pos = 0.0;
		float elev_rate = 0.0;

		float  elev_actuator(float longStickInput, double frameTime)
		{
			if (!simInitialized)
			{
				elev_pos = longStickInput;
				return elev_pos;
			}

			elev_rate = (float)(20.2 * (longStickInput - elev_pos));
			elev_rate = (float)limit(elev_rate, -4.0, 4.0);
			elev_pos += (float)(elev_rate * frameTime);
			elev_pos = (float)limit(elev_pos, -1.0, 1.0);

			return elev_pos;
		}

	//	double  airbrake_actuator(double airbrake_command, double frameTime)
	//	{
	//		if (!simInitialized)
	//		{
	//			airbrake_state = airbrake_command;
	//			return airbrake_state;
	//		}
	//		airbrake_rate = 20.2 * (airbrake_command - airbrake_state);
	//		airbrake_rate = limit(airbrake_rate, -0.75, 0.75);
	//		airbrake_state += (airbrake_rate * frameTime);
	//		airbrake_state = limit(airbrake_state, 0.0, 1.0);
//
//			return airbrake_state;
//		}

		double tailhook_actuator(double tailhook_command, double frameTime)
		{
			if (!simInitialized)
			{
				tailhook_state = tailhook_command;
				return tailhook_state;
			}
			tailhook_rate = 20.2 * (tailhook_command - tailhook_state);
			tailhook_rate = limit(tailhook_rate, -0.75, 0.75);
			tailhook_state += (tailhook_rate * frameTime);
			tailhook_state = limit(tailhook_state, 0.0, 1.0);
	
			return tailhook_state;
		}

		double dragchute_actuator(double dragchute_command, double frameTime, double velocity_fps, double gear_down,
			bool weight_on_wheels)
		{
			if (!simInitialized)
			{
				dragchute_state = dragchute_command;
				return dragchute_state;
			}

			// DRAG CHUTE DEPLOYMENT SAFETY CHECKS
			// Only allow deployment if:
			// 1. On the ground (weight on wheels)
			// 2. Speed between 60-180 knots (100-300 fps)
			// 3. Once deployed, can stay deployed until retracted or speed drops too low

			double minDeploySpeed_FPS = 100.0;  // ~60 knots minimum
			double maxDeploySpeed_FPS = 270.0;  // ~180 knots maximum (chute will tear off above this)
			double minSustainSpeed_FPS = 30.0;  // ~18 knots - auto-retract below this

			bool speedOkForDeploy = (velocity_fps >= minDeploySpeed_FPS &&
				velocity_fps <= maxDeploySpeed_FPS);
			bool speedTooSlow = (velocity_fps < minSustainSpeed_FPS);
			bool speedTooFast = (velocity_fps > maxDeploySpeed_FPS);

			// Override command based on conditions
			double safeCommand = dragchute_command;

			if (dragchute_command > 0.1) {
				// User wants to deploy
				if (!weight_on_wheels) {
					safeCommand = 0.0;  // Block deployment in air
				}
				else if (!speedOkForDeploy && dragchute_state < 0.1) {
					safeCommand = 0.0;  // Block initial deployment if speed not in range
				}
			}

			// Auto-retract if speed too slow or too fast
			if (speedTooSlow || speedTooFast) {
				safeCommand = 0.0;  // Force retraction
			}

			// Actuator dynamics
			if (safeCommand < 0.1) {
				// Instant retraction (jettison/stow)
				dragchute_state = 0.0;
			}
			else {
				// Gradual deployment
				dragchute_rate = 20.2 * (safeCommand - dragchute_state);
				dragchute_rate = limit(dragchute_rate, 0.0, 0.75);  // Only positive rate (deploy only)
				dragchute_state += (dragchute_rate * frameTime);
				dragchute_state = limit(dragchute_state, 0.0, 1.0);
			}

			return dragchute_state;
		}

		float misc_pos = 0.0;
		float misc_rate = 0.0;

		float  misc_actuator(float misc_cmd, double frameTime)
		{
			if (!simInitialized)
			{
				misc_pos = misc_cmd;
				return misc_pos;
			}
			misc_rate = (float)(20.2 * (misc_cmd - misc_pos));
			misc_rate = (float)limit(misc_rate, -0.5, 0.5);
			misc_pos += (float)(misc_rate * frameTime);
			misc_pos = (float)limit(misc_pos, 0.0, 1.0);

			return misc_pos;
		}

		float misc_posH = 0.0;
		float misc_rateH = 0.0;

		float  misc_actuatorH(float misc_cmdH, double frameTime)
		{
			if (!simInitialized)
			{
				misc_posH = misc_cmdH;
				return misc_posH;
			}
			misc_rateH = (float)(20.2 * (misc_cmdH - misc_posH));
			misc_rateH = (float)limit(misc_rateH, -0.5, 0.5);
			misc_posH += (float)(misc_rateH * frameTime);
			misc_posH = (float)limit(misc_posH, 0.0, 1.0);

			return misc_posH;
		}
	};


}

#endif
