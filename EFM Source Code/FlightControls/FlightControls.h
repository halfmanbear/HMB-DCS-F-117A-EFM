#ifndef __F117FLIGHTCONTROLS__
#define __F117FLIGHTCONTROLS__

#include "../stdafx.h"
#include <stdio.h>
#include <string>
#include <math.h>
#include "../UtilityFunctions.h"
#include "../include/general_filter.h"
#include <fstream>
#include <cstdlib>

namespace F117
{
	namespace FLIGHTCONTROLS
	{
		bool		simInitialized = false;

		static std::ofstream debugLog;
		static int logCounter = 0;

		// Control filters
		GeneralFilter	pitchRateWashout;
		GeneralFilter	pitchIntegrator;
		GeneralFilter	pitchPreActuatorFilter;
		GeneralFilter	pitchActuatorDynamicsFilter;
		GeneralFilter	accelFilter;
		GeneralFilter	latStickForceFilter;
		GeneralFilter	rollCommandFilter;
		GeneralFilter	rollActuatorDynamicsFilter;
		GeneralFilter	rollRateFilter1;
		GeneralFilter	rollRateFilter2;
		GeneralFilter	rudderCommandFilter;
		GeneralFilter	yawRateWashout;
		GeneralFilter	yawRateFilter;
		GeneralFilter	yawServoFilter;

		// Pitch controller variables
		double		longStickInput = 0.0;
		double		stickCommandPosFiltered = 0.0;
		double		azFiltered = 0.0;
		double		alphaFiltered = 0.0;
		double		longStickForce = 0.0;
		double		latStickInput = 0.0;

		double roll_angle_DEG;
		double pitch_angle_DEG;


		constexpr double PI = 3.14159265358979323846;

		inline double sgn(double x) {
			return (x > 0.0) - (x < 0.0);
		}

		// Controller for yaw - FBW yaw damper with sideslip suppression
		double fcs_yaw_controller(double pedInput, double pedTrim, double yaw_rate, double roll_rate, double aoa_filtered, double aileron_commanded, double ay, double dt)
		{
			// --- Filter initialization ---
			if (!(simInitialized))
			{
				// Pedal command lag filter (4 rad/s first-order)
				double numerators[2] = { 0.0, 4.0 };
				double denominators[2] = { 1.0, 4.0 };
				rudderCommandFilter.InitFilter(numerators, denominators, 1, dt);

				// Yaw rate high-pass washout (tau = 1s)
				double numerators1[2] = { 1.0, 0.0 };
				double denominators1[2] = { 1.0, 1.0 };
				yawRateWashout.InitFilter(numerators1, denominators1, 1, dt);

				// Yaw rate lead-lag smoothing (15 rad/s)
				double numerators2[2] = { 3.0, 15.0 };
				double denominators2[2] = { 1.0, 15.0 };
				yawRateFilter.InitFilter(numerators2, denominators2, 1, dt);

				// Actuator dynamics (2nd order, wn=52 rad/s, zeta=0.7)
				double numerators3[3] = { 0.0, 0.0, pow(52.0, 2.0) };
				double denominators3[3] = { 1.0, 2.0 * 0.7 * 52.0, pow(52.0, 2.0) };
				yawServoFilter.InitFilter(numerators3, denominators3, 2, dt);
			}

			// --- Pedal force command ---
			double rudderForceCommand = pedInput * 450.0;

			double rudderCommand = 0.0;
			if (abs(rudderForceCommand) < 44.0)
			{
				rudderCommand = 0.0;
			}
			else if (rudderForceCommand >= 44.0)
			{
				rudderCommand = -0.0739 * rudderForceCommand + 3.2512;
			}
			else if (rudderForceCommand <= -44.0)
			{
				rudderCommand = -0.0739 * rudderForceCommand - 3.2512;
			}

			rudderCommand = limit(rudderCommand, -30.0, 30.0);
			double rudderCommandFiltered = rudderCommandFilter.Filter(!(simInitialized), dt, rudderCommand);
			double pedalCommand = pedTrim - rudderCommandFiltered;

			// --- Yaw rate feedback (damping) ---
			double alphaGained = aoa_filtered * (5.0 / 57.3);
			double rollRateCoupling = roll_rate * alphaGained;
			double yawRateCorrected = yaw_rate - rollRateCoupling;

			double yawRateWashedOut = yawRateWashout.Filter(!(simInitialized), dt, yawRateCorrected);
			double yawRateSmoothed = yawRateFilter.Filter(!(simInitialized), dt, yawRateWashedOut);

			double yawDamping = 1.5 * yawRateSmoothed;

			// --- Lateral acceleration feedback (sideslip suppression) ---
			double sideAccelFeedback = 0.5 * ay;

			// --- Aileron-to-rudder interconnect (ARI) ---
			double ariCommand = limit(0.05 * aoa_filtered, 0.0, 1.5) * aileron_commanded;

			// --- Command summation ---
			double combinedCommand = pedalCommand + yawDamping + sideAccelFeedback + ariCommand;

			// --- Actuator dynamics ---
			double finalRudderCommand = yawServoFilter.Filter(!(simInitialized), dt, combinedCommand);

			return finalRudderCommand;
		}

		// Stick force schedule for pitch control
		double pitchTrim = 0.0;
		double fcs_pitch_controller_force_command(double longStickInput, double pitchTrim, double dt)
		{
			double longStickInputForce = 0.0;
			if (longStickInput > 0.0)
			{
				longStickInputForce = longStickInput * 80.0 + pitchTrim;
			}
			else
			{
				longStickInputForce = longStickInput * 180.0;
			}
			longStickInputForce = limit(longStickInputForce, -180.0, 80.0);
			longStickForce = longStickInputForce;

			double longStickCommand_G = 0.0;
			if (abs(longStickInputForce) <= 8.0)
			{
				longStickCommand_G = 0.0;
			}
			else if ((longStickInputForce < -8) && (longStickInputForce > -33.0))
			{
				longStickCommand_G = (0.016 * longStickInputForce) + 0.128;
			}
			else if (longStickInputForce <= -33.0)
			{
				longStickCommand_G = (0.067 * longStickInputForce) + 1.8112;
			}
			else if ((longStickInputForce > 8.0) && (longStickInputForce < 33.0))
			{
				longStickCommand_G = (0.032 * longStickInputForce) - 0.256;
			}
			else if (longStickInputForce >= 33.0)
			{
				longStickCommand_G = 0.0681 * longStickInputForce - 1.4468;
			}

			double longStickCommandWithTrim_G = pitchTrim - longStickCommand_G;

			double longStickCommandWithTrimLimited_G = limit(longStickCommandWithTrim_G, -4.0, 8.0);

			double longStickCommandWithTrimLimited_G_Rate = 4.0 * (longStickCommandWithTrimLimited_G - stickCommandPosFiltered);
			stickCommandPosFiltered += (longStickCommandWithTrimLimited_G_Rate * dt);

			return stickCommandPosFiltered;
		}

		// Schedule gain component due to dynamic pressure
		double dynamic_pressure_schedule(double dynPressure_LBFT2)
		{
			double dynamicPressure_kNM2 = dynPressure_LBFT2 * 1.4881639 / 1000.0;
			double scheduleOutput = 0.0;

			if (dynamicPressure_kNM2 < 9.576)
			{
				scheduleOutput = 1.0;
			}
			else if ((dynamicPressure_kNM2 >= 9.576) && (dynamicPressure_kNM2 <= 43.0))
			{
				scheduleOutput = (-0.018 * dynamicPressure_kNM2) + 1.1719;
			}
			else if (dynamicPressure_kNM2 > 43.0)
			{
				scheduleOutput = -0.003 * dynamicPressure_kNM2 + 0.5277;
			}

			scheduleOutput = limit(scheduleOutput, 0.5, 1.0);

			return scheduleOutput;
		}

		double fcs_pitch_controller(double longStickInput, double pitchTrim, double angle_of_attack_ind, double pitch_rate, double az, double differentialCommand, double dynPressure_LBFT2, double dt, double roll_angle_DEG, double pitch_angle_DEG, double velocity_fps, double mach, double thrust_N, double Cx_total)
		{
			if (!(simInitialized))
			{
				double numerators[2] = { 1.0, 0.0 };
				double denominators[2] = { 1.0, 1.0 };
				pitchRateWashout.InitFilter(numerators, denominators, 1, dt);

				numerators[0] = 0.0; numerators[1] = 2.5;
				denominators[0] = 1.0; denominators[1] = 0.0;
				pitchIntegrator.InitFilter(numerators, denominators, 1, dt);

				numerators[0] = 3.0; numerators[1] = 15;
				denominators[0] = 1.0; denominators[1] = 15.0;
				pitchPreActuatorFilter.InitFilter(numerators, denominators, 1, dt);

				double numerators2[3] = { 0.0, 0.0, pow(52.0, 2.0) };
				double denominators2[3] = { 1.0, 2.0 * 0.7 * 52.0, pow(52.0, 2.0) };
				pitchActuatorDynamicsFilter.InitFilter(numerators2, denominators2, 2, dt);

				numerators[0] = 0.0; numerators[1] = 15.0;
				denominators[0] = 1.0; denominators[1] = 15.0;
				accelFilter.InitFilter(numerators, denominators, 1, dt);

				stickCommandPosFiltered = 0.0;

				// Open debug log file on user's desktop
				const char* userProfile = getenv("USERPROFILE");
				std::string desktopPath;
				if (userProfile != nullptr)
				{
					desktopPath = std::string(userProfile) + "\\Desktop\\F117_GLimiter_Debug.csv";
				}
				else
				{
					// Fallback to hardcoded path
					desktopPath = "C:\\Users\\Fraser\\Desktop\\F117_GLimiter_Debug.csv";
				}
				debugLog.open(desktopPath);
				if (debugLog.is_open())
				{
					debugLog << "roll_deg,pitch_deg,az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,alpha_raw,alphaFiltered,stickInput,pitchRate,dynPressure,velocity_fps,mach,thrust_N,Cx_total,elevatorOut\n";
				}
			}

			double stickCommandPos = fcs_pitch_controller_force_command(longStickInput, pitchTrim, dt);
			double dynamicPressureScheduled = dynamic_pressure_schedule(dynPressure_LBFT2);

			double roll_RAD = roll_angle_DEG * (PI / 180.0);
			double pitch_RAD = pitch_angle_DEG * (PI / 180.0);

			azFiltered = accelFilter.Filter(!(simInitialized), dt, az);

			double alphaLimited = limit(angle_of_attack_ind, -5.0, 20.0);
			double alphaLimitedRate = 35.0 * (alphaLimited - alphaFiltered); // Increased for snappier AoA response testing
			alphaFiltered += (alphaLimitedRate * dt);

			double pitchRateWashedOut = pitchRateWashout.Filter(!(simInitialized), dt, pitch_rate);

			const double NZ_MAX = 6.3;
			const double NZ_MIN = -3.0;

			double gravity_component = cos(roll_RAD) * cos(pitch_RAD);
			double nz_measured = azFiltered + gravity_component;

			static double nz_reference = 1.0;

			const double STICK_DEADBAND = 0.05;
			const double NZ_RETURN_RATE = 2.0;  // Return to 1G when stick centered (time constant 0.5s)

			if (fabs(stickCommandPos) < STICK_DEADBAND)
			{
				// Stick centered: drive reference back toward 1G
				nz_reference += (1.0 - nz_reference) * NZ_RETURN_RATE * dt;
			}
			// When stick is deflected, hold nz_reference steady - don't track measured G
			// (tracking toward measured G caused windup: reference climbed during pulls
			// and stayed high after stick release, commanding uncommanded G)

			// No attitude flip - pull always commands positive nz (body frame)
			double nz_cmd = nz_reference + stickCommandPos;
			nz_cmd = limit(nz_cmd, NZ_MIN, NZ_MAX);

			double nz_error = nz_cmd - nz_measured;

			double pitchDamping = -2.5 * pitchRateWashedOut;  // Increased damping to suppress post-maneuver oscillation

			double nz_control = (8.0 * nz_error) + pitchDamping; //was 4 

			bool nzAtUpperLimit = (nz_measured >= NZ_MAX && nz_error > 0.0);
			bool nzAtLowerLimit = (nz_measured <= NZ_MIN && nz_error < 0.0);

			double integratorInput =
				(nzAtUpperLimit || nzAtLowerLimit) ? 0.0 : nz_control;

			double finalCombinedCommandFilteredLimited =
				limit(
					pitchIntegrator.Filter(!(simInitialized), dt, integratorInput),
					-25.0,
					25.0
				);

			double finalPitchCommandTotal =
				pitchPreActuatorFilter.Filter(
					!(simInitialized),
					dt,
					finalCombinedCommandFilteredLimited
				);

			// AoA limiter constants
			const double AOA_MAX = 20.0;  // degrees - F-117A FBW hard limit
			const double AOA_LIMIT_START = 19.0;  // degrees - start limiting here (raised for more agility)

			// Hard AoA limiter - applied AFTER all other terms
			// When AoA exceeds limit, command nose-down regardless of other inputs
			if (alphaFiltered > AOA_LIMIT_START)
			{
				double aoaError = alphaFiltered - AOA_LIMIT_START;
				double aoaPushdown = 0.30 * aoaError;  // Softer nose-down for more agility near limit // was 0.35

				// Reduce any nose-up command and add nose-down
				if (finalPitchCommandTotal > 0.0)
				{
					// Scale down pull commands as we approach limit
					double limitFactor = 1.0 - (aoaError / (AOA_MAX - AOA_LIMIT_START));
					limitFactor = limit(limitFactor, 0.0, 1.0);
					finalPitchCommandTotal *= limitFactor;
				}

				// Add nose-down command proportional to how far over the limit start
				finalPitchCommandTotal -= aoaPushdown;
			}

			// Debug logging (every 100 frames to avoid flooding)
			logCounter++;
			if (debugLog.is_open() && (logCounter % 100 == 0))
			{
				debugLog << roll_angle_DEG << ","
				         << pitch_angle_DEG << ","
				         << az << ","
				         << gravity_component << ","
				         << nz_measured << ","
				         << nz_reference << ","
				         << stickCommandPos << ","
				         << nz_cmd << ","
				         << nz_error << ","
				         << angle_of_attack_ind << ","  // Raw AoA before filtering
				         << alphaFiltered << ","
				         << longStickInput << ","       // Raw stick input -1 to +1
				         << pitch_rate << ","           // Pitch rate for damping analysis
				         << dynPressure_LBFT2 << ","    // Dynamic pressure
				         << velocity_fps << ","         // Airspeed (ft/s)
				         << mach << ","                 // Mach number
				         << thrust_N << ","             // Engine thrust (N)
				         << Cx_total << ","             // Total drag coefficient
				         << finalPitchCommandTotal << "\n";
				debugLog.flush();
			}

			return finalPitchCommandTotal;
		}

		// Controller for roll
		double fcs_roll_controller(double latStickInput, double longStickForce, double ay, double roll_rate, double roll_rate_trim, double dynPressure_LBFT2, double dt)
		{
			if (!(simInitialized))
			{
				double numerators[2] = { 0.0,60.0 };
				double denominators[2] = { 1.0,60.0 };
				latStickForceFilter.InitFilter(numerators, denominators, 1, dt);

				double numerators1[2] = { 0.0,10.0 };
				double denominators1[2] = { 1.0,10.0 };
				rollCommandFilter.InitFilter(numerators1, denominators1, 1, dt);

				double numerators2[3] = { 0.0, 0.0, pow(52.0,2.0) };
				double denomiantors2[3] = { 1.0, 2.0 * 0.7 * 52.0, pow(52.0,2.0) };
				rollActuatorDynamicsFilter.InitFilter(numerators2, denomiantors2, 2, dt);

				double numerators3[2] = { 0.0,50.0 };
				double denominators3[2] = { 1.0,50.0 };
				rollRateFilter1.InitFilter(numerators3, denominators3, 1, dt);

				double numerators4[3] = { 4.0, 64.0, 6400.0 };
				double denomiantors4[3] = { 1.0, 80.0, 6400.0 };
				rollRateFilter2.InitFilter(numerators4, denomiantors4, 2, dt);
			}

			double latStickForceCmd = latStickInput * 75.0;
			double latStickForce = latStickForceFilter.Filter(!(simInitialized), dt, latStickForceCmd);

			double latStickForceBiased = latStickForce - (ay * 8.9);

			double longStickForceGained = longStickForce * 0.0667;
			double rollFeelGain = 0.0;
			if (abs(longStickForce) > 25.0)
			{
				rollFeelGain = 0.7;
			}
			else if (longStickForce >= 0.0)
			{
				rollFeelGain = -0.012 * longStickForceGained + 1.0;
			}
			else if (longStickForce < 0.0)
			{
				rollFeelGain = 0.012 * longStickForceGained + 1.0;
			}

			double latStickForceFinal = latStickForceBiased * rollFeelGain;

			double rollRateCommand = 0.0;
			if (abs(latStickForceFinal) < 3.0)
			{
				rollRateCommand = 0.0;
			}
			else if ((latStickForceFinal >= 3.0) && (latStickForceFinal <= 25.0))
			{
				rollRateCommand = 0.9091 * latStickForceFinal - 2.7273;
			}
			else if ((latStickForceFinal > 25.0) && (latStickForceFinal <= 46.0))
			{
				rollRateCommand = 2.8571 * latStickForceFinal - 51.429;
			}
			else if ((latStickForceFinal > 46.0))
			{
				rollRateCommand = 7.5862 * latStickForceFinal - 268.97;
			}
			else if ((latStickForceFinal <= -3.0) && (latStickForceFinal >= -25.0))
			{
				rollRateCommand = 0.9091 * latStickForceFinal + 2.7273;
			}
			else if ((latStickForceFinal < -25.0) && (latStickForceFinal >= -46.0))
			{
				rollRateCommand = 2.8571 * latStickForceFinal + 51.429;
			}
			else if ((latStickForceFinal < -46.0))
			{
				rollRateCommand = 7.5862 * latStickForceFinal + 268.97;
			}

			double rollRateCommandFilterd = rollCommandFilter.Filter(!(simInitialized), dt, rollRateCommand);

			double rollRateFiltered1 = rollRateFilter1.Filter(!(simInitialized), dt, roll_rate);

			double rollRateFiltered2 = (rollRateFilter2.Filter(!(simInitialized), dt, rollRateFiltered1));

			double rollRateCommandCombined = rollRateFiltered2 - rollRateCommandFilterd - roll_rate_trim;

			double dynamicPressure_NM2 = dynPressure_LBFT2 * 47.880258889;

			double pressureGain = 0.0;
			if (dynamicPressure_NM2 < 19153.0)
			{
				pressureGain = 0.2;
			}
			else if ((dynamicPressure_NM2 >= 19153.0) && (dynamicPressure_NM2 <= 23941.0))
			{
				pressureGain = -0.00002089 * dynamicPressure_NM2 + 0.6;
			}
			else
			{
				pressureGain = 0.1;
			}

			double rollCommandGained = limit(rollRateCommandCombined * pressureGain, -21.5, 21.5);

			double rollActuatorCommand = rollActuatorDynamicsFilter.Filter(!(simInitialized), dt, rollCommandGained);
			return rollActuatorCommand;
		}
	}
}
#endif