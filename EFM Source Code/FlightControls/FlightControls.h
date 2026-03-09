#ifndef __F117FLIGHTCONTROLS__
#define __F117FLIGHTCONTROLS__

#include "../stdafx.h"

namespace F117
{
	namespace FLIGHTCONTROLS
	{
		extern bool simInitialized;
		extern double longStickInput;
		extern double alphaFiltered;
		extern double longStickForce;
		extern double latStickInput;

		double fcs_yaw_controller(double pedInput, double pedTrim, double yaw_rate, double roll_rate, double aoa_filtered, double aileron_commanded, double ay, double dt);
		double fcs_pitch_controller_force_command(double longStickInput, double pitchTrim, double dt);
		double dynamic_pressure_schedule(double dynPressure_LBFT2);
		double fcs_pitch_controller(double longStickInput, double pitchTrim, double angle_of_attack_ind, double pitch_rate, double az, double differentialCommand, double dynPressure_LBFT2, double dt, double roll_angle_DEG, double pitch_angle_DEG, double velocity_fps, double mach, double thrust_N, double Cx_total);
		double fcs_roll_controller(double latStickInput, double longStickForce, double ay, double roll_rate, double roll_rate_trim, double dynPressure_LBFT2, double dt);
	}
}

#endif
