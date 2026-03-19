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

		void reset_runtime_state();
		void update_pitch_mode_auto_inputs(double gearDown, bool airRefuelDoorOpen);
		void update_yaw_debug_snapshot(double betaDeg, double yawRateRps, double rudderDegCommanded, double rudderDeg, double rudderPct);
		void update_aero_debug_snapshot(double cyDeltaRudder, double cnDeltaRudder, double clDeltaRudder, double cnDeltaBeta, double clDeltaBeta, double cyTotal, double cnTotal, double clTotal);
		double fcs_yaw_controller(double pedInput, double pedTrim, double beta_deg, double yaw_rate, double roll_rate, double pitch_rate, double aoa_filtered, double dynPressure_PA, bool gearDown, double dt);
		double fcs_pitch_controller_force_command(double longStickInput, double pitchTrim, double dt);
		double fcs_pitch_controller(double longStickInput, double pitchTrim, double angle_of_attack_ind, double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_PA, double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s, double velocity_mps, double mach, double thrust_N, double Cx_total);
		double fcs_roll_controller(double latStickInput, double longStickForce, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_PA, bool gearDown, bool airRefuelDoorOpen, double dt);
	}
}

#endif

