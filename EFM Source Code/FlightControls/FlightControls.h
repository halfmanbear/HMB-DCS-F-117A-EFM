#pragma once

namespace F117
{
namespace Flight_controls
{
extern bool sim_initialized;
extern double long_stick_input;
extern double alpha_filtered;
extern double long_stick_force;
extern double lat_stick_input;

void reset_runtime_state();
void update_pitch_mode_auto_inputs(double gear_down, bool air_refuel_door_open);
void update_ground_contact(bool wheel_on_ground, double dt);
void set_recovery_nz_command(bool active, double nz);
void update_yaw_debug_snapshot(double beta_deg, double yaw_rate_rps, double rudder_deg_commanded, double rudder_deg,
                               double rudder_pct);
void update_aero_debug_snapshot(double cy_delta_rudder, double cn_delta_rudder, double cl_delta_rudder,
                                double cn_delta_beta, double cl_delta_beta, double cy_total, double cn_total,
                                double cl_total);
double fcs_yaw_controller(double ped_input, double ped_trim, double beta_deg, double yaw_rate, double turn_yaw_rate,
                          double roll_rate, double pitch_rate, double aoa_filtered, double dyn_pressure_pa,
                          bool gear_down, double dt);
double fcs_pitch_controller_force_command(double long_stick_input, double pitch_trim, double dt);
double fcs_pitch_controller(double long_stick_input, double pitch_trim, double angle_of_attack_ind,
                            double pitch_rate_deg_s, double az, double differential_command, double dyn_pressure_pa,
                            double dt, double roll_angle_deg, double pitch_angle_deg, double roll_rate_deg_s,
                            double velocity_mps, double mach, double thrust_n, double cx_total);
double fcs_roll_controller(double lat_stick_input, double long_stick_force, double ay, double ped_input,
                           double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim,
                           double dyn_pressure_pa, double roll_accel_per_surface_deg, double surface_per_roll_rate,
                           bool gear_down, bool air_refuel_door_open, double dt);
}  // namespace Flight_controls
}  // namespace F117
