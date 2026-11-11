#pragma once
// CSV debug log of the pitch control law (F117_GLimiter_Debug.csv in the DCS Logs folder).

namespace F117
{
namespace Flight_controls
{
namespace Detail
{

struct Pitch_debug_sample
{
    double roll_angle_deg{};
    double pitch_angle_deg{};
    double roll_rate_deg_s{};
    double raw_az{};
    double gravity_component{};
    double nz_measured{};
    double nz_reference{};
    double stick_command_pos{};
    double nz_command{};
    double nz_error{};
    double nz_proportional{};
    double pitch_rate_blend{};  // washed pitch-rate blend contribution (g) in the error signal
    double nz_control{};
    double integrator_out{};
    double raw_ao_a{};
    double filtered_ao_a{};
    double raw_stick_input{};
    double pitch_rate{};
    double dyn_pressure_pa{};
    double velocity_mps{};
    double mach{};
    double thrust_n{};
    double cx_total{};
    double elevator_out{};
};

void open_pitch_debug_log();
void close_pitch_debug_log();
// Writes one row every kPitchDebugLogIntervalFrames calls when the log is open.
void log_pitch_debug_sample(const Pitch_debug_sample& sample);

}  // namespace Detail
}  // namespace Flight_controls
}  // namespace F117
