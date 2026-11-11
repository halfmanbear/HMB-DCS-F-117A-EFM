#include "stdafx.h"
#include "PitchLog.h"

#include <fstream>
#include <string>

#include "Internal.h"
#include "../Diagnostics/Paths.h"

namespace F117
{
namespace Flight_controls
{
namespace Detail
{

namespace
{
std::ofstream debug_log;
int log_counter = 0;

void write_pitch_debug_header()
{
    // Columns: pitch_mode=0 always in flight (NormalAcceleration P+I law).
    // pitch_rate_blend = washed pitch-rate contribution (g) blended into the Nz error signal.
    debug_log << "roll_deg,pitch_deg,roll_rate_dps,roll_stick,roll_stick_force,roll_rate_cmd,roll_flat_turn_cmd,"
                 "roll_rate_cmd_filt,roll_cmd_surface,roll_surface_out,yaw_ped_input,beta_deg,yaw_rate_rps,"
                 "yaw_rate_dps,yaw_rudder_cmd,yaw_rudder_cmd_filt,yaw_pedal_cmd,yaw_damping,yaw_side_accel,"
                 "yaw_ari_cmd,yaw_combined_cmd,yaw_surface_out,rudder_deg_cmd,rudder_deg,rudder_pct,aero_cy_dr,"
                 "aero_cn_dr,aero_cl_dr,aero_cn_dbeta,aero_cl_dbeta,aero_cy_total,aero_cn_total,aero_cl_total,"
                 "az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,nz_proportional,"
                 "pitch_rate_blend,nz_control,integratorOut,alpha_raw,alphaFiltered,stickInput,pitchRate,"
                 "dynPressure_pa,velocity_mps,mach,thrust_N,Cx_total,elevatorOut,ground_contact_fade\n";
}
}  // namespace

void open_pitch_debug_log()
{
    log_counter = 0;
    if (debug_log.is_open())
    {
        debug_log.close();
    }

    const std::string dcs_log_path = build_saved_games_dcs_log_path("DCS", "F117_GLimiter_Debug.csv");
    if (!dcs_log_path.empty())
    {
        debug_log.open(dcs_log_path);
    }
    if (!debug_log.is_open())
    {
        const std::string open_beta_log_path =
            build_saved_games_dcs_log_path("DCS.openbeta", "F117_GLimiter_Debug.csv");
        if (!open_beta_log_path.empty())
        {
            debug_log.open(open_beta_log_path);
        }
    }
    if (!debug_log.is_open())
    {
        debug_log.open("F117_GLimiter_Debug.csv");
    }
    if (debug_log.is_open())
    {
        write_pitch_debug_header();
    }
}

void close_pitch_debug_log()
{
    log_counter = 0;
    if (debug_log.is_open())
    {
        debug_log.close();
    }
}

void log_pitch_debug_sample(const Pitch_debug_sample& s)
{
    ++log_counter;
    if (!debug_log.is_open() || (log_counter % pitch_debug_log_interval_frames) != 0)
    {
        return;
    }

    const Roll_snapshot& roll = debug_snapshot.roll;
    const Yaw_snapshot& yaw = debug_snapshot.yaw;
    const Aero_snapshot& aero = debug_snapshot.aero;

    debug_log << s.roll_angle_deg << "," << s.pitch_angle_deg << "," << s.roll_rate_deg_s << "," << roll.stick_input
              << "," << roll.stick_force << "," << roll.rate_command << "," << roll.flat_turn_command << ","
              << roll.rate_command_filtered << "," << roll.command_gained << "," << roll.surface_command << ","
              << yaw.ped_input << "," << yaw.beta_deg << "," << yaw.rate_rps << "," << yaw.rate_deg_s << ","
              << yaw.rudder_command << "," << yaw.rudder_command_filtered << "," << yaw.pedal_command << ","
              << yaw.damping << "," << yaw.side_accel_feedback << "," << yaw.ari_command << "," << yaw.combined_command
              << "," << yaw.surface_command << "," << yaw.rudder_deg_commanded << "," << yaw.rudder_deg << ","
              << yaw.rudder_pct << "," << aero.cy_delta_rudder << "," << aero.cn_delta_rudder << ","
              << aero.cl_delta_rudder << "," << aero.cn_delta_beta << "," << aero.cl_delta_beta << "," << aero.cy_total
              << "," << aero.cn_total << "," << aero.cl_total << "," << s.raw_az << "," << s.gravity_component << ","
              << s.nz_measured << "," << s.nz_reference << "," << s.stick_command_pos << "," << s.nz_command << ","
              << s.nz_error << "," << s.nz_proportional << "," << s.pitch_rate_blend << "," << s.nz_control << ","
              << s.integrator_out << "," << s.raw_ao_a << "," << s.filtered_ao_a << "," << s.raw_stick_input << ","
              << s.pitch_rate << "," << s.dyn_pressure_pa << "," << s.velocity_mps << "," << s.mach << "," << s.thrust_n
              << "," << s.cx_total << "," << s.elevator_out << "," << ground_contact_fade << "\n";
}

}  // namespace Detail
}  // namespace Flight_controls
}  // namespace F117
