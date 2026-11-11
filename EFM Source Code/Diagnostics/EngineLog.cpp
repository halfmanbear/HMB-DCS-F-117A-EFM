#include "stdafx.h"
#include "EngineLog.h"

#include <fstream>
#include <string>

#include "Paths.h"
#include "../FlightModel/State.h"

namespace
{
constexpr int engine_log_interval_frames = 100;

std::ofstream engine_debug_log;
int engine_log_counter = 0;
bool engine_log_initialized = false;

void open_engine_debug_log()
{
    const std::string saved_games_log_path = build_saved_games_dcs_path("DCS", "Logs\\F117_Engine_Debug.csv");
    if (!saved_games_log_path.empty())
    {
        engine_debug_log.open(saved_games_log_path);
    }
    if (!engine_debug_log.is_open())
    {
        const std::string open_beta_log_path =
            build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Engine_Debug.csv");
        if (!open_beta_log_path.empty())
        {
            engine_debug_log.open(open_beta_log_path);
        }
    }
    if (!engine_debug_log.is_open())
    {
        engine_debug_log.open("F117_Engine_Debug.csv");
    }
    if (engine_debug_log.is_open())
    {
        engine_debug_log
            << "engine1_thrust_N,engine2_thrust_N,total_thrust_N,throttle_pct,N2_rpm_pct,mach,velocity_mps,altitude_m,"
            << "pedInput,rudder_DEG_cmd,rudder_DEG,rudder_PCT,rudder_pos,beta_DEG,yawRate_RPS,yawTrim,aileron_DEG,"
               "aileron_PCT\n";
    }
}
}  // namespace

void log_engine_debug_sample()
{
    if (!engine_log_initialized)
    {
        open_engine_debug_log();
        engine_log_initialized = true;
    }

    engine_log_counter++;
    if (engine_debug_log.is_open() && (engine_log_counter % engine_log_interval_frames == 0))
    {
        engine_debug_log << F117::engine_thrust_n[0] << "," << F117::engine_thrust_n[1] << "," << F117::thrust_n << ","
                         << F117::throttle_input << "," << 0.5 * (F117::engines[0].n2 + F117::engines[1].n2) << ","
                         << F117::mach << "," << F117::total_velocity_mps << "," << F117::altitude_m << ","
                         << F117::ped_input << "," << F117::rudder_deg_commanded << "," << F117::rudder_deg << ","
                         << F117::rudder_pct << "," << F117::rudder_pos << "," << F117::beta_deg << ","
                         << F117::yaw_rate_rps << "," << F117::yaw_trim << "," << F117::aileron_deg << ","
                         << F117::aileron_pct << "\n";
        engine_debug_log.flush();
    }
}
