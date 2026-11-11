#include "stdafx.h"
#include "EngineLog.h"

#include <fstream>
#include <string>

#include "Paths.h"
#include "../FlightModel/State.h"

namespace
{
constexpr int kEngineLogIntervalFrames = 100;

std::ofstream engineDebugLog;
int engineLogCounter = 0;
bool engineLogInitialized = false;

void open_engine_debug_log()
{
    const std::string savedGamesLogPath = build_saved_games_dcs_path("DCS", "Logs\\F117_Engine_Debug.csv");
    if (!savedGamesLogPath.empty())
    {
        engineDebugLog.open(savedGamesLogPath);
    }
    if (!engineDebugLog.is_open())
    {
        const std::string openBetaLogPath = build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Engine_Debug.csv");
        if (!openBetaLogPath.empty())
        {
            engineDebugLog.open(openBetaLogPath);
        }
    }
    if (!engineDebugLog.is_open())
    {
        engineDebugLog.open("F117_Engine_Debug.csv");
    }
    if (engineDebugLog.is_open())
    {
        engineDebugLog
            << "engine1_thrust_N,engine2_thrust_N,total_thrust_N,throttle_pct,N2_rpm_pct,mach,velocity_mps,altitude_m,"
            << "pedInput,rudder_DEG_cmd,rudder_DEG,rudder_PCT,rudder_pos,beta_DEG,yawRate_RPS,yawTrim,aileron_DEG,"
               "aileron_PCT\n";
    }
}
}  // namespace

void log_engine_debug_sample()
{
    if (!engineLogInitialized)
    {
        open_engine_debug_log();
        engineLogInitialized = true;
    }

    engineLogCounter++;
    if (engineDebugLog.is_open() && (engineLogCounter % kEngineLogIntervalFrames == 0))
    {
        engineDebugLog << F117::engine_thrust_N[0] << "," << F117::engine_thrust_N[1] << "," << F117::thrust_N << ","
                       << F117::throttleInput << "," << 0.5 * (F117::engines[0].N2 + F117::engines[1].N2) << ","
                       << F117::mach << "," << F117::totalVelocity_MPS << "," << F117::altitude_m << ","
                       << F117::pedInput << "," << F117::rudder_DEG_commanded << "," << F117::rudder_DEG << ","
                       << F117::rudder_PCT << "," << F117::rudder_pos << "," << F117::beta_DEG << ","
                       << F117::yawRate_RPS << "," << F117::yawTrim << "," << F117::aileron_DEG << ","
                       << F117::aileron_PCT << "\n";
        engineDebugLog.flush();
    }
}
