#include "stdafx.h"
#include "PitchLog.h"

#include <fstream>
#include <string>

#include "Internal.h"
#include "../Diagnostics/Paths.h"

namespace F117 { namespace FLIGHTCONTROLS { namespace detail {

namespace
{
    std::ofstream debugLog;
    int logCounter = 0;

    void write_pitch_debug_header()
    {
        // Columns: pitch_mode=0 always in flight (NormalAcceleration P+I law).
        // pitch_rate_blend = washed pitch-rate contribution (g) blended into the Nz error signal.
        debugLog << "roll_deg,pitch_deg,roll_rate_dps,roll_stick,roll_stick_force,roll_rate_cmd,roll_flat_turn_cmd,"
                    "roll_rate_cmd_filt,roll_cmd_surface,roll_surface_out,yaw_ped_input,beta_deg,yaw_rate_rps,"
                    "yaw_rate_dps,yaw_rudder_cmd,yaw_rudder_cmd_filt,yaw_pedal_cmd,yaw_damping,yaw_side_accel,"
                    "yaw_ari_cmd,yaw_combined_cmd,yaw_surface_out,rudder_deg_cmd,rudder_deg,rudder_pct,aero_cy_dr,"
                    "aero_cn_dr,aero_cl_dr,aero_cn_dbeta,aero_cl_dbeta,aero_cy_total,aero_cn_total,aero_cl_total,"
                    "az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,nz_proportional,"
                    "pitch_rate_blend,nz_control,integratorOut,alpha_raw,alphaFiltered,stickInput,pitchRate,"
                    "dynPressure_pa,velocity_mps,mach,thrust_N,Cx_total,elevatorOut,ground_contact_fade\n";
    }
}

void open_pitch_debug_log()
{
    logCounter = 0;
    if (debugLog.is_open())
    {
        debugLog.close();
    }

    const std::string dcsLogPath = build_saved_games_dcs_log_path("DCS", "F117_GLimiter_Debug.csv");
    if (!dcsLogPath.empty())
    {
        debugLog.open(dcsLogPath);
    }
    if (!debugLog.is_open())
    {
        const std::string openBetaLogPath = build_saved_games_dcs_log_path("DCS.openbeta", "F117_GLimiter_Debug.csv");
        if (!openBetaLogPath.empty())
        {
            debugLog.open(openBetaLogPath);
        }
    }
    if (!debugLog.is_open())
    {
        debugLog.open("F117_GLimiter_Debug.csv");
    }
    if (debugLog.is_open())
    {
        write_pitch_debug_header();
    }
}

void close_pitch_debug_log()
{
    logCounter = 0;
    if (debugLog.is_open())
    {
        debugLog.close();
    }
}

void log_pitch_debug_sample(const PitchDebugSample& s)
{
    ++logCounter;
    if (!debugLog.is_open() || (logCounter % kPitchDebugLogIntervalFrames) != 0)
    {
        return;
    }

    const RollSnapshot& roll = debug_snapshot.roll;
    const YawSnapshot& yaw = debug_snapshot.yaw;
    const AeroSnapshot& aero = debug_snapshot.aero;

    debugLog << s.rollAngleDeg << ","
             << s.pitchAngleDeg << ","
             << s.rollRateDegS << ","
             << roll.stickInput << ","
             << roll.stickForce << ","
             << roll.rateCommand << ","
             << roll.flatTurnCommand << ","
             << roll.rateCommandFiltered << ","
             << roll.commandGained << ","
             << roll.surfaceCommand << ","
             << yaw.pedInput << ","
             << yaw.betaDeg << ","
             << yaw.rateRps << ","
             << yaw.rateDegS << ","
             << yaw.rudderCommand << ","
             << yaw.rudderCommandFiltered << ","
             << yaw.pedalCommand << ","
             << yaw.damping << ","
             << yaw.sideAccelFeedback << ","
             << yaw.ariCommand << ","
             << yaw.combinedCommand << ","
             << yaw.surfaceCommand << ","
             << yaw.rudderDegCommanded << ","
             << yaw.rudderDeg << ","
             << yaw.rudderPct << ","
             << aero.cyDeltaRudder << ","
             << aero.cnDeltaRudder << ","
             << aero.clDeltaRudder << ","
             << aero.cnDeltaBeta << ","
             << aero.clDeltaBeta << ","
             << aero.cyTotal << ","
             << aero.cnTotal << ","
             << aero.clTotal << ","
             << s.rawAz << ","
             << s.gravityComponent << ","
             << s.nzMeasured << ","
             << s.nzReference << ","
             << s.stickCommandPos << ","
             << s.nzCommand << ","
             << s.nzError << ","
             << s.nzProportional << ","
             << s.pitchRateBlend << ","
             << s.nzControl << ","
             << s.integratorOut << ","
             << s.rawAoA << ","
             << s.filteredAoA << ","
             << s.rawStickInput << ","
             << s.pitchRate << ","
             << s.dynPressure_PA << ","
             << s.velocity_MPS << ","
             << s.mach << ","
             << s.thrust_N << ","
             << s.cxTotal << ","
             << s.elevatorOut << ","
             << ground_contact_fade << "\n";
}

}}} // namespace F117::FLIGHTCONTROLS::detail
