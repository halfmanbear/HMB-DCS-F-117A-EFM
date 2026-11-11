#pragma once
// CSV debug log of the pitch control law (F117_GLimiter_Debug.csv in the DCS Logs folder).

namespace F117 { namespace FLIGHTCONTROLS { namespace detail {

struct PitchDebugSample
{
    double rollAngleDeg{};
    double pitchAngleDeg{};
    double rollRateDegS{};
    double rawAz{};
    double gravityComponent{};
    double nzMeasured{};
    double nzReference{};
    double stickCommandPos{};
    double nzCommand{};
    double nzError{};
    double nzProportional{};
    double pitchRateBlend{};     // washed pitch-rate blend contribution (g) in the error signal
    double nzControl{};
    double integratorOut{};
    double rawAoA{};
    double filteredAoA{};
    double rawStickInput{};
    double pitchRate{};
    double dynPressure_PA{};
    double velocity_MPS{};
    double mach{};
    double thrust_N{};
    double cxTotal{};
    double elevatorOut{};
};

void open_pitch_debug_log();
void close_pitch_debug_log();
// Writes one row every kPitchDebugLogIntervalFrames calls when the log is open.
void log_pitch_debug_sample(const PitchDebugSample& sample);

}}} // namespace F117::FLIGHTCONTROLS::detail
