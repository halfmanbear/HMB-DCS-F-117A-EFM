#include "stdafx.h"
#include "FlightControls.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <shlobj.h>

#include "../include/general_filter.h"

#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

double limit(double input, double lower_limit, double upper_limit);

namespace F117
{
    namespace FLIGHTCONTROLS
    {
        bool simInitialized = false;
        double longStickInput = 0.0;
        double alphaFiltered = 0.0;
        double longStickForce = 0.0;
        double latStickInput = 0.0;

        namespace
        {
            std::ofstream debugLog;
            int logCounter = 0;

            GeneralFilter pitchActuatorDynamicsFilter;
            GeneralFilter latStickForceFilter;
            GeneralFilter rollCommandFilter;
            GeneralFilter rollActuatorDynamicsFilter;
            GeneralFilter rollRateFilter1;
            GeneralFilter rollRateFilter2;
            GeneralFilter rudderCommandFilter;
            GeneralFilter yawRateWashout;
            GeneralFilter yawRateFilter;
            GeneralFilter yawServoFilter;

            double stickCommandPosFiltered = 0.0;
            double azFiltered = 0.0;
            double nz_reference = 1.0;  // current 1G reference; reset alongside filter state
            double pitchIntegratorState = 0.0;  // tracks raw integrator output for state anti-windup

            bool yawControllerInitialized = false;
            bool pitchControllerInitialized = false;
            bool rollControllerInitialized = false;

            double yawControlAccumulator = 0.0;
            double pitchControlAccumulator = 0.0;
            double rollControlAccumulator = 0.0;

            double yawHeldOutput = 0.0;
            double pitchHeldOutput = 0.0;
            double rollHeldOutput = 0.0;
            double lastRollStickInput = 0.0;
            double lastRollStickForce = 0.0;
            double lastRollRateCommand = 0.0;
            double lastRollFlatTurnCommand = 0.0;
            double lastRollRateCommandFiltered = 0.0;
            double lastRollCommandGained = 0.0;
            double lastRollSurfaceCommand = 0.0;
            double lastYawPedInput = 0.0;
            double lastYawRateDegS = 0.0;
            double lastYawRudderCommand = 0.0;
            double lastYawRudderCommandFiltered = 0.0;
            double lastYawPedalCommand = 0.0;
            double lastYawDamping = 0.0;
            double lastYawSideAccelFeedback = 0.0;
            double lastYawAriCommand = 0.0;
            double lastYawCombinedCommand = 0.0;
            double lastYawSurfaceCommand = 0.0;
            double lastYawBetaDeg = 0.0;
            double lastYawRateRps = 0.0;
            double lastYawRudderDegCommanded = 0.0;
            double lastYawRudderDeg = 0.0;
            double lastYawRudderPct = 0.0;
            double lastAeroCyDeltaRudder = 0.0;
            double lastAeroCnDeltaRudder = 0.0;
            double lastAeroClDeltaRudder = 0.0;
            double lastAeroCnDeltaBeta = 0.0;
            double lastAeroClDeltaBeta = 0.0;
            double lastAeroCyTotal = 0.0;
            double lastAeroCnTotal = 0.0;
            double lastAeroClTotal = 0.0;
            double pitchModeGearDown = 0.0;
            bool pitchModeAirRefuelDoorOpen = false;

            constexpr double kPi = 3.14159265358979323846;
            constexpr double kFixedControlTimeStep = 0.005;
            constexpr int kMaxControlSubstepsPerFrame = 20;
            constexpr double kMaxAccumulatedControlTime = kFixedControlTimeStep * kMaxControlSubstepsPerFrame;
            constexpr int kPitchDebugLogIntervalFrames = 10;

            enum class PitchCommandMode
            {
                NormalAcceleration = 0,
                PitchRate = 1,
                Automatic = 2
            };

            namespace YawControllerConfig
            {
                constexpr double kPedalForceScale = 450.0;
                constexpr double kPedalDeadbandForce = 44.0;
                constexpr double kPedalCommandSlope = -0.0739;
                constexpr double kPedalCommandOffset = 3.2512;
                // Internal rudder command limit in real surface degrees.
                // Higher = more commanded yaw authority before saturation; lower = flatter, less clipped full-pedal behavior.
                constexpr double kRudderLimitDeg = 15.0;
                constexpr double kYawAlphaCouplingGain = 5.0 / 57.3;
                // Yaw-rate damping gain. Higher = stronger rate opposition and less oscillation;
                // lower = more free yaw response, but more rebound near the pedal stops.
                constexpr double kYawDampingGain = 1.5;
                // Side-acceleration feedback gain. Higher = more auto-centering / coordination;
                // lower = easier steady flat turns on rudder alone, but less self-correction.
                constexpr double kSideAccelFeedbackGain = 0.25;
                constexpr double kAriAoAGain = 0.05;
                constexpr double kAriLimit = 1.5;
                // Pedal-command filter rate. Higher = rudder command reaches and leaves the target faster;
                // lower = smoother feel, but more lag in pedal response.
                constexpr double kCommandFilterRate = 4.0;
                constexpr double kWashoutTauInverse = 1.0;
                constexpr double kYawLeadLagNumerator0 = 3.0;
                constexpr double kYawLeadLagPole = 15.0;
                // Yaw servo natural frequency. Higher = quicker rudder bite; lower = softer onset and less tendency to overshoot.
                constexpr double kServoNaturalFrequency = 52.0;
                // Yaw servo damping ratio. Higher = flatter response and less rebound near max yaw;
                // lower = snappier response, but more oscillation/ring.
                constexpr double kServoDampingRatio = 0.95;
            }

            namespace PitchControllerConfig
            {
                // Pitch-law selector:
                // - Automatic: PitchRate when gear is down or AAR door is open; otherwise NormalAcceleration
                // - NormalAcceleration: neutral stick seeks trimmed Nz (current F-16-like behavior)
                // - PitchRate: neutral stick seeks zero pitch rate for comparison testing
                constexpr PitchCommandMode kCommandMode = PitchCommandMode::Automatic;
                constexpr double kPitchRateModeGearThreshold = 0.1;
                constexpr double kPositiveStickForceScale = 80.0;
                constexpr double kNegativeStickForceScale = 180.0;
                constexpr double kStickForceMin = -180.0;
                constexpr double kStickForceMax = 80.0;
                constexpr double kStickCommandDeadband = 8.0;
                constexpr double kStickCommandBreakpoint = 33.0;
                constexpr double kNzCommandMin = -4.0;
                constexpr double kNzCommandMax = 8.0;
                // Integrator strength on Nz error. Higher = more steady-state authority and more stored pull;
                // lower = less overshoot/carry-through, but slower trim-in to the commanded G.
                constexpr double kPitchIntegratorGain = 0.75;
                // Extra multiplier used only when the integrator is unwinding against the current error.
                // Higher = faster dump of stored pull/push and less overshoot; lower = smoother, but more carry-through.
                constexpr double kPitchIntegratorUnwindGain = 4.0;
                constexpr double kAoAFilterRate = 35.0;
                constexpr double kNzMax = 6.3;
                constexpr double kNzMin = -3.0;
                constexpr double kNzStickDeadband = 0.05;
                constexpr double kNzReferenceTrackRate = 2.0;
                // Pitch-rate damping gain. More negative = stronger damping and less transient overshoot;
                // less negative = sharper response, but more oscillation/overshoot.
                constexpr double kPitchDampingGain = -6.5;
                // Proportional gain on Nz error. Higher = quicker initial pull and tighter tracking;
                // lower = softer response and less transient overshoot.
                constexpr double kNzErrorGain = 1.7;
                constexpr double kPitchIntegratorLimit = 25.0;
                constexpr double kAoAMax = 20.0;
                constexpr double kAoALimitStart = 19.0;
                constexpr double kAoAPushdownGain = 0.30; //0.3
                constexpr double kPitchServoNaturalFrequency = 52.0; // aoa vapor testing was 52
                constexpr double kPitchServoDampingRatio = 2.5; //0.7

                // Pitch-rate command schedule used only when kCommandMode == PitchRate.
                // Higher values = more rate-command feel and less "return to 1G" on stick release.
                constexpr double kPitchRateCommandMinDegS = -20.0;
                constexpr double kPitchRateCommandAtNegForceBreakDegS = -3.0;
                constexpr double kPitchRateCommandAtPosForceBreakDegS = 3.0;
                constexpr double kPitchRateCommandMaxDegS = 18.0;
                // Proportional gain on pitch-rate error (rad/s). Higher = crisper rate capture;
                // lower = softer rate response but more lag.
                constexpr double kPitchRateErrorGain = 18.0;
                // Integrator gain for pitch-rate mode. Higher = better steady tracking of commanded rate;
                // lower = less carry-through and less risk of fighting the pilot on release.
                constexpr double kPitchRateIntegratorGain = 1.5;
                constexpr double kPitchRateIntegratorLimit = 25.0;
                // Outer-loop protection added only in PitchRate mode so the test law still respects
                // the same Nz/AoA envelope. Higher = stronger automatic unload when the rate law
                // approaches or drives through the limiter; lower = freer pitch-rate feel, but weaker protection.
                constexpr double kPitchRateNzLimitStartPositive = 5.7;
                constexpr double kPitchRateNzLimitStartNegative = -2.4;
                constexpr double kPitchRateNzLimiterGainDegSPerG = 12.0;
                constexpr double kPitchRateAoALimiterGainDegSPerDeg = 6.0;
                // Direct damping added only in PitchRate mode. More negative = less oscillation and
                // less limiter hunting; less negative = freer rate response, but more overshoot.
                constexpr double kPitchRateDampingGain = -3.0;
                // Supervisory Nz protection added directly to the final command in PitchRate mode.
                // Higher = firmer G limiting; lower = more pure rate-command feel with more breach risk.
                constexpr double kPitchRateNzProtectionGain = 5.0;

                // Gain on the gravity feed-forward that updates nz_reference each frame.
                // Higher = nz_reference follows rapid bank/pitch changes more aggressively;
                // lower = less brief unload/pitch-down during fast roll recoveries, but more gravity-comp lag.
                constexpr double kGravityFeedForwardGain = 1.0;

                // When the stick is in the deadband and nz_reference is BELOW the gravity component
                // (e.g. after being clamped at kNzMin by the feed-forward during an aggressive push),
                // snap back up at this faster rate so nz_cmd recovers immediately on stick release
                // rather than lagging behind for ~0.5 s at the -3 G floor.
                constexpr double kNzReferenceSnapRate = 20.0;

                // Force-to-Nz schedule: target G commands at the piecewise-linear breakpoints.
                // Changing a breakpoint force (above) or a target G (here) automatically
                // keeps all derived slopes/intercepts consistent.
                constexpr double kNzAtNegForceBreak = -0.4;  // Nz at –kStickCommandBreakpoint lbf (push)
                constexpr double kNzAtPosForceBreak =  0.4;  // Nz at +kStickCommandBreakpoint lbf (pull) //0.8
                constexpr double kNzAtMaxPullForce  =  4.0;  // Nz at +kStickForceMax lbf (pull)

                // Push (negative): segment 1 – deadband to first breakpoint
                constexpr double kNzSlopeNeg1     = kNzAtNegForceBreak / (-kStickCommandBreakpoint + kStickCommandDeadband);
                constexpr double kNzInterceptNeg1 = kNzSlopeNeg1 * kStickCommandDeadband;
                // Push (negative): segment 2 – beyond first breakpoint (steeper empirical ramp)
                constexpr double kNzSlopeNeg2     = 0.067;
                constexpr double kNzInterceptNeg2 = kNzAtNegForceBreak + kNzSlopeNeg2 * kStickCommandBreakpoint;
                // Pull (positive): segment 1 – deadband to first breakpoint
                constexpr double kNzSlopePos1     = kNzAtPosForceBreak / (kStickCommandBreakpoint - kStickCommandDeadband);
                constexpr double kNzInterceptPos1 = -kNzSlopePos1 * kStickCommandDeadband;
                // Pull (positive): segment 2 – first breakpoint to max force
                constexpr double kNzSlopePos2     = (kNzAtMaxPullForce - kNzAtPosForceBreak) / (kStickForceMax - kStickCommandBreakpoint);
                constexpr double kNzInterceptPos2 = kNzAtPosForceBreak - kNzSlopePos2 * kStickCommandBreakpoint;
            }

            namespace RollControllerConfig
            {
                constexpr double kLatStickForceScale = 75.0;
                // Lateral-acceleration bias gain used to shape the roll command in loaded flight.
                // Higher = more roll command added from ay; lower = cleaner stick-recenter stop, but less coupled feel under load.
                constexpr double kAyBiasGain = 8.9;
                // Lateral-stick magnitude where the ay bias reaches full strength.
                // Lower = bias comes in sooner; higher = bias fades out more around center stick, improving roll stop after release.
                constexpr double kAyBiasFullStick = 0.35;
                // Beta-to-roll-rate interconnect used only in rudder-held, near-center-stick flat turns.
                // Higher = more aileron help to resist the inside wing dropping during yaw-only turns;
                // lower = more natural bank build-up from the airframe with less FCS help.
                constexpr double kFlatTurnBetaGain = 2.5;
                // Bank-angle hold term paired with the beta interconnect above.
                // Higher = stronger wings-level hold in a flat turn; lower = less obvious automatic roll correction.
                constexpr double kFlatTurnBankGain = 3.0;
                // Lateral-stick fade point for the flat-turn interconnect.
                // Lower = pilot roll input overrides the helper sooner; higher = helper remains active farther off-center.
                constexpr double kFlatTurnLatStickFade = 0.2;
                // Pedal magnitude where the flat-turn interconnect reaches full strength.
                // Lower = more help with small rudder inputs; higher = helper mainly appears at strong pedal.
                constexpr double kFlatTurnPedalFull = 0.6;
                constexpr double kFlatTurnCommandLimit = 12.0;
                constexpr double kLongStickFeelGain = 0.0667;
                constexpr double kRollFeelForceThreshold = 25.0;
                constexpr double kRollFeelFixedGain = 0.7;
                constexpr double kRollFeelSlope = 0.012;
                constexpr double kStickDeadband = 3.0;
                constexpr double kFirstBreakpoint = 25.0;
                constexpr double kSecondBreakpoint = 46.0;
                constexpr double kPressureLow = 19153.0;
                constexpr double kPressureHigh = 23941.0;
                constexpr double kPressureLowGain = 0.2;
                constexpr double kPressureSlope = -0.00002089;
                constexpr double kPressureOffset = 0.6;
                constexpr double kPressureHighGain = 0.1;
                constexpr double kRollCommandLimit = 21.5;
                constexpr double kLatForceFilterPole = 60.0;
                // Roll-command filter pole. Higher = commanded roll rate recenters faster when the
                // stick is released, so roll inertia stops sooner; lower = smoother but more carry-through.
                constexpr double kRollCommandFilterPole = 20.0;
                // Roll actuator natural frequency. Higher = more immediate roll surface response;
                // lower = softer initial bite and less chance of exciting the airframe.
                constexpr double kRollServoNaturalFrequency = 52.0;
                // Roll actuator damping ratio. Higher = less overshoot/oscillation in roll stop;
                // lower = snappier response, but more tendency to ring.
                constexpr double kRollServoDampingRatio = 0.85;
                // Measured roll-rate filter pole. Higher = less lag in rate feedback and quicker stopping;
                // lower = smoother/noisier rejection tradeoff, but more phase lag.
                constexpr double kRollRateFilterPole = 50.0;
                constexpr double kRollRateFilter2Num0 = 4.0;
                constexpr double kRollRateFilter2Num1 = 64.0;
                constexpr double kRollRateFilter2Den1 = 80.0;
                constexpr double kRollRateFilter2OmegaSquared = 6400.0;
                constexpr double kDynPressureLbFt2ToNm2 = 47.880258889;

                // Stick force-to-roll-rate schedule: target roll rate commands (deg/s)
                // at each piecewise-linear breakpoint.  Adjusting a breakpoint force or
                // a target rate here keeps all derived slopes/intercepts consistent.
                constexpr double kRollRateAtBreak1 = 20.0;  // deg/s at kFirstBreakpoint lbf
                constexpr double kRollRateAtBreak2 = 80.0;  // deg/s at kSecondBreakpoint lbf

                // Segment slopes and intercepts.  The schedule is symmetric: positive
                // side uses +intercept, negative side uses –intercept.
                constexpr double kRollSlopeSeg1     = kRollRateAtBreak1 / (kFirstBreakpoint - kStickDeadband);
                constexpr double kRollInterceptSeg1 = -kRollSlopeSeg1 * kStickDeadband;
                constexpr double kRollSlopeSeg2     = (kRollRateAtBreak2 - kRollRateAtBreak1) / (kSecondBreakpoint - kFirstBreakpoint);
                constexpr double kRollInterceptSeg2 = kRollRateAtBreak1 - kRollSlopeSeg2 * kFirstBreakpoint;
                // Segment 3: empirical steeper ramp beyond kSecondBreakpoint
                constexpr double kRollSlopeSeg3     = 7.5862;
                constexpr double kRollInterceptSeg3 = kRollRateAtBreak2 - kRollSlopeSeg3 * kSecondBreakpoint;
            }

            inline bool yaw_filters_need_init()
            {
                return !yawControllerInitialized;
            }

            inline bool pitch_filters_need_init()
            {
                return !pitchControllerInitialized;
            }

            inline bool roll_filters_need_init()
            {
                return !rollControllerInitialized;
            }

            inline void advance_control_accumulator(double& accumulator, double frameTime_SEC)
            {
                if (frameTime_SEC <= 0.0)
                {
                    return;
                }

                accumulator += frameTime_SEC;
                if (accumulator > kMaxAccumulatedControlTime)
                {
                    accumulator = kMaxAccumulatedControlTime;
                }
            }

            PitchCommandMode resolve_pitch_command_mode()
            {
                if (PitchControllerConfig::kCommandMode != PitchCommandMode::Automatic)
                {
                    return PitchControllerConfig::kCommandMode;
                }

                return (pitchModeGearDown > PitchControllerConfig::kPitchRateModeGearThreshold || pitchModeAirRefuelDoorOpen)
                    ? PitchCommandMode::PitchRate
                    : PitchCommandMode::NormalAcceleration;
            }

            std::string narrow_from_wide(const std::wstring& wide)
            {
                if (wide.empty())
                {
                    return std::string();
                }

                const int requiredChars = WideCharToMultiByte(
                    CP_ACP,
                    0,
                    wide.c_str(),
                    -1,
                    nullptr,
                    0,
                    nullptr,
                    nullptr);
                if (requiredChars <= 1)
                {
                    return std::string();
                }

                std::vector<char> buffer(requiredChars);
                WideCharToMultiByte(
                    CP_ACP,
                    0,
                    wide.c_str(),
                    -1,
                    buffer.data(),
                    requiredChars,
                    nullptr,
                    nullptr);
                return std::string(buffer.data());
            }

            std::string get_saved_games_root_path()
            {
                PWSTR savedGamesPath = nullptr;
                if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_SavedGames, KF_FLAG_DEFAULT, nullptr, &savedGamesPath)))
                {
                    const std::wstring widePath(savedGamesPath);
                    CoTaskMemFree(savedGamesPath);
                    const std::string narrowPath = narrow_from_wide(widePath);
                    if (!narrowPath.empty())
                    {
                        return narrowPath;
                    }
                }

                const char* userProfile = getenv("USERPROFILE");
                if (userProfile != nullptr)
                {
                    return std::string(userProfile) + "\\Saved Games";
                }

                return std::string();
            }

            std::string build_saved_games_dcs_log_path(const char* dcsFolderName, const char* fileName)
            {
                const std::string savedGamesRoot = get_saved_games_root_path();
                if (savedGamesRoot.empty())
                {
                    return std::string();
                }

                return savedGamesRoot + "\\" + dcsFolderName + "\\Logs\\" + fileName;
            }

            void write_pitch_debug_header()
            {
                debugLog << "roll_deg,pitch_deg,roll_rate,roll_stick,roll_stick_force,roll_rate_cmd,roll_flat_turn_cmd,roll_rate_cmd_filt,roll_cmd_surface,roll_surface_out,yaw_ped_input,beta_deg,yaw_rate_rps,yaw_rate_dps,yaw_rudder_cmd,yaw_rudder_cmd_filt,yaw_pedal_cmd,yaw_damping,yaw_side_accel,yaw_ari_cmd,yaw_combined_cmd,yaw_surface_out,rudder_deg_cmd,rudder_deg,rudder_pct,aero_cy_dr,aero_cn_dr,aero_cl_dr,aero_cn_dbeta,aero_cl_dbeta,aero_cy_total,aero_cn_total,aero_cl_total,pitch_mode,pitch_mode_cmd,az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,nz_proportional,pitch_damping,nz_control,integratorOut,alpha_raw,alphaFiltered,stickInput,pitchRate,dynPressure,velocity_fps,mach,thrust_N,Cx_total,elevatorOut\n";
            }

            void open_pitch_debug_log()
            {
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

            void log_pitch_debug_sample(
                double rollAngleDeg,
                double pitchAngleDeg,
                double rollRateDegS,
                double pitchMode,
                double pitchModeCommand,
                double rawAz,
                double gravityComponent,
                double nzMeasured,
                double nzReference,
                double stickCommandPos,
                double nzCommand,
                double nzError,
                double nzProportional,
                double pitchDamping,
                double nzControl,
                double integratorOut,
                double rawAoA,
                double filteredAoA,
                double rawStickInput,
                double pitchRate,
                double dynamicPressureLbFt2,
                double velocityFps,
                double mach,
                double thrustN,
                double cxTotal,
                double elevatorOut)
            {
                ++logCounter;
                if (!debugLog.is_open() || (logCounter % kPitchDebugLogIntervalFrames) != 0)
                {
                    return;
                }

                debugLog << rollAngleDeg << ","
                         << pitchAngleDeg << ","
                         << rollRateDegS << ","
                         << lastRollStickInput << ","
                         << lastRollStickForce << ","
                         << lastRollRateCommand << ","
                         << lastRollFlatTurnCommand << ","
                         << lastRollRateCommandFiltered << ","
                         << lastRollCommandGained << ","
                         << lastRollSurfaceCommand << ","
                         << lastYawPedInput << ","
                         << lastYawBetaDeg << ","
                         << lastYawRateRps << ","
                         << lastYawRateDegS << ","
                         << lastYawRudderCommand << ","
                         << lastYawRudderCommandFiltered << ","
                         << lastYawPedalCommand << ","
                         << lastYawDamping << ","
                         << lastYawSideAccelFeedback << ","
                         << lastYawAriCommand << ","
                         << lastYawCombinedCommand << ","
                         << lastYawSurfaceCommand << ","
                         << lastYawRudderDegCommanded << ","
                         << lastYawRudderDeg << ","
                         << lastYawRudderPct << ","
                         << lastAeroCyDeltaRudder << ","
                         << lastAeroCnDeltaRudder << ","
                         << lastAeroClDeltaRudder << ","
                         << lastAeroCnDeltaBeta << ","
                         << lastAeroClDeltaBeta << ","
                         << lastAeroCyTotal << ","
                         << lastAeroCnTotal << ","
                         << lastAeroClTotal << ","
                         << pitchMode << ","
                         << pitchModeCommand << ","
                         << rawAz << ","
                         << gravityComponent << ","
                         << nzMeasured << ","
                         << nzReference << ","
                         << stickCommandPos << ","
                         << nzCommand << ","
                         << nzError << ","
                         << nzProportional << ","
                         << pitchDamping << ","
                         << nzControl << ","
                         << integratorOut << ","
                         << rawAoA << ","
                         << filteredAoA << ","
                         << rawStickInput << ","
                         << pitchRate << ","
                         << dynamicPressureLbFt2 << ","
                         << velocityFps << ","
                         << mach << ","
                         << thrustN << ","
                         << cxTotal << ","
                         << elevatorOut << "\n";
            }

            void init_yaw_filters(double dt)
            {
                double numerators[2] = { 0.0, YawControllerConfig::kCommandFilterRate };
                double denominators[2] = { 1.0, YawControllerConfig::kCommandFilterRate };
                rudderCommandFilter.InitFilter(numerators, denominators, 1, dt);

                double numerators1[2] = { 1.0, 0.0 };
                double denominators1[2] = { 1.0, YawControllerConfig::kWashoutTauInverse };
                yawRateWashout.InitFilter(numerators1, denominators1, 1, dt);

                double numerators2[2] = { YawControllerConfig::kYawLeadLagNumerator0, YawControllerConfig::kYawLeadLagPole };
                double denominators2[2] = { 1.0, YawControllerConfig::kYawLeadLagPole };
                yawRateFilter.InitFilter(numerators2, denominators2, 1, dt);

                const double servoOmegaSquared = std::pow(YawControllerConfig::kServoNaturalFrequency, 2.0);
                double numerators3[3] = { 0.0, 0.0, servoOmegaSquared };
                double denominators3[3] = { 1.0, 2.0 * YawControllerConfig::kServoDampingRatio * YawControllerConfig::kServoNaturalFrequency, servoOmegaSquared };
                yawServoFilter.InitFilter(numerators3, denominators3, 2, dt);

                yawControlAccumulator = 0.0;
                yawHeldOutput = 0.0;
                lastYawPedInput = 0.0;
                lastYawRateDegS = 0.0;
                lastYawRudderCommand = 0.0;
                lastYawRudderCommandFiltered = 0.0;
                lastYawPedalCommand = 0.0;
                lastYawDamping = 0.0;
                lastYawSideAccelFeedback = 0.0;
                lastYawAriCommand = 0.0;
                lastYawCombinedCommand = 0.0;
                lastYawSurfaceCommand = 0.0;
                lastYawBetaDeg = 0.0;
                lastYawRateRps = 0.0;
                lastYawRudderDegCommanded = 0.0;
                lastYawRudderDeg = 0.0;
                lastYawRudderPct = 0.0;
                lastAeroCyDeltaRudder = 0.0;
                lastAeroCnDeltaRudder = 0.0;
                lastAeroClDeltaRudder = 0.0;
                lastAeroCnDeltaBeta = 0.0;
                lastAeroClDeltaBeta = 0.0;
                lastAeroCyTotal = 0.0;
                lastAeroCnTotal = 0.0;
                lastAeroClTotal = 0.0;
                yawControllerInitialized = true;
            }

            void init_pitch_filters(double dt)
            {
                const double servoOmegaSquared = std::pow(PitchControllerConfig::kPitchServoNaturalFrequency, 2.0);
                double numerators[3] = { 0.0, 0.0, servoOmegaSquared };
                double denominators[3] = { 1.0, 2.0 * PitchControllerConfig::kPitchServoDampingRatio * PitchControllerConfig::kPitchServoNaturalFrequency, servoOmegaSquared };
                pitchActuatorDynamicsFilter.InitFilter(numerators, denominators, 2, dt);

                stickCommandPosFiltered = 0.0;
                azFiltered = 0.0;
                nz_reference = 1.0;
                pitchIntegratorState = 0.0;
                pitchControlAccumulator = 0.0;
                pitchHeldOutput = 0.0;
                logCounter = 0;
                pitchControllerInitialized = true;
                open_pitch_debug_log();
            }

            void init_roll_filters(double dt)
            {
                double numerators[2] = { 0.0, RollControllerConfig::kLatForceFilterPole };
                double denominators[2] = { 1.0, RollControllerConfig::kLatForceFilterPole };
                latStickForceFilter.InitFilter(numerators, denominators, 1, dt);

                double numerators1[2] = { 0.0, RollControllerConfig::kRollCommandFilterPole };
                double denominators1[2] = { 1.0, RollControllerConfig::kRollCommandFilterPole };
                rollCommandFilter.InitFilter(numerators1, denominators1, 1, dt);

                const double servoOmegaSquared = std::pow(RollControllerConfig::kRollServoNaturalFrequency, 2.0);
                double numerators2[3] = { 0.0, 0.0, servoOmegaSquared };
                double denominators2[3] = { 1.0, 2.0 * RollControllerConfig::kRollServoDampingRatio * RollControllerConfig::kRollServoNaturalFrequency, servoOmegaSquared };
                rollActuatorDynamicsFilter.InitFilter(numerators2, denominators2, 2, dt);

                double numerators3[2] = { 0.0, RollControllerConfig::kRollRateFilterPole };
                double denominators3[2] = { 1.0, RollControllerConfig::kRollRateFilterPole };
                rollRateFilter1.InitFilter(numerators3, denominators3, 1, dt);

                double numerators4[3] = { RollControllerConfig::kRollRateFilter2Num0, RollControllerConfig::kRollRateFilter2Num1, RollControllerConfig::kRollRateFilter2OmegaSquared };
                double denominators4[3] = { 1.0, RollControllerConfig::kRollRateFilter2Den1, RollControllerConfig::kRollRateFilter2OmegaSquared };
                rollRateFilter2.InitFilter(numerators4, denominators4, 2, dt);

                rollControlAccumulator = 0.0;
                rollHeldOutput = 0.0;
                lastRollStickInput = 0.0;
                lastRollStickForce = 0.0;
                lastRollRateCommand = 0.0;
                lastRollFlatTurnCommand = 0.0;
                lastRollRateCommandFiltered = 0.0;
                lastRollCommandGained = 0.0;
                lastRollSurfaceCommand = 0.0;
                rollControllerInitialized = true;
            }

        }

        void reset_runtime_state()
        {
            simInitialized = false;
            alphaFiltered = 0.0;
            longStickForce = 0.0;

            stickCommandPosFiltered = 0.0;
            azFiltered = 0.0;
            nz_reference = 1.0;
            pitchIntegratorState = 0.0;

            yawControllerInitialized = false;
            pitchControllerInitialized = false;
            rollControllerInitialized = false;

            yawControlAccumulator = 0.0;
            pitchControlAccumulator = 0.0;
            rollControlAccumulator = 0.0;

            yawHeldOutput = 0.0;
            pitchHeldOutput = 0.0;
            rollHeldOutput = 0.0;
            lastYawPedInput = 0.0;
            lastYawRateDegS = 0.0;
            lastYawRudderCommand = 0.0;
            lastYawRudderCommandFiltered = 0.0;
            lastYawPedalCommand = 0.0;
            lastYawDamping = 0.0;
            lastYawSideAccelFeedback = 0.0;
            lastYawAriCommand = 0.0;
            lastYawCombinedCommand = 0.0;
            lastYawSurfaceCommand = 0.0;
            lastYawBetaDeg = 0.0;
            lastYawRateRps = 0.0;
            lastYawRudderDegCommanded = 0.0;
            lastYawRudderDeg = 0.0;
            lastYawRudderPct = 0.0;
            lastAeroCyDeltaRudder = 0.0;
            lastAeroCnDeltaRudder = 0.0;
            lastAeroClDeltaRudder = 0.0;
            lastAeroCnDeltaBeta = 0.0;
            lastAeroClDeltaBeta = 0.0;
            lastAeroCyTotal = 0.0;
            lastAeroCnTotal = 0.0;
            lastAeroClTotal = 0.0;
            pitchModeGearDown = 0.0;
            pitchModeAirRefuelDoorOpen = false;
            lastRollStickInput = 0.0;
            lastRollStickForce = 0.0;
            lastRollRateCommand = 0.0;
            lastRollFlatTurnCommand = 0.0;
            lastRollRateCommandFiltered = 0.0;
            lastRollCommandGained = 0.0;
            lastRollSurfaceCommand = 0.0;

            logCounter = 0;
            if (debugLog.is_open())
            {
                debugLog.close();
            }
        }

        void update_pitch_mode_auto_inputs(double gearDown, bool airRefuelDoorOpen)
        {
            pitchModeGearDown = gearDown;
            pitchModeAirRefuelDoorOpen = airRefuelDoorOpen;
        }

        void update_yaw_debug_snapshot(double betaDeg, double yawRateRps, double rudderDegCommanded, double rudderDeg, double rudderPct)
        {
            lastYawBetaDeg = betaDeg;
            lastYawRateRps = yawRateRps;
            lastYawRudderDegCommanded = rudderDegCommanded;
            lastYawRudderDeg = rudderDeg;
            lastYawRudderPct = rudderPct;
        }

        void update_aero_debug_snapshot(double cyDeltaRudder, double cnDeltaRudder, double clDeltaRudder, double cnDeltaBeta, double clDeltaBeta, double cyTotal, double cnTotal, double clTotal)
        {
            lastAeroCyDeltaRudder = cyDeltaRudder;
            lastAeroCnDeltaRudder = cnDeltaRudder;
            lastAeroClDeltaRudder = clDeltaRudder;
            lastAeroCnDeltaBeta = cnDeltaBeta;
            lastAeroClDeltaBeta = clDeltaBeta;
            lastAeroCyTotal = cyTotal;
            lastAeroCnTotal = cnTotal;
            lastAeroClTotal = clTotal;
        }

        double fcs_yaw_controller_step(double pedInput, double pedTrim, double yaw_rate, double roll_rate, double aoa_filtered, double aileron_commanded, double ay, double dt, bool resetFilters)
        {
            double rudderForceCommand = pedInput * YawControllerConfig::kPedalForceScale;
            double rudderCommand = 0.0;
            if (std::abs(rudderForceCommand) < YawControllerConfig::kPedalDeadbandForce)
            {
                rudderCommand = 0.0;
            }
            else if (rudderForceCommand >= YawControllerConfig::kPedalDeadbandForce)
            {
                rudderCommand = YawControllerConfig::kPedalCommandSlope * rudderForceCommand + YawControllerConfig::kPedalCommandOffset;
            }
            else if (rudderForceCommand <= -YawControllerConfig::kPedalDeadbandForce)
            {
                rudderCommand = YawControllerConfig::kPedalCommandSlope * rudderForceCommand - YawControllerConfig::kPedalCommandOffset;
            }

            rudderCommand = limit(rudderCommand, -YawControllerConfig::kRudderLimitDeg, YawControllerConfig::kRudderLimitDeg);
            double rudderCommandFiltered = rudderCommandFilter.Filter(resetFilters, dt, rudderCommand);
            double pedalCommand = pedTrim - rudderCommandFiltered;

            double alphaGained = aoa_filtered * YawControllerConfig::kYawAlphaCouplingGain;
            double rollRateCoupling = roll_rate * alphaGained;
            double yawRateCorrected = yaw_rate - rollRateCoupling;

            double yawRateWashedOut = yawRateWashout.Filter(resetFilters, dt, yawRateCorrected);
            double yawRateSmoothed = yawRateFilter.Filter(resetFilters, dt, yawRateWashedOut);
            double yawDamping = YawControllerConfig::kYawDampingGain * yawRateSmoothed;

            double sideAccelFeedback = YawControllerConfig::kSideAccelFeedbackGain * ay;
            double ariCommand = limit(YawControllerConfig::kAriAoAGain * aoa_filtered, 0.0, YawControllerConfig::kAriLimit) * aileron_commanded;
            double combinedCommand = pedalCommand + yawDamping + sideAccelFeedback + ariCommand;
            double yawSurfaceCommand = yawServoFilter.Filter(resetFilters, dt, combinedCommand);

            lastYawPedInput = pedInput;
            lastYawRateDegS = yaw_rate;
            lastYawRudderCommand = rudderCommand;
            lastYawRudderCommandFiltered = rudderCommandFiltered;
            lastYawPedalCommand = pedalCommand;
            lastYawDamping = yawDamping;
            lastYawSideAccelFeedback = sideAccelFeedback;
            lastYawAriCommand = ariCommand;
            lastYawCombinedCommand = combinedCommand;
            lastYawSurfaceCommand = yawSurfaceCommand;

            return yawSurfaceCommand;
        }

        double fcs_yaw_controller(double pedInput, double pedTrim, double yaw_rate, double roll_rate, double aoa_filtered, double aileron_commanded, double ay, double dt)
        {
            const bool needsInit = yaw_filters_need_init();
            if (needsInit)
            {
                init_yaw_filters(kFixedControlTimeStep);
            }

            advance_control_accumulator(yawControlAccumulator, dt);
            if (needsInit && yawControlAccumulator < kFixedControlTimeStep)
            {
                yawControlAccumulator = kFixedControlTimeStep;
            }

            bool resetFilters = needsInit;
            while (yawControlAccumulator >= kFixedControlTimeStep)
            {
                yawHeldOutput = fcs_yaw_controller_step(
                    pedInput,
                    pedTrim,
                    yaw_rate,
                    roll_rate,
                    aoa_filtered,
                    aileron_commanded,
                    ay,
                    kFixedControlTimeStep,
                    resetFilters);
                resetFilters = false;
                yawControlAccumulator -= kFixedControlTimeStep;
            }

            return yawHeldOutput;
        }

        double fcs_pitch_controller_force_command(double longStickInputCommand, double pitchTrim, double dt)
        {
            double longStickInputForce = 0.0;
            if (longStickInputCommand > 0.0)
            {
                longStickInputForce = longStickInputCommand * PitchControllerConfig::kPositiveStickForceScale + pitchTrim;
            }
            else
            {
                longStickInputForce = longStickInputCommand * PitchControllerConfig::kNegativeStickForceScale;
            }
            longStickInputForce = limit(longStickInputForce, PitchControllerConfig::kStickForceMin, PitchControllerConfig::kStickForceMax);
            longStickForce = longStickInputForce;

            double longStickCommand_G = 0.0;
            if (std::abs(longStickInputForce) <= PitchControllerConfig::kStickCommandDeadband)
            {
                longStickCommand_G = 0.0;
            }
            else if ((longStickInputForce < -PitchControllerConfig::kStickCommandDeadband) && (longStickInputForce > -PitchControllerConfig::kStickCommandBreakpoint))
            {
                longStickCommand_G = PitchControllerConfig::kNzSlopeNeg1 * longStickInputForce + PitchControllerConfig::kNzInterceptNeg1;
            }
            else if (longStickInputForce <= -PitchControllerConfig::kStickCommandBreakpoint)
            {
                longStickCommand_G = PitchControllerConfig::kNzSlopeNeg2 * longStickInputForce + PitchControllerConfig::kNzInterceptNeg2;
            }
            else if ((longStickInputForce > PitchControllerConfig::kStickCommandDeadband) && (longStickInputForce < PitchControllerConfig::kStickCommandBreakpoint))
            {
                longStickCommand_G = PitchControllerConfig::kNzSlopePos1 * longStickInputForce + PitchControllerConfig::kNzInterceptPos1;
            }
            else if (longStickInputForce >= PitchControllerConfig::kStickCommandBreakpoint)
            {
                longStickCommand_G = PitchControllerConfig::kNzSlopePos2 * longStickInputForce + PitchControllerConfig::kNzInterceptPos2;
            }

            double longStickCommandWithTrim_G = pitchTrim - longStickCommand_G;
            double longStickCommandWithTrimLimited_G = limit(longStickCommandWithTrim_G, PitchControllerConfig::kNzCommandMin, PitchControllerConfig::kNzCommandMax);
            stickCommandPosFiltered = longStickCommandWithTrimLimited_G;

            return stickCommandPosFiltered;
        }

        double fcs_pitch_rate_controller_force_command(double longStickInputCommand, double pitchTrim)
        {
            double longStickInputForce = 0.0;
            if (longStickInputCommand >= 0.0)
            {
                longStickInputForce = longStickInputCommand * PitchControllerConfig::kPositiveStickForceScale;
            }
            else
            {
                longStickInputForce = longStickInputCommand * PitchControllerConfig::kNegativeStickForceScale;
            }

            longStickInputForce = limit(longStickInputForce, PitchControllerConfig::kStickForceMin, PitchControllerConfig::kStickForceMax);

            double pitchRateCommandDegS = 0.0;
            if (std::abs(longStickInputForce) <= PitchControllerConfig::kStickCommandDeadband)
            {
                pitchRateCommandDegS = 0.0;
            }
            else if (longStickInputForce < -PitchControllerConfig::kStickCommandDeadband)
            {
                if (longStickInputForce > -PitchControllerConfig::kStickCommandBreakpoint)
                {
                    const double blend = (-longStickInputForce - PitchControllerConfig::kStickCommandDeadband) /
                        (PitchControllerConfig::kStickCommandBreakpoint - PitchControllerConfig::kStickCommandDeadband);
                    pitchRateCommandDegS =
                        PitchControllerConfig::kPitchRateCommandAtNegForceBreakDegS * blend;
                }
                else
                {
                    const double blend = (-longStickInputForce - PitchControllerConfig::kStickCommandBreakpoint) /
                        (-PitchControllerConfig::kStickForceMin - PitchControllerConfig::kStickCommandBreakpoint);
                    pitchRateCommandDegS =
                        PitchControllerConfig::kPitchRateCommandAtNegForceBreakDegS +
                        (PitchControllerConfig::kPitchRateCommandMinDegS - PitchControllerConfig::kPitchRateCommandAtNegForceBreakDegS) * blend;
                }
            }
            else
            {
                if (longStickInputForce < PitchControllerConfig::kStickCommandBreakpoint)
                {
                    const double blend = (longStickInputForce - PitchControllerConfig::kStickCommandDeadband) /
                        (PitchControllerConfig::kStickCommandBreakpoint - PitchControllerConfig::kStickCommandDeadband);
                    pitchRateCommandDegS =
                        PitchControllerConfig::kPitchRateCommandAtPosForceBreakDegS * blend;
                }
                else
                {
                    const double blend = (longStickInputForce - PitchControllerConfig::kStickCommandBreakpoint) /
                        (PitchControllerConfig::kStickForceMax - PitchControllerConfig::kStickCommandBreakpoint);
                    pitchRateCommandDegS =
                        PitchControllerConfig::kPitchRateCommandAtPosForceBreakDegS +
                        (PitchControllerConfig::kPitchRateCommandMaxDegS - PitchControllerConfig::kPitchRateCommandAtPosForceBreakDegS) * blend;
                }
            }

            // Match the existing Nz-command stick sense: aft stick should command positive pull/pitch-up.
            // For quick A/B testing we re-use pitchTrim as a small bias on the pitch-rate command.
            return pitchTrim - pitchRateCommandDegS;
        }

        double fcs_pitch_controller_step(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind, double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_LBFT2, double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s, double velocity_fps, double mach, double thrust_N, double Cx_total, bool resetFilters)
        {
            // TODO: differentialCommand (elevon differential for pitch/roll mixing) is not yet implemented.
            (void)differentialCommand;

            const PitchCommandMode activePitchCommandMode = resolve_pitch_command_mode();
            const bool usePitchRateCommand = (activePitchCommandMode == PitchCommandMode::PitchRate);
            double stickCommandPos = fcs_pitch_controller_force_command(longStickInputCommand, pitchTrim, dt);
            double pitchRateCommandDegS = fcs_pitch_rate_controller_force_command(longStickInputCommand, pitchTrim);
            double roll_RAD = roll_angle_DEG * (kPi / 180.0);
            double pitch_RAD = pitch_angle_DEG * (kPi / 180.0);

            azFiltered = az;

            double alphaLimited = limit(angle_of_attack_ind, -5.0, PitchControllerConfig::kAoAMax);
            const double alphaBlend = 1.0 - std::exp(-PitchControllerConfig::kAoAFilterRate * dt);
            alphaFiltered += (alphaLimited - alphaFiltered) * alphaBlend;

            const double pitchRate_RAD_s = pitch_rate_DEG_s * (kPi / 180.0);
            double gravity_component = std::cos(roll_RAD) * std::cos(pitch_RAD);
            double nz_measured = azFiltered + gravity_component;

            if (!usePitchRateCommand && std::fabs(stickCommandPos) < PitchControllerConfig::kNzStickDeadband)
            {
                const double gravityError = gravity_component - nz_reference;
                const double trackRate = (gravityError > 0.0)
                    ? PitchControllerConfig::kNzReferenceSnapRate
                    : PitchControllerConfig::kNzReferenceTrackRate;
                const double referenceBlend = 1.0 - std::exp(-trackRate * dt);
                nz_reference += gravityError * referenceBlend;
            }

            {
                const double roll_rate_rad_s = roll_rate_DEG_s * (kPi / 180.0);
                const double pitch_rate_rad_s = pitch_rate_DEG_s * (kPi / 180.0);
                const double gravity_rate =
                    (-std::sin(roll_RAD) * std::cos(pitch_RAD) * roll_rate_rad_s) +
                    (-std::cos(roll_RAD) * std::sin(pitch_RAD) * pitch_rate_rad_s);
                nz_reference += PitchControllerConfig::kGravityFeedForwardGain * gravity_rate * dt;
                nz_reference = limit(nz_reference, PitchControllerConfig::kNzMin, PitchControllerConfig::kNzMax);
            }

            double nz_cmd = limit(nz_reference + stickCommandPos, PitchControllerConfig::kNzMin, PitchControllerConfig::kNzMax);
            double nz_error = nz_cmd - nz_measured;
            double nzProportional = PitchControllerConfig::kNzErrorGain * nz_error;
            double pitchDamping = 0.0;
            double nz_control = 0.0;
            double integratorGain = PitchControllerConfig::kPitchIntegratorGain;
            double integratorInput = 0.0;
            double integratorLimit = PitchControllerConfig::kPitchIntegratorLimit;
            double finalCombinedCommandFilteredLimited = 0.0;
            double pitchModeCommandForLog = usePitchRateCommand ? pitchRateCommandDegS : nz_cmd;

            if (usePitchRateCommand)
            {
                double pitchRateLimiterBiasDegS = 0.0;
                if (nz_measured > PitchControllerConfig::kPitchRateNzLimitStartPositive)
                {
                    pitchRateLimiterBiasDegS -= PitchControllerConfig::kPitchRateNzLimiterGainDegSPerG * (nz_measured - PitchControllerConfig::kPitchRateNzLimitStartPositive);
                }
                else if (nz_measured < PitchControllerConfig::kPitchRateNzLimitStartNegative)
                {
                    pitchRateLimiterBiasDegS += PitchControllerConfig::kPitchRateNzLimiterGainDegSPerG * (PitchControllerConfig::kPitchRateNzLimitStartNegative - nz_measured);
                }

                if (alphaFiltered > PitchControllerConfig::kAoALimitStart)
                {
                    pitchRateLimiterBiasDegS -= PitchControllerConfig::kPitchRateAoALimiterGainDegSPerDeg * (alphaFiltered - PitchControllerConfig::kAoALimitStart);
                }

                const double pitchRateLimitedCommandDegS = limit(
                    pitchRateCommandDegS + pitchRateLimiterBiasDegS,
                    PitchControllerConfig::kPitchRateCommandMinDegS,
                    PitchControllerConfig::kPitchRateCommandMaxDegS);
                const double pitchRateCommandRadS = pitchRateLimitedCommandDegS * (kPi / 180.0);
                const double pitchRateError = pitchRateCommandRadS - pitchRate_RAD_s;
                nzProportional = PitchControllerConfig::kPitchRateErrorGain * pitchRateError;
                nz_error = pitchRateError;
                nz_cmd = pitchRateLimitedCommandDegS;
                pitchModeCommandForLog = pitchRateLimitedCommandDegS;
                integratorGain = PitchControllerConfig::kPitchRateIntegratorGain;
                integratorInput = pitchRateError;
                integratorLimit = PitchControllerConfig::kPitchRateIntegratorLimit;

                pitchDamping = PitchControllerConfig::kPitchRateDampingGain * pitchRate_RAD_s;
                double pitchRateNzProtection = 0.0;
                if (nz_measured > PitchControllerConfig::kPitchRateNzLimitStartPositive)
                {
                    pitchRateNzProtection -= PitchControllerConfig::kPitchRateNzProtectionGain *
                        (nz_measured - PitchControllerConfig::kPitchRateNzLimitStartPositive);
                }
                else if (nz_measured < PitchControllerConfig::kPitchRateNzLimitStartNegative)
                {
                    pitchRateNzProtection += PitchControllerConfig::kPitchRateNzProtectionGain *
                        (PitchControllerConfig::kPitchRateNzLimitStartNegative - nz_measured);
                }

                const double unsaturatedCommandBeforeIntegration =
                    nzProportional + pitchIntegratorState + pitchDamping + pitchRateNzProtection;
                const bool actuatorSaturatedHigh =
                    (unsaturatedCommandBeforeIntegration >= integratorLimit && pitchRateError > 0.0);
                const bool actuatorSaturatedLow =
                    (unsaturatedCommandBeforeIntegration <= -integratorLimit && pitchRateError < 0.0);
                if (actuatorSaturatedHigh || actuatorSaturatedLow)
                {
                    integratorInput = 0.0;
                }

                if ((pitchIntegratorState > 0.0 && integratorInput < 0.0) ||
                    (pitchIntegratorState < 0.0 && integratorInput > 0.0))
                {
                    integratorGain *= PitchControllerConfig::kPitchIntegratorUnwindGain;
                }

                pitchIntegratorState = limit(
                    pitchIntegratorState + integratorGain * dt * integratorInput,
                    -integratorLimit,
                    integratorLimit);

                const double integratorOutput = pitchIntegratorState;
                nz_control = nzProportional + pitchDamping + pitchRateNzProtection;
                finalCombinedCommandFilteredLimited = limit(
                    nzProportional + integratorOutput + pitchDamping + pitchRateNzProtection,
                    -integratorLimit,
                    integratorLimit);
            }
            else
            {
                pitchDamping = PitchControllerConfig::kPitchDampingGain * pitchRate_RAD_s;
                nz_control = nzProportional + pitchDamping;

                bool nzAtUpperLimit = (nz_measured >= PitchControllerConfig::kNzMax && nz_error > 0.0);
                bool nzAtLowerLimit = (nz_measured <= PitchControllerConfig::kNzMin && nz_error < 0.0);

                // Freeze the integrator only when the aircraft is already at the measured G limit,
                // or when the controller output itself is saturated and the error is still trying to
                // drive further into saturation.  This preserves authority to reach the commanded G
                // limit without allowing integrator wind-up into a jerky response.
                const double unsaturatedCommandBeforeIntegration =
                    nzProportional + pitchIntegratorState + pitchDamping;
                const bool actuatorSaturatedHigh =
                    (unsaturatedCommandBeforeIntegration >= integratorLimit && nz_error > 0.0);
                const bool actuatorSaturatedLow =
                    (unsaturatedCommandBeforeIntegration <= -integratorLimit && nz_error < 0.0);
                integratorInput =
                    (nzAtUpperLimit || nzAtLowerLimit || actuatorSaturatedHigh || actuatorSaturatedLow) ? 0.0 : nz_error;

                if ((pitchIntegratorState > 0.0 && integratorInput < 0.0) ||
                    (pitchIntegratorState < 0.0 && integratorInput > 0.0))
                {
                    integratorGain *= PitchControllerConfig::kPitchIntegratorUnwindGain;
                }

                pitchIntegratorState = limit(
                    pitchIntegratorState + integratorGain * dt * integratorInput,
                    -integratorLimit,
                    integratorLimit);

                const double integratorOutput = pitchIntegratorState;
                finalCombinedCommandFilteredLimited = limit(
                    nzProportional + integratorOutput + pitchDamping,
                    -integratorLimit,
                    integratorLimit);
            }

            double integratorOutput = pitchIntegratorState;
            double finalPitchCommandTotal = pitchActuatorDynamicsFilter.Filter(resetFilters, dt, finalCombinedCommandFilteredLimited);

            if (alphaFiltered > PitchControllerConfig::kAoALimitStart)
            {
                double aoaError = alphaFiltered - PitchControllerConfig::kAoALimitStart;
                double aoaPushdown = PitchControllerConfig::kAoAPushdownGain * aoaError;
                if (finalPitchCommandTotal > 0.0)
                {
                    double limitFactor = 1.0 - (aoaError / (PitchControllerConfig::kAoAMax - PitchControllerConfig::kAoALimitStart));
                    limitFactor = limit(limitFactor, 0.0, 1.0);
                    finalPitchCommandTotal *= limitFactor;
                }
                finalPitchCommandTotal -= aoaPushdown;
            }

            log_pitch_debug_sample(
                roll_angle_DEG,
                pitch_angle_DEG,
                roll_rate_DEG_s,
                static_cast<double>(usePitchRateCommand ? 1 : 0),
                pitchModeCommandForLog,
                az,
                gravity_component,
                nz_measured,
                nz_reference,
                stickCommandPos,
                nz_cmd,
                nz_error,
                nzProportional,
                pitchDamping,
                nz_control,
                integratorOutput,
                angle_of_attack_ind,
                alphaFiltered,
                longStickInputCommand,
                pitch_rate_DEG_s,
                dynPressure_LBFT2,
                velocity_fps,
                mach,
                thrust_N,
                Cx_total,
                finalPitchCommandTotal);

            return finalPitchCommandTotal;
        }

        double fcs_pitch_controller(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind, double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_LBFT2, double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s, double velocity_fps, double mach, double thrust_N, double Cx_total)
        {
            const bool needsInit = pitch_filters_need_init();
            if (needsInit)
            {
                init_pitch_filters(kFixedControlTimeStep);
            }

            advance_control_accumulator(pitchControlAccumulator, dt);
            if (needsInit && pitchControlAccumulator < kFixedControlTimeStep)
            {
                pitchControlAccumulator = kFixedControlTimeStep;
            }

            bool resetFilters = needsInit;
            while (pitchControlAccumulator >= kFixedControlTimeStep)
            {
                pitchHeldOutput = fcs_pitch_controller_step(
                    longStickInputCommand,
                    pitchTrim,
                    angle_of_attack_ind,
                    pitch_rate_DEG_s,
                    az,
                    differentialCommand,
                    dynPressure_LBFT2,
                    kFixedControlTimeStep,
                    roll_angle_DEG,
                    pitch_angle_DEG,
                    roll_rate_DEG_s,
                    velocity_fps,
                    mach,
                    thrust_N,
                    Cx_total,
                    resetFilters);
                resetFilters = false;
                pitchControlAccumulator -= kFixedControlTimeStep;
            }

            return pitchHeldOutput;
        }
        double fcs_roll_controller_step(double latStickInputCommand, double longStickForceCommand, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_LBFT2, double dt, bool resetFilters)
        {
            double latStickForceCmd = latStickInputCommand * RollControllerConfig::kLatStickForceScale;
            double latStickForce = latStickForceFilter.Filter(resetFilters, dt, latStickForceCmd);
            double ayBiasBlend = limit(std::abs(latStickInputCommand) / RollControllerConfig::kAyBiasFullStick, 0.0, 1.0);
            double latStickForceBiased = latStickForce - (ay * RollControllerConfig::kAyBiasGain * ayBiasBlend);

            double longStickForceGained = longStickForceCommand * RollControllerConfig::kLongStickFeelGain;
            double rollFeelGain = 0.0;
            if (std::abs(longStickForceCommand) > RollControllerConfig::kRollFeelForceThreshold)
            {
                rollFeelGain = RollControllerConfig::kRollFeelFixedGain;
            }
            else if (longStickForceCommand >= 0.0)
            {
                rollFeelGain = -RollControllerConfig::kRollFeelSlope * longStickForceGained + 1.0;
            }
            else if (longStickForceCommand < 0.0)
            {
                rollFeelGain = RollControllerConfig::kRollFeelSlope * longStickForceGained + 1.0;
            }

            double latStickForceFinal = latStickForceBiased * rollFeelGain;
            double rollRateCommand = 0.0;
            if (std::abs(latStickForceFinal) < RollControllerConfig::kStickDeadband)
            {
                rollRateCommand = 0.0;
            }
            else if ((latStickForceFinal >= RollControllerConfig::kStickDeadband) && (latStickForceFinal <= RollControllerConfig::kFirstBreakpoint))
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg1 * latStickForceFinal + RollControllerConfig::kRollInterceptSeg1;
            }
            else if ((latStickForceFinal > RollControllerConfig::kFirstBreakpoint) && (latStickForceFinal <= RollControllerConfig::kSecondBreakpoint))
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg2 * latStickForceFinal + RollControllerConfig::kRollInterceptSeg2;
            }
            else if (latStickForceFinal > RollControllerConfig::kSecondBreakpoint)
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg3 * latStickForceFinal + RollControllerConfig::kRollInterceptSeg3;
            }
            else if ((latStickForceFinal <= -RollControllerConfig::kStickDeadband) && (latStickForceFinal >= -RollControllerConfig::kFirstBreakpoint))
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg1 * latStickForceFinal - RollControllerConfig::kRollInterceptSeg1;
            }
            else if ((latStickForceFinal < -RollControllerConfig::kFirstBreakpoint) && (latStickForceFinal >= -RollControllerConfig::kSecondBreakpoint))
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg2 * latStickForceFinal - RollControllerConfig::kRollInterceptSeg2;
            }
            else if (latStickForceFinal < -RollControllerConfig::kSecondBreakpoint)
            {
                rollRateCommand = RollControllerConfig::kRollSlopeSeg3 * latStickForceFinal - RollControllerConfig::kRollInterceptSeg3;
            }

            double flatTurnPedalBlend = limit(std::abs(pedInput) / RollControllerConfig::kFlatTurnPedalFull, 0.0, 1.0);
            double flatTurnStickBlend = 1.0 - limit(std::abs(latStickInputCommand) / RollControllerConfig::kFlatTurnLatStickFade, 0.0, 1.0);
            double flatTurnBlend = flatTurnPedalBlend * flatTurnStickBlend;
            double flatTurnRollCommand = flatTurnBlend * limit(
                beta_deg * RollControllerConfig::kFlatTurnBetaGain - roll_angle_deg * RollControllerConfig::kFlatTurnBankGain,
                -RollControllerConfig::kFlatTurnCommandLimit,
                RollControllerConfig::kFlatTurnCommandLimit);

            double rollRateCommandTotal = rollRateCommand + flatTurnRollCommand;
            double rollRateCommandFiltered = rollCommandFilter.Filter(resetFilters, dt, rollRateCommandTotal);
            double rollRateFiltered1 = rollRateFilter1.Filter(resetFilters, dt, roll_rate);
            double rollRateFiltered2 = rollRateFilter2.Filter(resetFilters, dt, rollRateFiltered1);
            double rollRateCommandCombined = rollRateFiltered2 - rollRateCommandFiltered - roll_rate_trim;

            double dynamicPressure_NM2 = dynPressure_LBFT2 * RollControllerConfig::kDynPressureLbFt2ToNm2;
            double pressureGain = 0.0;
            if (dynamicPressure_NM2 < RollControllerConfig::kPressureLow)
            {
                pressureGain = RollControllerConfig::kPressureLowGain;
            }
            else if ((dynamicPressure_NM2 >= RollControllerConfig::kPressureLow) && (dynamicPressure_NM2 <= RollControllerConfig::kPressureHigh))
            {
                pressureGain = RollControllerConfig::kPressureSlope * dynamicPressure_NM2 + RollControllerConfig::kPressureOffset;
            }
            else
            {
                pressureGain = RollControllerConfig::kPressureHighGain;
            }

            double rollCommandGained = limit(rollRateCommandCombined * pressureGain, -RollControllerConfig::kRollCommandLimit, RollControllerConfig::kRollCommandLimit);
            double rollSurfaceCommand = rollActuatorDynamicsFilter.Filter(resetFilters, dt, rollCommandGained);

            lastRollStickInput = latStickInputCommand;
            lastRollStickForce = latStickForceFinal;
            lastRollRateCommand = rollRateCommand;
            lastRollFlatTurnCommand = flatTurnRollCommand;
            lastRollRateCommandFiltered = rollRateCommandFiltered;
            lastRollCommandGained = rollCommandGained;
            lastRollSurfaceCommand = rollSurfaceCommand;

            return rollSurfaceCommand;
        }

        double fcs_roll_controller(double latStickInputCommand, double longStickForceCommand, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_LBFT2, double dt)
        {
            const bool needsInit = roll_filters_need_init();
            if (needsInit)
            {
                init_roll_filters(kFixedControlTimeStep);
            }

            advance_control_accumulator(rollControlAccumulator, dt);
            if (needsInit && rollControlAccumulator < kFixedControlTimeStep)
            {
                rollControlAccumulator = kFixedControlTimeStep;
            }

            bool resetFilters = needsInit;
            while (rollControlAccumulator >= kFixedControlTimeStep)
            {
                rollHeldOutput = fcs_roll_controller_step(
                    latStickInputCommand,
                    longStickForceCommand,
                    ay,
                    pedInput,
                    beta_deg,
                    roll_angle_deg,
                    roll_rate,
                    roll_rate_trim,
                    dynPressure_LBFT2,
                    kFixedControlTimeStep,
                    resetFilters);
                resetFilters = false;
                rollControlAccumulator -= kFixedControlTimeStep;
            }

            return rollHeldOutput;
        }
    }
}







