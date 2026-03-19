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
            GeneralFilter pitchRateWashoutFilter;        // AOA limiter pitch-rate anticipation
            GeneralFilter pitchRateFeedbackWashout;      // main-loop pitch-rate blend (P+I feedback)

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

            double autoBetaTrimState      = 0.0; // automatic Beta trim accumulator (deg)
            double aoaLimiterIntegrator   = 0.0; // P+I AOA limiter integrator state

            constexpr double kPi = 3.14159265358979323846;
            constexpr double kFixedControlTimeStep = 0.005;
            constexpr int kMaxControlSubstepsPerFrame = 20;
            constexpr double kMaxAccumulatedControlTime = kFixedControlTimeStep * kMaxControlSubstepsPerFrame;
            constexpr int kPitchDebugLogIntervalFrames = 10;


            namespace YawControllerConfig
            {
                // Beta-command architecture (real F-117 directional axis).
                // Pedal position commands a target Beta; fins are driven to satisfy that Beta.

                // Pedal → Beta command gain (deg Beta per unit normalised pedal).
                constexpr double kBetaCommandGain = 10.0;
                // Beta command limit schedule vs dynamic pressure.
                // Below kBetaCmdQbarLow the full limit applies (crosswind landing authority).
                // Above kBetaCmdQbarHigh the limit is reduced to protect the airframe.
                constexpr double kBetaCmdQbarLow  = 2000.0;   // Pa (~80 kts)
                constexpr double kBetaCmdQbarHigh = 28000.0;  // Pa (~370 kts)
                constexpr double kBetaCmdLimitHigh = 10.0;    // deg (low-speed limit)
                constexpr double kBetaCmdLimitLow  =  3.0;    // deg (high-speed limit)
                // Small pedal deadband at neutral (normalised units).
                constexpr double kBetaPedalDeadband = 0.02;
                // Base Beta feedback gain (deg fin per deg Beta error).
                // Gain doubles linearly as AOA increases from 0 to kBetaGainAoADoubleAlpha
                // to keep Beta tight at high AOA where its destabilising pitching moment is largest.
                constexpr double kBetaFeedbackGainBase     = 3.0;
                constexpr double kBetaGainAoADoubleAlpha   = 20.0; // deg AOA where gain doubles
                // Automatic Beta trim: bleeds residual Beta out slowly while gear is up and
                // pedal force is below threshold. Acts through a small deadband.
                constexpr double kAutoBetaTrimDeadband        = 0.5;  // deg
                constexpr double kAutoBetaTrimRate            = 0.4;  // (deg trim)/s per deg Beta excess
                constexpr double kAutoBetaTrimLimit           = 5.0;  // deg max trim authority
                constexpr double kAutoBetaTrimPedalThreshold  = 0.05; // normalised pedal (~22 lb)
                // Inertia-coupling term: product of pitch_rate × roll_rate fed back to yaw
                // to prevent divergence during high roll-rate manoeuvres (real F-117 feature).
                constexpr double kInertiaCouplingGain = -0.4;  // deg fin per (rad/s)^2
                // Yaw-rate damping.
                constexpr double kYawDampingGain = 1.5;
                // Beta command low-pass filter rate (smooths pedal transients).
                constexpr double kCommandFilterRate = 4.0;
                // Stability-axis yaw rate washout time-constant inverse.
                constexpr double kWashoutTauInverse = 1.0;
                // Yaw rate lead-lag shaping.
                constexpr double kYawLeadLagNumerator0 = 3.0;
                constexpr double kYawLeadLagPole = 15.0;
                // Fin servo dynamics.
                constexpr double kServoNaturalFrequency = 52.0;
                constexpr double kServoDampingRatio = 0.95;
            }

            namespace PitchControllerConfig
            {
                // Threshold on gearDown signal (0–1) above which gear is considered extended.
                constexpr double kGearDownThreshold = 0.1;
                constexpr double kPositiveStickForceScale = 80.0;
                constexpr double kNegativeStickForceScale = 180.0;
                constexpr double kStickForceMin = -180.0;
                constexpr double kStickForceMax = 80.0;
                constexpr double kStickCommandDeadband = 8.0;
                constexpr double kStickCommandBreakpoint = 33.0;
                constexpr double kNzCommandMin = -2.0;
                constexpr double kNzCommandMax = 7.0;
                // Integrator strength on Nz error. Higher = more steady-state authority and more stored pull;
                // lower = less overshoot/carry-through, but slower trim-in to the commanded G.
                constexpr double kPitchIntegratorGain = 0.75;
                // Extra multiplier used only when the integrator is unwinding against the current error.
                // Higher = faster dump of stored pull/push and less overshoot; lower = smoother, but more carry-through.
                constexpr double kPitchIntegratorUnwindGain = 3.0; //was 4.0 TEST
                constexpr double kAoAFilterRate = 35.0;
                // Real F-117: full aft stick = +7g, full forward = -2g.
                constexpr double kNzMax = 7.0;
                constexpr double kNzMin = -2.0;
                constexpr double kNzStickDeadband = 0.05;
                constexpr double kNzReferenceTrackRate = 2.0;
                // Proportional gain on Nz error (scaled by 1/qbar at runtime).
                constexpr double kNzErrorGain = 1.7;
                // Pitch-rate blend: washed-out pitch rate is blended INTO the Nz feedback before the
                // error is computed (per Loschke: "specified blend of washed out pitch rate and the
                // normal acceleration"). This gives a true P+I architecture — pitch rate damps transient
                // response via the error signal; the washout ensures zero steady-state contribution so the
                // integrator does not fight the damper in sustained pull/push manoeuvres.
                // Gain derived to match old separate-damping authority at reference q:
                //   kPitchRateBlendGain = old_kPitchDampingGain_magnitude / kNzErrorGain = 7.0 / 1.7 ≈ 4.1
                constexpr double kPitchRateBlendGain          = 4.1;  // g·s/rad
                constexpr double kPitchRateBlendWashoutTauInv = 1.0;  // rad/s (1-second washout time constant)
                constexpr double kPitchIntegratorLimit = 25.0;
                constexpr double kAoAMax = 20.0;
                // Base AOA limiter threshold — Mach-scheduled at runtime: threshold rises at high Mach
                // (airframe becomes more stable transsonically) so the limiter is less conservative.
                constexpr double kAoALimitStart = 19.0;
                constexpr double kAoALimiterMachGain = 2.5; // deg threshold added per Mach unit
                // Gear-down: AOA limiter biased higher to allow lower approach/landing speeds.
                constexpr double kAoALimitStartGearDownBias = 3.0; // deg added to threshold
                // High-gain P+I AOA limiter (has more authority than the pilot).
                // The limiter is triggered by alpha + washed-out pitch rate anticipation exceeding
                // the Mach-scheduled threshold.
                constexpr double kAoALimiterProportionalGain = 8.0;
                constexpr double kAoALimiterIntegratorGain   = 2.5;
                constexpr double kAoALimiterIntegratorLimit  = 30.0;
                // Pitch-rate anticipation for the AOA limiter.
                // Washed-out pitch rate is scaled by kAoAAnticipationGainRef / qbar so the anticipation
                // is stronger at low q (where departure risk is highest on aggressive pull-ups).
                constexpr double kAoAAnticipationWashoutTauInverse = 2.0;  // 0.5 s washout
                constexpr double kAoAAnticipationGainRef            = 1.5; // deg AoA per deg/s at kQbarRef
                constexpr double kAoAAnticipationMax                = 8.0; // deg (anticipation clamp)
                // Dynamic pressure scheduling of forward-loop gains (1/qbar).
                // All forward-loop gains are proportional to qbarRef/qbar, clamped to [min, max].
                constexpr double kQbarRef          = 14000.0; // Pa (~300 kts SL)
                constexpr double kQbarMin          =  2000.0; // Pa (prevents divide issues)
                constexpr double kQbarGainScaleMin =  0.5;
                constexpr double kQbarGainScaleMax =  3.0;
                // Roll-rate^2 inertia coupling fed back to pitch axis.
                // Prevents inertia-coupled pitch departures during high-AoA rolling manoeuvres.
                constexpr double kRollRateSquaredAoAThreshold  = 12.0; // deg, AoA above which active
                constexpr double kRollRateSquaredRateThreshold =  1.0; // rad/s, rate above which active
                constexpr double kRollRateSquaredGain          =  0.05; // deg elevator per (rad/s)^2
                // Speed stability: non-linear AoA feedback at AoA > 7 deg and below 200 kts.
                // Provides apparent speed stability on approach (pitches down as speed falls).
                constexpr double kSpeedStabilityVelocityMPS  = 103.0; // 200 kts
                constexpr double kSpeedStabilityAoAThreshold =   7.0; // deg
                constexpr double kSpeedStabilityGain         =   0.3; // deg per deg AoA excess
                constexpr double kPitchServoNaturalFrequency = 52.0;
                constexpr double kPitchServoDampingRatio = 1.5;


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
                constexpr double kPressureLow = 19153.0;
                constexpr double kPressureHigh = 23941.0;
                constexpr double kPressureLowGain = 2.0;
                constexpr double kPressureSlope = -0.0002088;
                constexpr double kPressureOffset = 5.998;
                constexpr double kPressureHighGain = 1.0;
                constexpr double kRollCommandLimit = 21.5;
                constexpr double kLatForceFilterPole = 60.0;
                constexpr double kRollCommandFilterPole = 20.0;
                constexpr double kRollServoNaturalFrequency = 52.0;
                constexpr double kRollServoDampingRatio = 0.85;
                constexpr double kRollRateFilterPole = 50.0;
                constexpr double kRollRateFilter2Num0 = 4.0;
                constexpr double kRollRateFilter2Num1 = 64.0;
                constexpr double kRollRateFilter2Den1 = 80.0;
                constexpr double kRollRateFilter2OmegaSquared = 6400.0;

                // Parabolic stick-force-to-roll-rate shaping (real F-117 uses non-linear parabolic
                // shaping per Loschke). The schedule is: rollRateCmd = K * (force - deadband)^2
                // giving a gentle onset near centre and increasing responsiveness toward full stick.
                // kRollRateMax is the commanded rate at full stick deflection (150 deg/s estimated
                // from stock footage analysis). kRollParabolicGain is derived from these two values
                // so changing kRollRateMax automatically keeps the curve consistent.
                constexpr double kRollRateMax       = 150.0;  // deg/s at full stick
                constexpr double kRollUsableForce   = kLatStickForceScale - kStickDeadband; // 72 lbf
                constexpr double kRollParabolicGain = kRollRateMax / (kRollUsableForce * kRollUsableForce);
                // Gear down: roll rate feedback gain is increased to reduce turbulence response on approach.
                constexpr double kGearDownRateFeedbackGainFactor = 1.6;
                // Air refuelling: lateral stick input gain is reduced for control harmony with the
                // reduced pitch gain in AAR mode.
                constexpr double kAARLateralStickGainScale = 0.6;
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
                // Columns: pitch_mode=0 always in flight (NormalAcceleration P+I law).
                // pitch_rate_blend = washed pitch-rate contribution (g) blended into the Nz error signal.
                debugLog << "roll_deg,pitch_deg,roll_rate_dps,roll_stick,roll_stick_force,roll_rate_cmd,roll_flat_turn_cmd,roll_rate_cmd_filt,roll_cmd_surface,roll_surface_out,yaw_ped_input,beta_deg,yaw_rate_rps,yaw_rate_dps,yaw_rudder_cmd,yaw_rudder_cmd_filt,yaw_pedal_cmd,yaw_damping,yaw_side_accel,yaw_ari_cmd,yaw_combined_cmd,yaw_surface_out,rudder_deg_cmd,rudder_deg,rudder_pct,aero_cy_dr,aero_cn_dr,aero_cl_dr,aero_cn_dbeta,aero_cl_dbeta,aero_cy_total,aero_cn_total,aero_cl_total,az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,nz_proportional,pitch_rate_blend,nz_control,integratorOut,alpha_raw,alphaFiltered,stickInput,pitchRate,dynPressure_pa,velocity_mps,mach,thrust_N,Cx_total,elevatorOut\n";
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
                double rawAz,
                double gravityComponent,
                double nzMeasured,
                double nzReference,
                double stickCommandPos,
                double nzCommand,
                double nzError,
                double nzProportional,
                double pitchRateBlendContrib,  // washed pitch-rate blend contribution (g) in error signal
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
                         << rawAz << ","
                         << gravityComponent << ","
                         << nzMeasured << ","
                         << nzReference << ","
                         << stickCommandPos << ","
                         << nzCommand << ","
                         << nzError << ","
                         << nzProportional << ","
                         << pitchRateBlendContrib << ","
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
                // Beta command low-pass filter
                double numerators[2] = { 0.0, YawControllerConfig::kCommandFilterRate };
                double denominators[2] = { 1.0, YawControllerConfig::kCommandFilterRate };
                rudderCommandFilter.InitFilter(numerators, denominators, 1, dt);

                // Stability-axis yaw rate washout
                double numerators1[2] = { 1.0, 0.0 };
                double denominators1[2] = { 1.0, YawControllerConfig::kWashoutTauInverse };
                yawRateWashout.InitFilter(numerators1, denominators1, 1, dt);

                // Yaw rate lead-lag shaping
                double numerators2[2] = { YawControllerConfig::kYawLeadLagNumerator0, YawControllerConfig::kYawLeadLagPole };
                double denominators2[2] = { 1.0, YawControllerConfig::kYawLeadLagPole };
                yawRateFilter.InitFilter(numerators2, denominators2, 1, dt);

                // Fin servo dynamics
                const double servoOmegaSquared = std::pow(YawControllerConfig::kServoNaturalFrequency, 2.0);
                double numerators3[3] = { 0.0, 0.0, servoOmegaSquared };
                double denominators3[3] = { 1.0, 2.0 * YawControllerConfig::kServoDampingRatio * YawControllerConfig::kServoNaturalFrequency, servoOmegaSquared };
                yawServoFilter.InitFilter(numerators3, denominators3, 2, dt);

                yawControlAccumulator = 0.0;
                yawHeldOutput = 0.0;
                autoBetaTrimState = 0.0;
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

                // AOA limiter pitch-rate anticipation washout filter (tau = 0.5 s)
                double washoutNum[2]  = { 1.0, 0.0 };
                double washoutDen[2]  = { 1.0, PitchControllerConfig::kAoAAnticipationWashoutTauInverse };
                pitchRateWashoutFilter.InitFilter(washoutNum, washoutDen, 1, dt);

                // Main-loop pitch-rate feedback blend washout filter (tau = 1.0 s)
                // Separate from the AOA anticipation filter — different time constant.
                double blendWashoutNum[2] = { 1.0, 0.0 };
                double blendWashoutDen[2] = { 1.0, PitchControllerConfig::kPitchRateBlendWashoutTauInv };
                pitchRateFeedbackWashout.InitFilter(blendWashoutNum, blendWashoutDen, 1, dt);

                stickCommandPosFiltered = 0.0;
                azFiltered = 0.0;
                nz_reference = 1.0;
                pitchIntegratorState = 0.0;
                aoaLimiterIntegrator  = 0.0;
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
            autoBetaTrimState = 0.0;
            aoaLimiterIntegrator = 0.0;
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

        // Real F-117 directional axis: Beta-command augmentation (proportional only).
        // Pedal position commands a target sideslip angle (Beta). The fins are driven to
        // satisfy that command. Beta feedback gain is AoA-scheduled (doubles at high AoA to
        // resist the destabilising pitching moment from Beta). Automatic Beta trim bleeds
        // residual Beta when gear is up and pedal force is below threshold. Inertia coupling
        // (pitch_rate × roll_rate) prevents directional divergence during high-rate manoeuvres.
        // The ARI (aileron-rudder interconnect) present in early design was removed from the
        // real aircraft after evaluation — it is NOT implemented here.
        double fcs_yaw_controller_step(double pedInput, double pedTrim, double beta_deg, double yaw_rate, double roll_rate, double pitch_rate, double aoa_filtered, double dynPressure_PA, bool gearDown, double dt, bool resetFilters)
        {
            // 1. Pedal → Beta command (with small deadband)
            double rawPedalCmd = 0.0;
            if (std::abs(pedInput) > YawControllerConfig::kBetaPedalDeadband)
            {
                const double sign = (pedInput > 0.0) ? 1.0 : -1.0;
                rawPedalCmd = sign * (std::abs(pedInput) - YawControllerConfig::kBetaPedalDeadband)
                              / (1.0 - YawControllerConfig::kBetaPedalDeadband)
                              * YawControllerConfig::kBetaCommandGain;
            }

            // Dynamic pressure schedule: limit Beta command at high speed
            const double qbarClamped = (std::max)(dynPressure_PA, YawControllerConfig::kBetaCmdQbarLow);
            double betaCmdLimit = YawControllerConfig::kBetaCmdLimitHigh;
            if (qbarClamped >= YawControllerConfig::kBetaCmdQbarHigh)
            {
                betaCmdLimit = YawControllerConfig::kBetaCmdLimitLow;
            }
            else if (qbarClamped > YawControllerConfig::kBetaCmdQbarLow)
            {
                const double t = (qbarClamped - YawControllerConfig::kBetaCmdQbarLow)
                               / (YawControllerConfig::kBetaCmdQbarHigh - YawControllerConfig::kBetaCmdQbarLow);
                betaCmdLimit = YawControllerConfig::kBetaCmdLimitHigh
                             + t * (YawControllerConfig::kBetaCmdLimitLow - YawControllerConfig::kBetaCmdLimitHigh);
            }
            const double betaCmdLimited = limit(rawPedalCmd, -betaCmdLimit, betaCmdLimit);

            // Low-pass filter the Beta command to smooth pedal transients
            const double betaCmdFiltered = rudderCommandFilter.Filter(resetFilters, dt, betaCmdLimited);

            // 2. Automatic Beta trim (series trim): active when gear is up and pedal near centre.
            // Slowly trims away residual Beta (e.g. from engine thrust asymmetry).
            if (!gearDown && std::abs(pedInput) < YawControllerConfig::kAutoBetaTrimPedalThreshold)
            {
                const double betaExcess = beta_deg - limit(beta_deg, -YawControllerConfig::kAutoBetaTrimDeadband, YawControllerConfig::kAutoBetaTrimDeadband);
                autoBetaTrimState -= YawControllerConfig::kAutoBetaTrimRate * betaExcess * dt;
                autoBetaTrimState  = limit(autoBetaTrimState, -YawControllerConfig::kAutoBetaTrimLimit, YawControllerConfig::kAutoBetaTrimLimit);
            }

            const double betaCommandTotal = betaCmdFiltered + pedTrim + autoBetaTrimState;

            // 3. Beta feedback with AoA-scheduled gain.
            // Gain doubles linearly from base at 0° AoA to 2× base at kBetaGainAoADoubleAlpha.
            const double aoaFraction = limit(aoa_filtered / YawControllerConfig::kBetaGainAoADoubleAlpha, 0.0, 1.0);
            const double betaGain = YawControllerConfig::kBetaFeedbackGainBase * (1.0 + aoaFraction);
            const double betaError = betaCommandTotal - beta_deg;
            const double betaFeedback = betaGain * betaError;

            // 4. Stability-axis yaw rate damping (stability axis correction: subtract roll×alpha coupling)
            const double alphaRad = aoa_filtered * (kPi / 180.0);
            const double yawRateStabilityAxis = yaw_rate - roll_rate * alphaRad;
            const double yawRateWashedOut = yawRateWashout.Filter(resetFilters, dt, yawRateStabilityAxis);
            const double yawRateSmoothed  = yawRateFilter.Filter(resetFilters, dt, yawRateWashedOut);
            const double yawDamping = YawControllerConfig::kYawDampingGain * yawRateSmoothed;

            // 5. Inertia coupling: pitch_rate × roll_rate to prevent inertia-coupled yaw divergence.
            const double pitch_rate_rad_s = pitch_rate * (kPi / 180.0);
            const double roll_rate_rad_s  = roll_rate  * (kPi / 180.0);
            const double inertiaCoupling  = YawControllerConfig::kInertiaCouplingGain * pitch_rate_rad_s * roll_rate_rad_s;

            // 6. Combine and pass through fin servo dynamics
            const double combinedCommand    = betaFeedback + yawDamping + inertiaCoupling;
            const double yawSurfaceCommand  = yawServoFilter.Filter(resetFilters, dt, combinedCommand);

            lastYawPedInput            = pedInput;
            lastYawRateDegS            = yaw_rate;
            lastYawRudderCommand       = rawPedalCmd;
            lastYawRudderCommandFiltered = betaCmdFiltered;
            lastYawPedalCommand        = betaCommandTotal;
            lastYawDamping             = yawDamping;
            lastYawSideAccelFeedback   = inertiaCoupling;  // re-used log slot
            lastYawAriCommand          = 0.0;              // ARI removed from real aircraft
            lastYawCombinedCommand     = combinedCommand;
            lastYawSurfaceCommand      = yawSurfaceCommand;

            return yawSurfaceCommand;
        }

        double fcs_yaw_controller(double pedInput, double pedTrim, double beta_deg, double yaw_rate, double roll_rate, double pitch_rate, double aoa_filtered, double dynPressure_PA, bool gearDown, double dt)
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
                    beta_deg,
                    yaw_rate,
                    roll_rate,
                    pitch_rate,
                    aoa_filtered,
                    dynPressure_PA,
                    gearDown,
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


        double fcs_pitch_controller_step(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind, double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_PA, double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s, double velocity_mps, double mach, double thrust_N, double Cx_total, bool resetFilters)
        {
            // TODO: differentialCommand (elevon differential for pitch/roll mixing) is not yet implemented.
            (void)differentialCommand;

            double stickCommandPos = fcs_pitch_controller_force_command(longStickInputCommand, pitchTrim, dt);

            // AAR: reduce pitch stick position gain (per Loschke: "stick position input gain is decreased")
            if (pitchModeAirRefuelDoorOpen)
            {
                stickCommandPos *= RollControllerConfig::kAARLateralStickGainScale;
            }
            double roll_RAD = roll_angle_DEG * (kPi / 180.0);
            double pitch_RAD = pitch_angle_DEG * (kPi / 180.0);

            azFiltered = az;

            double alphaLimited = limit(angle_of_attack_ind, -5.0, PitchControllerConfig::kAoAMax);
            const double alphaBlend = 1.0 - std::exp(-PitchControllerConfig::kAoAFilterRate * dt);
            alphaFiltered += (alphaLimited - alphaFiltered) * alphaBlend;

            const double pitchRate_RAD_s = pitch_rate_DEG_s * (kPi / 180.0);
            double gravity_component = std::cos(roll_RAD) * std::cos(pitch_RAD);
            double nz_measured = azFiltered + gravity_component;

            if (std::fabs(stickCommandPos) < PitchControllerConfig::kNzStickDeadband)
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
            double nz_control = 0.0;
            double integratorGain = PitchControllerConfig::kPitchIntegratorGain;
            double integratorInput = 0.0;
            double integratorLimit = PitchControllerConfig::kPitchIntegratorLimit;
            double finalCombinedCommandFilteredLimited = 0.0;

            // 1/qbar dynamic pressure gain schedule (real F-117: all forward-loop gains
            // scheduled as a function of inverse dynamic pressure so stick feel is consistent
            // across the flight envelope).
            const double qbarScaled = (std::max)(dynPressure_PA, PitchControllerConfig::kQbarMin);
            const double qbarGainScale = limit(
                PitchControllerConfig::kQbarRef / qbarScaled,
                PitchControllerConfig::kQbarGainScaleMin,
                PitchControllerConfig::kQbarGainScaleMax);

            const double scaledNzErrorGain = PitchControllerConfig::kNzErrorGain * qbarGainScale;

            // Washed-out pitch rate blended INTO the Nz feedback before the error is formed.
            // Per Loschke: "a specified blend of washed out pitch rate and the normal acceleration".
            // Washout (tau = 1 s) ensures zero steady-state contribution so the integrator never
            // fights a persistent damping offset in sustained manoeuvres.
            const double pitchRateWashedBlend = pitchRateFeedbackWashout.Filter(resetFilters, dt, pitchRate_RAD_s);

            {
                // P+I g-command law per Loschke.
                // Pitch-rate blend damps transients via the error signal; 1/qbar scaling on the
                // proportional gain automatically scales the blend contribution across the envelope.
                const double blendedNzFeedback = nz_measured
                    + PitchControllerConfig::kPitchRateBlendGain * pitchRateWashedBlend;
                nz_error      = nz_cmd - blendedNzFeedback;
                nzProportional = scaledNzErrorGain * nz_error;
                nz_control    = nzProportional;

                // Gear-down and AAR: proportional only (no integrator) — gives classic phugoid + short period.
                // Gear-down eliminates need for WoW switching at touchdown/lift-off.
                // AAR: per Loschke, pitch law reverts to proportional-only with reduced stick gain.
                const bool gearIsDown = (pitchModeGearDown > PitchControllerConfig::kGearDownThreshold);
                const bool proportionalOnly = gearIsDown || pitchModeAirRefuelDoorOpen;

                if (proportionalOnly)
                {
                    // Freeze and bleed integrator to zero; proportional path only
                    pitchIntegratorState = pitchIntegratorState * (1.0 - limit(dt * 5.0, 0.0, 1.0));
                    integratorInput = 0.0;
                }
                else
                {
                    bool nzAtUpperLimit = (nz_measured >= PitchControllerConfig::kNzMax && nz_error > 0.0);
                    bool nzAtLowerLimit = (nz_measured <= PitchControllerConfig::kNzMin && nz_error < 0.0);

                    const double unsaturatedCommandBeforeIntegration =
                        nzProportional + pitchIntegratorState;
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
                }

                finalCombinedCommandFilteredLimited = limit(
                    nzProportional + pitchIntegratorState,
                    -integratorLimit,
                    integratorLimit);
            }

            const double integratorOutput = pitchIntegratorState;
            double finalPitchCommandTotal = pitchActuatorDynamicsFilter.Filter(resetFilters, dt, finalCombinedCommandFilteredLimited);

            // ---- Real F-117 AOA limiter: high-gain P+I, activated by AoA + washed-out pitch-rate anticipation ----
            // Threshold is Mach-scheduled (rises at high Mach; airframe more stable transonically).
            // Pitch-rate anticipation is dynamic-pressure-scheduled (stronger at low q for aggressive pull-ups).
            // When active the limiter has MORE authority than the pilot.
            {
                // Gear-down biases the limiter threshold higher (lower approach/landing speed possible).
                const bool gearIsDown = (pitchModeGearDown > PitchControllerConfig::kGearDownThreshold);
                const double machClamped = limit(mach, 0.0, 1.0);
                double limiterThreshold = PitchControllerConfig::kAoALimitStart
                                        + PitchControllerConfig::kAoALimiterMachGain * machClamped;
                if (gearIsDown)
                {
                    limiterThreshold += PitchControllerConfig::kAoALimitStartGearDownBias;
                }
                limiterThreshold = limit(limiterThreshold, PitchControllerConfig::kAoALimitStart, PitchControllerConfig::kAoAMax - 0.5);

                // Washed-out pitch rate for anticipation (q-scheduled gain)
                const double pitchRateForAnticipation = pitchRateWashoutFilter.Filter(resetFilters, dt, pitch_rate_DEG_s);
                const double qbarForAnticiption = (std::max)(dynPressure_PA, PitchControllerConfig::kQbarMin);
                const double anticipationGain = PitchControllerConfig::kAoAAnticipationGainRef
                                              * (PitchControllerConfig::kQbarRef / qbarForAnticiption);
                const double anticipation = limit(anticipationGain * pitchRateForAnticipation,
                                                  -PitchControllerConfig::kAoAAnticipationMax,
                                                   PitchControllerConfig::kAoAAnticipationMax);

                const double limiterInput = alphaFiltered + anticipation;

                if (limiterInput > limiterThreshold)
                {
                    const double limiterError = limiterInput - limiterThreshold;

                    // Integrator (clamp and freeze at limit)
                    const bool limIntSatHigh = (aoaLimiterIntegrator >= PitchControllerConfig::kAoALimiterIntegratorLimit);
                    if (!limIntSatHigh)
                    {
                        aoaLimiterIntegrator = limit(
                            aoaLimiterIntegrator + PitchControllerConfig::kAoALimiterIntegratorGain * dt * limiterError,
                            0.0, PitchControllerConfig::kAoALimiterIntegratorLimit);
                    }

                    const double limiterCommand = -(PitchControllerConfig::kAoALimiterProportionalGain * limiterError
                                                  + aoaLimiterIntegrator);
                    // Limiter overrides if it produces a stronger nose-down command
                    if (limiterCommand < finalPitchCommandTotal)
                    {
                        finalPitchCommandTotal = limiterCommand;
                    }
                }
                else
                {
                    // Decay integrator when not active so re-entry is smooth
                    aoaLimiterIntegrator = limit(aoaLimiterIntegrator - PitchControllerConfig::kAoALimiterIntegratorGain * dt * 2.0, 0.0, PitchControllerConfig::kAoALimiterIntegratorLimit);
                }
            }

            // ---- Roll-rate^2 inertia coupling (pitch axis) ----
            // Prevents inertia-coupled pitch departures during high-AoA rolling manoeuvres.
            {
                const double rollRateRadS = roll_rate_DEG_s * (kPi / 180.0);
                if (alphaFiltered > PitchControllerConfig::kRollRateSquaredAoAThreshold
                    && std::abs(rollRateRadS) > PitchControllerConfig::kRollRateSquaredRateThreshold)
                {
                    const double rollRateSq = rollRateRadS * rollRateRadS;
                    finalPitchCommandTotal -= PitchControllerConfig::kRollRateSquaredGain * rollRateSq;
                }
            }

            // ---- Speed stability (below 200 kts, AoA > 7 deg) ----
            // Provides apparent speed stability on approach: nose-down tendency as speed falls.
            {
                if (velocity_mps < PitchControllerConfig::kSpeedStabilityVelocityMPS
                    && alphaFiltered > PitchControllerConfig::kSpeedStabilityAoAThreshold)
                {
                    const double speedFade = 1.0 - velocity_mps / PitchControllerConfig::kSpeedStabilityVelocityMPS;
                    const double aoaExcess = alphaFiltered - PitchControllerConfig::kSpeedStabilityAoAThreshold;
                    finalPitchCommandTotal -= PitchControllerConfig::kSpeedStabilityGain * aoaExcess * speedFade;
                }
            }

            log_pitch_debug_sample(
                roll_angle_DEG,
                pitch_angle_DEG,
                roll_rate_DEG_s,
                az,
                gravity_component,
                nz_measured,
                nz_reference,
                stickCommandPos,
                nz_cmd,
                nz_error,
                nzProportional,
                PitchControllerConfig::kPitchRateBlendGain * pitchRateWashedBlend,  // pitch_rate_blend
                nz_control,
                integratorOutput,
                angle_of_attack_ind,
                alphaFiltered,
                longStickInputCommand,
                pitch_rate_DEG_s,
                dynPressure_PA,
                velocity_mps,
                mach,
                thrust_N,
                Cx_total,
                finalPitchCommandTotal);

            return finalPitchCommandTotal;
        }

        double fcs_pitch_controller(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind, double pitch_rate_DEG_s, double az, double differentialCommand, double dynPressure_PA, double dt, double roll_angle_DEG, double pitch_angle_DEG, double roll_rate_DEG_s, double velocity_mps, double mach, double thrust_N, double Cx_total)
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
                    dynPressure_PA,
                    kFixedControlTimeStep,
                    roll_angle_DEG,
                    pitch_angle_DEG,
                    roll_rate_DEG_s,
                    velocity_mps,
                    mach,
                    thrust_N,
                    Cx_total,
                    resetFilters);
                resetFilters = false;
                pitchControlAccumulator -= kFixedControlTimeStep;
            }

            return pitchHeldOutput;
        }
        double fcs_roll_controller_step(double latStickInputCommand, double longStickForceCommand, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_PA, bool gearDown, bool airRefuelDoorOpen, double dt, bool resetFilters)
        {
            // Air refuelling: lateral stick gain reduced for control harmony with reduced pitch gain.
            const double effectiveLateralStick = airRefuelDoorOpen
                ? latStickInputCommand * RollControllerConfig::kAARLateralStickGainScale
                : latStickInputCommand;

            double latStickForceCmd = effectiveLateralStick * RollControllerConfig::kLatStickForceScale;
            double latStickForce = latStickForceFilter.Filter(resetFilters, dt, latStickForceCmd);
            double ayBiasBlend = limit(std::abs(effectiveLateralStick) / RollControllerConfig::kAyBiasFullStick, 0.0, 1.0);
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
            // Parabolic stick-force-to-roll-rate shaping.
            // rollRateCmd = sign * K * (|force| - deadband)^2, clamped to ±kRollRateMax.
            // This matches the real F-117's non-linear parabolic shaping: gentle onset at centre
            // (good precision) with rapidly increasing authority toward full deflection (good jinking).
            double rollRateCommand = 0.0;
            if (std::abs(latStickForceFinal) > RollControllerConfig::kStickDeadband)
            {
                const double sign = (latStickForceFinal > 0.0) ? 1.0 : -1.0;
                const double forceBeyondDeadband = std::abs(latStickForceFinal) - RollControllerConfig::kStickDeadband;
                rollRateCommand = sign * RollControllerConfig::kRollParabolicGain * forceBeyondDeadband * forceBeyondDeadband;
                rollRateCommand = limit(rollRateCommand, -RollControllerConfig::kRollRateMax, RollControllerConfig::kRollRateMax);
            }

            double flatTurnPedalBlend = limit(std::abs(pedInput) / RollControllerConfig::kFlatTurnPedalFull, 0.0, 1.0);
            double flatTurnStickBlend = 1.0 - limit(std::abs(effectiveLateralStick) / RollControllerConfig::kFlatTurnLatStickFade, 0.0, 1.0);
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

            double pressureGain = 0.0;
            if (dynPressure_PA < RollControllerConfig::kPressureLow)
            {
                pressureGain = RollControllerConfig::kPressureLowGain;
            }
            else if ((dynPressure_PA >= RollControllerConfig::kPressureLow) && (dynPressure_PA <= RollControllerConfig::kPressureHigh))
            {
                pressureGain = RollControllerConfig::kPressureSlope * dynPressure_PA + RollControllerConfig::kPressureOffset;
            }
            else
            {
                pressureGain = RollControllerConfig::kPressureHighGain;
            }

            // Gear down: increase roll rate feedback gain to reduce response to turbulence on approach.
            if (gearDown)
            {
                pressureGain *= RollControllerConfig::kGearDownRateFeedbackGainFactor;
            }

            double rollCommandGained = limit(rollRateCommandCombined * pressureGain, -RollControllerConfig::kRollCommandLimit, RollControllerConfig::kRollCommandLimit);
            double rollSurfaceCommand = rollActuatorDynamicsFilter.Filter(resetFilters, dt, rollCommandGained);

            lastRollStickInput = effectiveLateralStick;
            lastRollStickForce = latStickForceFinal;
            lastRollRateCommand = rollRateCommand;
            lastRollFlatTurnCommand = flatTurnRollCommand;
            lastRollRateCommandFiltered = rollRateCommandFiltered;
            lastRollCommandGained = rollCommandGained;
            lastRollSurfaceCommand = rollSurfaceCommand;

            return rollSurfaceCommand;
        }

        double fcs_roll_controller(double latStickInputCommand, double longStickForceCommand, double ay, double pedInput, double beta_deg, double roll_angle_deg, double roll_rate, double roll_rate_trim, double dynPressure_PA, bool gearDown, bool airRefuelDoorOpen, double dt)
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
                    dynPressure_PA,
                    gearDown,
                    airRefuelDoorOpen,
                    kFixedControlTimeStep,
                    resetFilters);
                resetFilters = false;
                rollControlAccumulator -= kFixedControlTimeStep;
            }

            return rollHeldOutput;
        }
    }
}







