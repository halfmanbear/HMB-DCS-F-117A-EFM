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

            GeneralFilter pitchRateWashout;
            GeneralFilter pitchIntegrator;
            GeneralFilter pitchPreActuatorFilter;
            GeneralFilter pitchActuatorDynamicsFilter;
            GeneralFilter accelFilter;
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

            constexpr double kPi = 3.14159265358979323846;
            constexpr int kPitchDebugLogIntervalFrames = 100;

            namespace YawControllerConfig
            {
                constexpr double kPedalForceScale = 450.0;
                constexpr double kPedalDeadbandForce = 44.0;
                constexpr double kPedalCommandSlope = -0.0739;
                constexpr double kPedalCommandOffset = 3.2512;
                constexpr double kRudderLimitDeg = 30.0;
                constexpr double kYawAlphaCouplingGain = 5.0 / 57.3;
                constexpr double kYawDampingGain = 1.5;
                constexpr double kSideAccelFeedbackGain = 0.5;
                constexpr double kAriAoAGain = 0.05;
                constexpr double kAriLimit = 1.5;
                constexpr double kCommandFilterRate = 4.0;
                constexpr double kWashoutTauInverse = 1.0;
                constexpr double kYawLeadLagNumerator0 = 3.0;
                constexpr double kYawLeadLagPole = 15.0;
                constexpr double kServoNaturalFrequency = 52.0;
                constexpr double kServoDampingRatio = 0.7;
            }

            namespace PitchControllerConfig
            {
                constexpr double kPositiveStickForceScale = 80.0;
                constexpr double kNegativeStickForceScale = 180.0;
                constexpr double kStickForceMin = -180.0;
                constexpr double kStickForceMax = 80.0;
                constexpr double kStickCommandDeadband = 8.0;
                constexpr double kStickCommandBreakpoint = 33.0;
                constexpr double kStickCommandRate = 4.0;
                constexpr double kNzCommandMin = -4.0;
                constexpr double kNzCommandMax = 8.0;
                constexpr double kPitchIntegratorGain = 2.5;
                constexpr double kPitchPreActuatorNumerator0 = 3.0;
                constexpr double kPitchPreActuatorPole = 15.0;
                constexpr double kAccelFilterPole = 15.0;
                constexpr double kAoAFilterRate = 35.0;
                constexpr double kNzMax = 6.3;
                constexpr double kNzMin = -3.0;
                constexpr double kNzStickDeadband = 0.05;
                constexpr double kNzReferenceReturnRate = 2.0;
                constexpr double kPitchDampingGain = -2.5;
                constexpr double kNzErrorGain = 8.0;
                constexpr double kPitchIntegratorLimit = 25.0;
                constexpr double kAoAMax = 20.0;
                constexpr double kAoALimitStart = 19.0;
                constexpr double kAoAPushdownGain = 0.30;
                constexpr double kPitchServoNaturalFrequency = 52.0;
                constexpr double kPitchServoDampingRatio = 0.7;
            }

            namespace RollControllerConfig
            {
                constexpr double kLatStickForceScale = 75.0;
                constexpr double kAyBiasGain = 8.9;
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
                constexpr double kRollCommandFilterPole = 10.0;
                constexpr double kRollServoNaturalFrequency = 52.0;
                constexpr double kRollServoDampingRatio = 0.7;
                constexpr double kRollRateFilterPole = 50.0;
                constexpr double kRollRateFilter2Num0 = 4.0;
                constexpr double kRollRateFilter2Num1 = 64.0;
                constexpr double kRollRateFilter2Den1 = 80.0;
                constexpr double kRollRateFilter2OmegaSquared = 6400.0;
                constexpr double kDynPressureLbFt2ToNm2 = 47.880258889;
            }

            inline bool filters_need_init()
            {
                return !simInitialized;
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
                debugLog << "roll_deg,pitch_deg,az_raw,gravity_comp,nz_measured,nz_reference,stickCmdPos,nz_cmd,nz_error,alpha_raw,alphaFiltered,stickInput,pitchRate,dynPressure,velocity_fps,mach,thrust_N,Cx_total,elevatorOut\n";
            }

            void open_pitch_debug_log()
            {
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
                double rawAz,
                double gravityComponent,
                double nzMeasured,
                double nzReference,
                double stickCommandPos,
                double nzCommand,
                double nzError,
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
                         << rawAz << ","
                         << gravityComponent << ","
                         << nzMeasured << ","
                         << nzReference << ","
                         << stickCommandPos << ","
                         << nzCommand << ","
                         << nzError << ","
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
                debugLog.flush();
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
            }

            void init_pitch_filters(double dt)
            {
                double numerators[2] = { 1.0, 0.0 };
                double denominators[2] = { 1.0, 1.0 };
                pitchRateWashout.InitFilter(numerators, denominators, 1, dt);

                numerators[0] = 0.0;
                numerators[1] = PitchControllerConfig::kPitchIntegratorGain;
                denominators[0] = 1.0;
                denominators[1] = 0.0;
                pitchIntegrator.InitFilter(numerators, denominators, 1, dt);

                numerators[0] = PitchControllerConfig::kPitchPreActuatorNumerator0;
                numerators[1] = PitchControllerConfig::kPitchPreActuatorPole;
                denominators[0] = 1.0;
                denominators[1] = PitchControllerConfig::kPitchPreActuatorPole;
                pitchPreActuatorFilter.InitFilter(numerators, denominators, 1, dt);

                const double servoOmegaSquared = std::pow(PitchControllerConfig::kPitchServoNaturalFrequency, 2.0);
                double numerators2[3] = { 0.0, 0.0, servoOmegaSquared };
                double denominators2[3] = { 1.0, 2.0 * PitchControllerConfig::kPitchServoDampingRatio * PitchControllerConfig::kPitchServoNaturalFrequency, servoOmegaSquared };
                pitchActuatorDynamicsFilter.InitFilter(numerators2, denominators2, 2, dt);

                numerators[0] = 0.0;
                numerators[1] = PitchControllerConfig::kAccelFilterPole;
                denominators[0] = 1.0;
                denominators[1] = PitchControllerConfig::kAccelFilterPole;
                accelFilter.InitFilter(numerators, denominators, 1, dt);

                stickCommandPosFiltered = 0.0;
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
            }
        }

        double fcs_yaw_controller(double pedInput, double pedTrim, double yaw_rate, double roll_rate, double aoa_filtered, double aileron_commanded, double ay, double dt)
        {
            if (filters_need_init())
            {
                init_yaw_filters(dt);
            }

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
            double rudderCommandFiltered = rudderCommandFilter.Filter(filters_need_init(), dt, rudderCommand);
            double pedalCommand = pedTrim - rudderCommandFiltered;

            double alphaGained = aoa_filtered * YawControllerConfig::kYawAlphaCouplingGain;
            double rollRateCoupling = roll_rate * alphaGained;
            double yawRateCorrected = yaw_rate - rollRateCoupling;

            double yawRateWashedOut = yawRateWashout.Filter(filters_need_init(), dt, yawRateCorrected);
            double yawRateSmoothed = yawRateFilter.Filter(filters_need_init(), dt, yawRateWashedOut);
            double yawDamping = YawControllerConfig::kYawDampingGain * yawRateSmoothed;

            double sideAccelFeedback = YawControllerConfig::kSideAccelFeedbackGain * ay;
            double ariCommand = limit(YawControllerConfig::kAriAoAGain * aoa_filtered, 0.0, YawControllerConfig::kAriLimit) * aileron_commanded;
            double combinedCommand = pedalCommand + yawDamping + sideAccelFeedback + ariCommand;

            return yawServoFilter.Filter(filters_need_init(), dt, combinedCommand);
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
                longStickCommand_G = (0.016 * longStickInputForce) + 0.128;
            }
            else if (longStickInputForce <= -PitchControllerConfig::kStickCommandBreakpoint)
            {
                longStickCommand_G = (0.067 * longStickInputForce) + 1.8112;
            }
            else if ((longStickInputForce > PitchControllerConfig::kStickCommandDeadband) && (longStickInputForce < PitchControllerConfig::kStickCommandBreakpoint))
            {
                longStickCommand_G = (0.032 * longStickInputForce) - 0.256;
            }
            else if (longStickInputForce >= PitchControllerConfig::kStickCommandBreakpoint)
            {
                longStickCommand_G = 0.0681 * longStickInputForce - 1.4468;
            }

            double longStickCommandWithTrim_G = pitchTrim - longStickCommand_G;
            double longStickCommandWithTrimLimited_G = limit(longStickCommandWithTrim_G, PitchControllerConfig::kNzCommandMin, PitchControllerConfig::kNzCommandMax);
            double longStickCommandWithTrimLimited_G_Rate = PitchControllerConfig::kStickCommandRate * (longStickCommandWithTrimLimited_G - stickCommandPosFiltered);
            stickCommandPosFiltered += longStickCommandWithTrimLimited_G_Rate * dt;

            return stickCommandPosFiltered;
        }

        double dynamic_pressure_schedule(double dynPressure_LBFT2)
        {
            double dynamicPressure_kNM2 = dynPressure_LBFT2 * 1.4881639 / 1000.0;
            double scheduleOutput = 0.0;

            if (dynamicPressure_kNM2 < 9.576)
            {
                scheduleOutput = 1.0;
            }
            else if ((dynamicPressure_kNM2 >= 9.576) && (dynamicPressure_kNM2 <= 43.0))
            {
                scheduleOutput = (-0.018 * dynamicPressure_kNM2) + 1.1719;
            }
            else if (dynamicPressure_kNM2 > 43.0)
            {
                scheduleOutput = -0.003 * dynamicPressure_kNM2 + 0.5277;
            }

            return limit(scheduleOutput, 0.5, 1.0);
        }

        double fcs_pitch_controller(double longStickInputCommand, double pitchTrim, double angle_of_attack_ind, double pitch_rate, double az, double differentialCommand, double dynPressure_LBFT2, double dt, double roll_angle_DEG, double pitch_angle_DEG, double velocity_fps, double mach, double thrust_N, double Cx_total)
        {
            (void)differentialCommand;
            if (filters_need_init())
            {
                init_pitch_filters(dt);
            }

            double stickCommandPos = fcs_pitch_controller_force_command(longStickInputCommand, pitchTrim, dt);
            double roll_RAD = roll_angle_DEG * (kPi / 180.0);
            double pitch_RAD = pitch_angle_DEG * (kPi / 180.0);

            azFiltered = accelFilter.Filter(filters_need_init(), dt, az);

            double alphaLimited = limit(angle_of_attack_ind, -5.0, PitchControllerConfig::kAoAMax);
            double alphaLimitedRate = PitchControllerConfig::kAoAFilterRate * (alphaLimited - alphaFiltered);
            alphaFiltered += alphaLimitedRate * dt;

            double pitchRateWashedOut = pitchRateWashout.Filter(filters_need_init(), dt, pitch_rate);
            double gravity_component = std::cos(roll_RAD) * std::cos(pitch_RAD);
            double nz_measured = azFiltered + gravity_component;

            static double nz_reference = 1.0;
            if (std::fabs(stickCommandPos) < PitchControllerConfig::kNzStickDeadband)
            {
                nz_reference += (1.0 - nz_reference) * PitchControllerConfig::kNzReferenceReturnRate * dt;
            }

            double nz_cmd = limit(nz_reference + stickCommandPos, PitchControllerConfig::kNzMin, PitchControllerConfig::kNzMax);
            double nz_error = nz_cmd - nz_measured;
            double pitchDamping = PitchControllerConfig::kPitchDampingGain * pitchRateWashedOut;
            double nz_control = (PitchControllerConfig::kNzErrorGain * nz_error) + pitchDamping;

            bool nzAtUpperLimit = (nz_measured >= PitchControllerConfig::kNzMax && nz_error > 0.0);
            bool nzAtLowerLimit = (nz_measured <= PitchControllerConfig::kNzMin && nz_error < 0.0);
            double integratorInput = (nzAtUpperLimit || nzAtLowerLimit) ? 0.0 : nz_control;

            double finalCombinedCommandFilteredLimited = limit(
                pitchIntegrator.Filter(filters_need_init(), dt, integratorInput),
                -PitchControllerConfig::kPitchIntegratorLimit,
                PitchControllerConfig::kPitchIntegratorLimit);

            double finalPitchCommandTotal = pitchPreActuatorFilter.Filter(
                filters_need_init(),
                dt,
                finalCombinedCommandFilteredLimited);

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
                az,
                gravity_component,
                nz_measured,
                nz_reference,
                stickCommandPos,
                nz_cmd,
                nz_error,
                angle_of_attack_ind,
                alphaFiltered,
                longStickInputCommand,
                pitch_rate,
                dynPressure_LBFT2,
                velocity_fps,
                mach,
                thrust_N,
                Cx_total,
                finalPitchCommandTotal);

            return finalPitchCommandTotal;
        }

        double fcs_roll_controller(double latStickInputCommand, double longStickForceCommand, double ay, double roll_rate, double roll_rate_trim, double dynPressure_LBFT2, double dt)
        {
            if (filters_need_init())
            {
                init_roll_filters(dt);
            }

            double latStickForceCmd = latStickInputCommand * RollControllerConfig::kLatStickForceScale;
            double latStickForce = latStickForceFilter.Filter(filters_need_init(), dt, latStickForceCmd);
            double latStickForceBiased = latStickForce - (ay * RollControllerConfig::kAyBiasGain);

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
                rollRateCommand = 0.9091 * latStickForceFinal - 2.7273;
            }
            else if ((latStickForceFinal > RollControllerConfig::kFirstBreakpoint) && (latStickForceFinal <= RollControllerConfig::kSecondBreakpoint))
            {
                rollRateCommand = 2.8571 * latStickForceFinal - 51.429;
            }
            else if (latStickForceFinal > RollControllerConfig::kSecondBreakpoint)
            {
                rollRateCommand = 7.5862 * latStickForceFinal - 268.97;
            }
            else if ((latStickForceFinal <= -RollControllerConfig::kStickDeadband) && (latStickForceFinal >= -RollControllerConfig::kFirstBreakpoint))
            {
                rollRateCommand = 0.9091 * latStickForceFinal + 2.7273;
            }
            else if ((latStickForceFinal < -RollControllerConfig::kFirstBreakpoint) && (latStickForceFinal >= -RollControllerConfig::kSecondBreakpoint))
            {
                rollRateCommand = 2.8571 * latStickForceFinal + 51.429;
            }
            else if (latStickForceFinal < -RollControllerConfig::kSecondBreakpoint)
            {
                rollRateCommand = 7.5862 * latStickForceFinal + 268.97;
            }

            double rollRateCommandFiltered = rollCommandFilter.Filter(filters_need_init(), dt, rollRateCommand);
            double rollRateFiltered1 = rollRateFilter1.Filter(filters_need_init(), dt, roll_rate);
            double rollRateFiltered2 = rollRateFilter2.Filter(filters_need_init(), dt, rollRateFiltered1);
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
            return rollActuatorDynamicsFilter.Filter(filters_need_init(), dt, rollCommandGained);
        }
    }
}
