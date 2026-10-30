#pragma once
#include <cmath>

// Pilot Activated Automatic Recovery System, TO 1F-117A-1A PDF pages 19-20
// (printed 4-6, 4-7, Figure 4-3). Pressing the stick ARS switch returns the aircraft to
// wings level with a small positive pitch attitude. The response depends on the sector of
// the PAARS Response Map containing the current pitch and bank. The pilot can always
// overpower it with the stick. A second ARS press or the disengage paddle deselects it.
namespace Paars
{
    // Figure 4-3 sectors.
    enum Sector
    {
        kPullRollInverted = 1,       // nose high, steep bank or inverted
        kPushRollUpright = 2,        // nose high, near upright
        kRollUpright = 3,            // near level, banked or inverted
        kPullRollUpright = 4,        // nose low (or level), upright side
        kPushRollUprightInverted = 5,// nose low, inverted side
        kPullZeroRollNoseDown = 6,   // near vertical dive
        kPullZeroRollNoseUp = 7      // near vertical climb
    };

    // Sector boundaries digitised from Figure 4-3 (symmetric in bank angle).
    constexpr double kVerticalClimbDeg = 83.0;
    constexpr double kVerticalDiveDeg = -70.0;
    constexpr double kLevelBandTopDeg = 5.5;
    constexpr double kLevelBandHalfWidthDeg = 21.0;
    constexpr double kNoseLowFloorDeg = -15.0;
    constexpr double kNoseLowFloorBankDeg = 60.0;

    // Simulator choices; the manual only says "small positive pitch attitude" and that
    // retracting the gear reduces pitch by approximately four degrees.
    constexpr double kTargetPitchGearUpDeg = 5.0;
    constexpr double kTargetPitchGearDownDeg = 9.0;
    constexpr double kPullNz = 3.0;
    constexpr double kVerticalPullNz = 4.0;
    constexpr double kInvertedPushNz = -1.0;
    constexpr double kAttitudeNzMin = 0.0;
    constexpr double kAttitudeNzMax = 4.0;
    constexpr double kPitchGainNzPerDeg = 0.08;
    constexpr double kPitchRateDampingNzPerDegS = 0.05;
    constexpr double kRollGainPerS = 2.0;
    constexpr double kRollRateMaxDegS = 90.0;
    constexpr double kPulloutNz = 2.8;  // assumed for the minimum pullout altitude estimate

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;

    inline double wrap_deg(double angle)
    {
        angle = std::fmod(angle, 360.0);
        if (angle > 180.0) angle -= 360.0;
        if (angle <= -180.0) angle += 360.0;
        return angle;
    }

    // Upper edge of sectors 3/4 (flat at 5.5 deg near wings level, rising to 31 deg inverted).
    inline double upper_boundary_deg(double absBankDeg)
    {
        if (absBankDeg <= kLevelBandHalfWidthDeg) return kLevelBandTopDeg;
        return kLevelBandTopDeg + (absBankDeg - kLevelBandHalfWidthDeg) * (25.5 / 159.0);
    }

    // Lower edge of sector 3 (diagonal from the level band to -15 deg at 60 deg bank).
    inline double lower_boundary_deg(double absBankDeg)
    {
        if (absBankDeg <= kLevelBandHalfWidthDeg) return kLevelBandTopDeg;
        if (absBankDeg >= kNoseLowFloorBankDeg) return kNoseLowFloorDeg;
        return kLevelBandTopDeg - (absBankDeg - kLevelBandHalfWidthDeg)
            * ((kLevelBandTopDeg - kNoseLowFloorDeg) / (kNoseLowFloorBankDeg - kLevelBandHalfWidthDeg));
    }

    // Edge between sectors 1 and 2: from (37 deg bank, 83 deg pitch) to (104, 19).
    inline double sector2_bank_limit_deg(double pitchDeg)
    {
        return 37.0 + (kVerticalClimbDeg - pitchDeg) * (67.0 / 64.0);
    }

    inline int classify_sector(double pitchDeg, double bankDeg)
    {
        const double absBank = std::fabs(wrap_deg(bankDeg));
        if (pitchDeg >= kVerticalClimbDeg) return kPullZeroRollNoseUp;
        if (pitchDeg <= kVerticalDiveDeg) return kPullZeroRollNoseDown;
        if (pitchDeg > upper_boundary_deg(absBank))
        {
            return absBank < sector2_bank_limit_deg(pitchDeg) ? kPushRollUpright : kPullRollInverted;
        }
        if (pitchDeg >= lower_boundary_deg(absBank) && absBank > kLevelBandHalfWidthDeg)
        {
            return kRollUpright;
        }
        return absBank < 90.0 ? kPullRollUpright : kPushRollUprightInverted;
    }

    // Not functional (PDF 19): A/R door open with |bank| < 45 and pitch -20..+25,
    // or weight on wheels with |bank| < 20 and pitch -10..+15.
    inline bool inhibited(double pitchDeg, double bankDeg, bool weightOnWheels, bool airRefuelDoorOpen)
    {
        const double absBank = std::fabs(wrap_deg(bankDeg));
        if (airRefuelDoorOpen && absBank < 45.0 && pitchDeg > -20.0 && pitchDeg < 25.0) return true;
        if (weightOnWheels && absBank < 20.0 && pitchDeg > -10.0 && pitchDeg < 15.0) return true;
        return false;
    }

    struct Command
    {
        int sector;
        double nz;            // commanded normal load factor (1 = level flight)
        double rollRateDegS;  // commanded roll rate, positive right wing down
    };

    inline double clamp_value(double value, double low, double high)
    {
        return value < low ? low : (value > high ? high : value);
    }

    inline double roll_rate_to(double bankErrorDeg)
    {
        return clamp_value(kRollGainPerS * bankErrorDeg, -kRollRateMaxDegS, kRollRateMaxDegS);
    }

    inline Command compute_command(double pitchDeg, double bankDeg, double rollRateDegS, double pitchRateDegS, bool gearDown)
    {
        const double bank = wrap_deg(bankDeg);
        const double gravityNz = std::cos(bank * kDegToRad) * std::cos(pitchDeg * kDegToRad);
        const double targetPitch = gearDown ? kTargetPitchGearDownDeg : kTargetPitchGearUpDeg;
        const double attitudeNz = clamp_value(
            gravityNz + kPitchGainNzPerDeg * (targetPitch - pitchDeg) - kPitchRateDampingNzPerDegS * pitchRateDegS,
            kAttitudeNzMin, kAttitudeNzMax);

        // Roll upright the short way; when exactly inverted keep any existing roll direction.
        double uprightError = -bank;
        if (std::fabs(bank) > 179.0 && std::fabs(rollRateDegS) > 1.0)
        {
            uprightError = rollRateDegS > 0.0 ? 180.0 : -180.0;
        }
        const double invertedError = (bank >= 0.0 ? 180.0 : -180.0) - bank;

        Command command{classify_sector(pitchDeg, bank), 1.0, 0.0};
        switch (command.sector)
        {
        case kPullRollInverted:
            command.nz = kPullNz;
            command.rollRateDegS = roll_rate_to(invertedError);
            break;
        case kPushRollUpright:
        case kPullRollUpright:
            command.nz = attitudeNz;
            command.rollRateDegS = roll_rate_to(uprightError);
            break;
        case kRollUpright:
            command.nz = gravityNz; // hold the flight path while rolling
            command.rollRateDegS = roll_rate_to(uprightError);
            break;
        case kPushRollUprightInverted:
            command.nz = kInvertedPushNz;
            command.rollRateDegS = roll_rate_to(uprightError);
            break;
        case kPullZeroRollNoseDown:
        case kPullZeroRollNoseUp:
            command.nz = kVerticalPullNz;
            command.rollRateDegS = 0.0;
            break;
        }
        return command;
    }

    // Estimate of the lowest altitude reached during recovery (the manual's NIAC
    // computation is not given): descent while rolling upright, then a constant-g pull
    // from the current flight path angle to level.
    inline double minimum_pullout_altitude_ft(double altitudeFt, double tasMps, double flightPathDeg, double bankDeg)
    {
        constexpr double kGravity = 9.81;
        constexpr double kFeetPerMetre = 3.28084;
        if (!(tasMps > 0.0) || flightPathDeg >= 0.0) return altitudeFt;
        const double gamma = flightPathDeg * kDegToRad;
        const double rollTimeS = std::fabs(wrap_deg(bankDeg)) / kRollRateMaxDegS + 1.0; // + g onset
        const double rollLossM = tasMps * -std::sin(gamma) * rollTimeS;
        const double pullLossM = tasMps * tasMps * (1.0 - std::cos(gamma)) / (kGravity * (kPulloutNz - 1.0));
        return altitudeFt - (rollLossM + pullLossM) * kFeetPerMetre;
    }

    // Displayed value: minimum pullout altitude to the next lower thousand feet.
    inline double displayed_pullout_altitude_ft(double minimumFt)
    {
        return std::floor(minimumFt / 1000.0) * 1000.0;
    }

    class Controller
    {
    public:
        bool engaged() const { return engaged_; }

        // Stick ARS switch, first press.
        bool engage(double pitchDeg, double bankDeg, bool weightOnWheels, bool airRefuelDoorOpen)
        {
            if (engaged_ || inhibited(pitchDeg, bankDeg, weightOnWheels, airRefuelDoorOpen)) return false;
            engaged_ = true;
            return true;
        }

        // Second ARS press or the disengage paddle.
        void disengage() { engaged_ = false; }

        // Returns false (and disengages) when PAARS is not functional.
        bool update(double pitchDeg, double bankDeg, double rollRateDegS, double pitchRateDegS,
            bool gearDown, bool weightOnWheels, bool airRefuelDoorOpen, Command* command)
        {
            if (!engaged_) return false;
            if (inhibited(pitchDeg, bankDeg, weightOnWheels, airRefuelDoorOpen))
            {
                engaged_ = false;
                return false;
            }
            *command = compute_command(pitchDeg, bankDeg, rollRateDegS, pitchRateDegS, gearDown);
            return true;
        }

        void reset() { engaged_ = false; }

    private:
        bool engaged_ = false;
    };
}
