#pragma once
#include <cmath>
#include <limits>

// OCIP autothrottle, TO 1F-117A-1A PDF pages 15-17 and 52 (printed 4-2 to 4-4, 4-39).
// The A/T engage switch, with weight off the gear, engages CAS HOLD at the current CAS.
// Disengagement by any means lights A/T DISC; a second press of the throttle disengage
// button or the stick paddle extinguishes it. Mach, GS and AOA hold and CAS select are
// entered on the DEP, which is not implemented, so only CAS hold is reachable here.
namespace Autothrottle
{
    constexpr double kKnotsPerMps = 1.0 / 0.514444;
    constexpr double kCasMinKt = 140.0; // DEP keypad entry limits, Figure FO-3
    constexpr double kCasMaxKt = 525.0;

    // Simulator choices; the manual gives no values for these.
    constexpr double kServoRatePctPerS = 15.0;   // power lever servo rate
    constexpr double kAxisOverridePct = 5.0;     // axis travel that overpowers an engaged A/T
    constexpr double kAxisPickupPct = 2.0;       // axis travel that takes back a held lever
    constexpr double kProportionalPctPerKt = 2.0;
    constexpr double kIntegralPctPerKtS = 0.25;
    constexpr double kAccelDampingPctPerKtS = 6.0;
    constexpr double kAccelFilterTauS = 0.5;
    constexpr double kIntegrateBandKt = 5.0;     // integrate only near the target speed

    // The EFM builds as C++14, which has no std::clamp.
    inline double clamp_value(double value, double low, double high)
    {
        return value < low ? low : (value > high ? high : value);
    }

    // Subsonic pitot relation: CAS from true airspeed and static conditions.
    inline double calibrated_airspeed_kt(double tasMps, double temperatureK, double staticPressurePa)
    {
        constexpr double kGamma = 1.4;
        constexpr double kGasConstant = 287.058;
        constexpr double kSeaLevelPressurePa = 101325.0;
        constexpr double kSeaLevelSoundMps = 340.294;
        if (!(tasMps > 0.0) || !(temperatureK > 0.0) || !(staticPressurePa > 0.0))
        {
            return 0.0;
        }
        const double mach = tasMps / std::sqrt(kGamma * kGasConstant * temperatureK);
        const double impactPressurePa = staticPressurePa * (std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0);
        const double casMps = kSeaLevelSoundMps
            * std::sqrt(5.0 * (std::pow(impactPressurePa / kSeaLevelPressurePa + 1.0, 2.0 / 7.0) - 1.0));
        return casMps * kKnotsPerMps;
    }

    class Controller
    {
    public:
        bool engaged() const { return engaged_; }
        bool disconnect_light() const { return disconnectLight_; }
        double target_cas_kt() const { return targetCasKt_; }

        // A/T engage switch. It springs back (returns false) on the ground or with engines stopped.
        bool engage(double casKt, double throttlePct, bool airborne, bool engineRunning)
        {
            if (engaged_ || !airborne || !engineRunning || !std::isfinite(casKt) || !std::isfinite(throttlePct))
            {
                return false;
            }
            engaged_ = true;
            disconnectLight_ = false;
            holding_ = false;
            targetCasKt_ = clamp_value(casKt, kCasMinKt, kCasMaxKt);
            leverPct_ = clamp_value(throttlePct, 0.0, 100.0);
            integralPct_ = leverPct_;
            accelKtS_ = 0.0;
            havePreviousCas_ = false;
            return true;
        }

        void select_cas(double casKt)
        {
            if (std::isfinite(casKt))
            {
                targetCasKt_ = clamp_value(casKt, kCasMinKt, kCasMaxKt);
            }
        }

        // Disengagement by any means lights A/T DISC. The lever stays where the servo left it.
        void disengage()
        {
            if (engaged_)
            {
                engaged_ = false;
                disconnectLight_ = true;
                holding_ = true;
            }
        }

        // Throttle disengage button and stick paddle: first press disengages,
        // the next press extinguishes A/T DISC.
        void disconnect_button()
        {
            if (engaged_)
            {
                disengage();
            }
            else
            {
                disconnectLight_ = false;
            }
        }

        // Physical throttle axis. Returns true when the axis should set the throttle.
        // A fixed axis cannot be driven by the servo, so deliberate travel overpowers
        // an engaged A/T, and a lever held after disengagement waits for axis movement.
        bool accept_axis(double axisPct)
        {
            if (!std::isfinite(axisPct))
            {
                return false;
            }
            if (!std::isfinite(axisReferencePct_))
            {
                axisReferencePct_ = axisPct;
                return !engaged_ && !holding_;
            }
            if (engaged_)
            {
                if (std::fabs(axisPct - axisReferencePct_) <= kAxisOverridePct)
                {
                    return false;
                }
                disengage();
            }
            else if (holding_ && std::fabs(axisPct - axisReferencePct_) <= kAxisPickupPct)
            {
                return false;
            }
            holding_ = false;
            axisReferencePct_ = axisPct;
            return true;
        }

        // Keyboard throttle steps act on the lever directly.
        void pilot_throttle_step()
        {
            disengage();
            holding_ = false;
        }

        // Returns the throttle command; unchanged when not engaged.
        double update(double dt, double casKt, double throttlePct, bool airborne, bool engineRunning)
        {
            if (!engaged_)
            {
                return throttlePct;
            }
            if (!airborne || !engineRunning || !std::isfinite(casKt))
            {
                disengage();
                return throttlePct;
            }
            if (!(dt > 0.0))
            {
                return leverPct_;
            }

            if (havePreviousCas_)
            {
                const double rawAccel = (casKt - previousCasKt_) / dt;
                accelKtS_ += (rawAccel - accelKtS_) * (1.0 - std::exp(-dt / kAccelFilterTauS));
            }
            previousCasKt_ = casKt;
            havePreviousCas_ = true;

            const double errorKt = targetCasKt_ - casKt;
            const double feedbackPct = kProportionalPctPerKt * errorKt - kAccelDampingPctPerKtS * accelKtS_;
            const double unclampedPct = integralPct_ + feedbackPct;
            const bool pushingHigh = unclampedPct >= 100.0 && errorKt > 0.0;
            const bool pushingLow = unclampedPct <= 0.0 && errorKt < 0.0;
            if (!pushingHigh && !pushingLow && std::fabs(errorKt) < kIntegrateBandKt)
            {
                integralPct_ = clamp_value(integralPct_ + kIntegralPctPerKtS * errorKt * dt, 0.0, 100.0);
            }

            const double desiredPct = clamp_value(integralPct_ + feedbackPct, 0.0, 100.0);
            const double stepPct = kServoRatePctPerS * dt;
            leverPct_ = clamp_value(desiredPct, leverPct_ - stepPct, leverPct_ + stepPct);
            return leverPct_;
        }

        void reset() { *this = Controller(); }

    private:
        bool engaged_ = false;
        bool disconnectLight_ = false;
        bool holding_ = false;
        bool havePreviousCas_ = false;
        double targetCasKt_ = 0.0;
        double leverPct_ = 0.0;
        double integralPct_ = 0.0;
        double accelKtS_ = 0.0;
        double previousCasKt_ = 0.0;
        double axisReferencePct_ = std::numeric_limits<double>::quiet_NaN();
    };
}
