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
constexpr double knots_per_mps = 1.0 / 0.514444;
constexpr double cas_min_kt = 140.0;  // DEP keypad entry limits, Figure FO-3
constexpr double cas_max_kt = 525.0;

// Simulator choices; the manual gives no values for these.
constexpr double servo_rate_pct_per_s = 15.0;  // power lever servo rate
constexpr double axis_override_pct = 5.0;      // axis travel that overpowers an engaged A/T
constexpr double axis_pickup_pct = 2.0;        // axis travel that takes back a held lever
constexpr double proportional_pct_per_kt = 2.0;
constexpr double integral_pct_per_kt_s = 0.25;
constexpr double accel_damping_pct_per_kt_s = 6.0;
constexpr double accel_filter_tau_s = 0.5;
constexpr double integrate_band_kt = 5.0;  // integrate only near the target speed

// Local clamp so this header needs no <algorithm>.
inline double clamp_value(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

// Subsonic pitot relation: CAS from true airspeed and static conditions.
inline double calibrated_airspeed_kt(double tas_mps, double temperature_k, double static_pressure_pa)
{
    constexpr double gamma = 1.4;
    constexpr double gas_constant_value = 287.058;
    constexpr double sea_level_pressure_pa = 101325.0;
    constexpr double sea_level_sound_mps = 340.294;
    if (!(tas_mps > 0.0) || !(temperature_k > 0.0) || !(static_pressure_pa > 0.0))
    {
        return 0.0;
    }
    const double mach = tas_mps / std::sqrt(gamma * gas_constant_value * temperature_k);
    const double impact_pressure_pa = static_pressure_pa * (std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0);
    const double cas_mps =
        sea_level_sound_mps *
        std::sqrt(5.0 * (std::pow(impact_pressure_pa / sea_level_pressure_pa + 1.0, 2.0 / 7.0) - 1.0));
    return cas_mps * knots_per_mps;
}

class Controller
{
public:
    bool engaged() const
    {
        return engaged_;
    }
    bool disconnect_light() const
    {
        return disconnect_light_;
    }
    double target_cas_kt() const
    {
        return target_cas_kt_;
    }

    // A/T engage switch. It springs back (returns false) on the ground or with engines stopped.
    bool engage(double cas_kt, double throttle_pct, bool airborne, bool engine_running)
    {
        if (engaged_ || !airborne || !engine_running || !std::isfinite(cas_kt) || !std::isfinite(throttle_pct))
        {
            return false;
        }
        engaged_ = true;
        disconnect_light_ = false;
        holding_ = false;
        target_cas_kt_ = clamp_value(cas_kt, cas_min_kt, cas_max_kt);
        lever_pct_ = clamp_value(throttle_pct, 0.0, 100.0);
        integral_pct_ = lever_pct_;
        accel_kt_s_ = 0.0;
        have_previous_cas_ = false;
        return true;
    }

    void select_cas(double cas_kt)
    {
        if (std::isfinite(cas_kt))
        {
            target_cas_kt_ = clamp_value(cas_kt, cas_min_kt, cas_max_kt);
        }
    }

    // Disengagement by any means lights A/T DISC. The lever stays where the servo left it.
    void disengage()
    {
        if (engaged_)
        {
            engaged_ = false;
            disconnect_light_ = true;
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
            disconnect_light_ = false;
        }
    }

    // Physical throttle axis. Returns true when the axis should set the throttle.
    // A fixed axis cannot be driven by the servo, so deliberate travel overpowers
    // an engaged A/T, and a lever held after disengagement waits for axis movement.
    // Channel 0 is the common axis, 1 the left axis, 2 the right axis; each tracks its own reference.
    bool accept_axis(double axis_pct, int channel = 0)
    {
        if (!std::isfinite(axis_pct) || channel < 0 || channel >= axis_channels)
        {
            return false;
        }
        double& reference = axis_reference_pct_[channel];
        if (!std::isfinite(reference))
        {
            reference = axis_pct;
            return !engaged_ && !holding_;
        }
        if (engaged_)
        {
            if (std::fabs(axis_pct - reference) <= axis_override_pct)
            {
                return false;
            }
            disengage();
        }
        else if (holding_ && std::fabs(axis_pct - reference) <= axis_pickup_pct)
        {
            return false;
        }
        holding_ = false;
        reference = axis_pct;
        return true;
    }

    // Keyboard throttle steps act on the lever directly.
    void pilot_throttle_step()
    {
        disengage();
        holding_ = false;
    }

    // Returns the throttle command; unchanged when not engaged.
    double update(double dt, double cas_kt, double throttle_pct, bool airborne, bool engine_running)
    {
        if (!engaged_)
        {
            return throttle_pct;
        }
        if (!airborne || !engine_running || !std::isfinite(cas_kt))
        {
            disengage();
            return throttle_pct;
        }
        if (!(dt > 0.0))
        {
            return lever_pct_;
        }

        if (have_previous_cas_)
        {
            const double raw_accel = (cas_kt - previous_cas_kt_) / dt;
            accel_kt_s_ += (raw_accel - accel_kt_s_) * (1.0 - std::exp(-dt / accel_filter_tau_s));
        }
        previous_cas_kt_ = cas_kt;
        have_previous_cas_ = true;

        const double error_kt = target_cas_kt_ - cas_kt;
        const double feedback_pct = proportional_pct_per_kt * error_kt - accel_damping_pct_per_kt_s * accel_kt_s_;
        const double unclamped_pct = integral_pct_ + feedback_pct;
        const bool pushing_high = unclamped_pct >= 100.0 && error_kt > 0.0;
        const bool pushing_low = unclamped_pct <= 0.0 && error_kt < 0.0;
        if (!pushing_high && !pushing_low && std::fabs(error_kt) < integrate_band_kt)
        {
            integral_pct_ = clamp_value(integral_pct_ + integral_pct_per_kt_s * error_kt * dt, 0.0, 100.0);
        }

        const double desired_pct = clamp_value(integral_pct_ + feedback_pct, 0.0, 100.0);
        const double step_pct = servo_rate_pct_per_s * dt;
        lever_pct_ = clamp_value(desired_pct, lever_pct_ - step_pct, lever_pct_ + step_pct);
        return lever_pct_;
    }

    void reset()
    {
        *this = Controller();
    }

private:
    bool engaged_ = false;
    bool disconnect_light_ = false;
    bool holding_ = false;
    bool have_previous_cas_ = false;
    double target_cas_kt_ = 0.0;
    double lever_pct_ = 0.0;
    double integral_pct_ = 0.0;
    double accel_kt_s_ = 0.0;
    double previous_cas_kt_ = 0.0;
    static constexpr int axis_channels = 3;
    double axis_reference_pct_[axis_channels] = {std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace Autothrottle
