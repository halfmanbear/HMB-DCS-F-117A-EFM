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
    pull_roll_inverted = 1,          // nose high, steep bank or inverted
    push_roll_upright = 2,           // nose high, near upright
    roll_upright = 3,                // near level, banked or inverted
    pull_roll_upright = 4,           // nose low (or level), upright side
    push_roll_upright_inverted = 5,  // nose low, inverted side
    pull_zero_roll_nose_down = 6,    // near vertical dive
    pull_zero_roll_nose_up = 7       // near vertical climb
};

// Sector boundaries digitised from Figure 4-3 (symmetric in bank angle).
constexpr double vertical_climb_deg = 83.0;
constexpr double vertical_dive_deg = -70.0;
constexpr double level_band_top_deg = 5.5;
constexpr double level_band_half_width_deg = 21.0;
constexpr double nose_low_floor_deg = -15.0;
constexpr double nose_low_floor_bank_deg = 60.0;

// Simulator choices; the manual only says "small positive pitch attitude" and that
// retracting the gear reduces pitch by approximately four degrees.
constexpr double target_pitch_gear_up_deg = 5.0;
constexpr double target_pitch_gear_down_deg = 9.0;
constexpr double pull_nz = 3.0;
constexpr double vertical_pull_nz = 4.0;
constexpr double inverted_push_nz = -1.0;
constexpr double attitude_nz_min = 0.0;
constexpr double attitude_nz_max = 4.0;
constexpr double pitch_gain_nz_per_deg = 0.08;
constexpr double pitch_rate_damping_nz_per_deg_s = 0.05;
constexpr double roll_gain_per_s = 2.0;
constexpr double roll_rate_max_deg_s = 90.0;
constexpr double pullout_nz = 2.8;  // assumed for the minimum pullout altitude estimate

constexpr double pi = 3.14159265358979323846;
constexpr double deg_to_rad = pi / 180.0;

inline double wrap_deg(double angle)
{
    angle = std::fmod(angle, 360.0);
    if (angle > 180.0)
        angle -= 360.0;
    if (angle <= -180.0)
        angle += 360.0;
    return angle;
}

// Upper edge of sectors 3/4 (flat at 5.5 deg near wings level, rising to 31 deg inverted).
inline double upper_boundary_deg(double abs_bank_deg)
{
    if (abs_bank_deg <= level_band_half_width_deg)
        return level_band_top_deg;
    return level_band_top_deg + (abs_bank_deg - level_band_half_width_deg) * (25.5 / 159.0);
}

// Lower edge of sector 3 (diagonal from the level band to -15 deg at 60 deg bank).
inline double lower_boundary_deg(double abs_bank_deg)
{
    if (abs_bank_deg <= level_band_half_width_deg)
        return level_band_top_deg;
    if (abs_bank_deg >= nose_low_floor_bank_deg)
        return nose_low_floor_deg;
    return level_band_top_deg -
           (abs_bank_deg - level_band_half_width_deg) *
               ((level_band_top_deg - nose_low_floor_deg) / (nose_low_floor_bank_deg - level_band_half_width_deg));
}

// Edge between sectors 1 and 2: from (37 deg bank, 83 deg pitch) to (104, 19).
inline double sector2_bank_limit_deg(double pitch_deg)
{
    return 37.0 + (vertical_climb_deg - pitch_deg) * (67.0 / 64.0);
}

inline int classify_sector(double pitch_deg, double bank_deg)
{
    const double abs_bank = std::fabs(wrap_deg(bank_deg));
    if (pitch_deg >= vertical_climb_deg)
        return pull_zero_roll_nose_up;
    if (pitch_deg <= vertical_dive_deg)
        return pull_zero_roll_nose_down;
    if (pitch_deg > upper_boundary_deg(abs_bank))
    {
        return abs_bank < sector2_bank_limit_deg(pitch_deg) ? push_roll_upright : pull_roll_inverted;
    }
    if (pitch_deg >= lower_boundary_deg(abs_bank) && abs_bank > level_band_half_width_deg)
    {
        return roll_upright;
    }
    return abs_bank < 90.0 ? pull_roll_upright : push_roll_upright_inverted;
}

// Not functional (PDF 19): A/R door open with |bank| < 45 and pitch -20..+25,
// or weight on wheels with |bank| < 20 and pitch -10..+15.
inline bool inhibited(double pitch_deg, double bank_deg, bool weight_on_wheels, bool air_refuel_door_open)
{
    const double abs_bank = std::fabs(wrap_deg(bank_deg));
    if (air_refuel_door_open && abs_bank < 45.0 && pitch_deg > -20.0 && pitch_deg < 25.0)
        return true;
    if (weight_on_wheels && abs_bank < 20.0 && pitch_deg > -10.0 && pitch_deg < 15.0)
        return true;
    return false;
}

struct Command
{
    int sector;
    double nz;               // commanded normal load factor (1 = level flight)
    double roll_rate_deg_s;  // commanded roll rate, positive right wing down
};

inline double clamp_value(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

inline double roll_rate_to(double bank_error_deg)
{
    return clamp_value(roll_gain_per_s * bank_error_deg, -roll_rate_max_deg_s, roll_rate_max_deg_s);
}

inline Command compute_command(double pitch_deg, double bank_deg, double roll_rate_deg_s, double pitch_rate_deg_s,
                               bool gear_down)
{
    const double bank = wrap_deg(bank_deg);
    const double gravity_nz = std::cos(bank * deg_to_rad) * std::cos(pitch_deg * deg_to_rad);
    const double target_pitch = gear_down ? target_pitch_gear_down_deg : target_pitch_gear_up_deg;
    const double attitude_nz = clamp_value(gravity_nz + pitch_gain_nz_per_deg * (target_pitch - pitch_deg) -
                                               pitch_rate_damping_nz_per_deg_s * pitch_rate_deg_s,
                                           attitude_nz_min, attitude_nz_max);

    // Roll upright the short way; when exactly inverted keep any existing roll direction.
    double upright_error = -bank;
    if (std::fabs(bank) > 179.0 && std::fabs(roll_rate_deg_s) > 1.0)
    {
        upright_error = roll_rate_deg_s > 0.0 ? 180.0 : -180.0;
    }
    const double inverted_error = (bank >= 0.0 ? 180.0 : -180.0) - bank;

    Command command{classify_sector(pitch_deg, bank), 1.0, 0.0};
    switch (command.sector)
    {
        case pull_roll_inverted:
            command.nz = pull_nz;
            command.roll_rate_deg_s = roll_rate_to(inverted_error);
            break;
        case push_roll_upright:
        case pull_roll_upright:
            command.nz = attitude_nz;
            command.roll_rate_deg_s = roll_rate_to(upright_error);
            break;
        case roll_upright:
            command.nz = gravity_nz;  // hold the flight path while rolling
            command.roll_rate_deg_s = roll_rate_to(upright_error);
            break;
        case push_roll_upright_inverted:
            command.nz = inverted_push_nz;
            command.roll_rate_deg_s = roll_rate_to(upright_error);
            break;
        case pull_zero_roll_nose_down:
        case pull_zero_roll_nose_up:
            command.nz = vertical_pull_nz;
            command.roll_rate_deg_s = 0.0;
            break;
    }
    return command;
}

// Estimate of the lowest altitude reached during recovery (the manual's NIAC
// computation is not given): descent while rolling upright, then a constant-g pull
// from the current flight path angle to level.
inline double minimum_pullout_altitude_ft(double altitude_ft, double tas_mps, double flight_path_deg, double bank_deg)
{
    constexpr double gravity = 9.81;
    constexpr double feet_per_metre = 3.28084;
    if (!(tas_mps > 0.0) || flight_path_deg >= 0.0)
        return altitude_ft;
    const double gamma = flight_path_deg * deg_to_rad;
    const double roll_time_s = std::fabs(wrap_deg(bank_deg)) / roll_rate_max_deg_s + 1.0;  // + g onset
    const double roll_loss_m = tas_mps * -std::sin(gamma) * roll_time_s;
    const double pull_loss_m = tas_mps * tas_mps * (1.0 - std::cos(gamma)) / (gravity * (pullout_nz - 1.0));
    return altitude_ft - (roll_loss_m + pull_loss_m) * feet_per_metre;
}

// Displayed value: minimum pullout altitude to the next lower thousand feet.
inline double displayed_pullout_altitude_ft(double minimum_ft)
{
    return std::floor(minimum_ft / 1000.0) * 1000.0;
}

class Controller
{
public:
    bool engaged() const
    {
        return engaged_;
    }

    // Stick ARS switch, first press.
    bool engage(double pitch_deg, double bank_deg, bool weight_on_wheels, bool air_refuel_door_open)
    {
        if (engaged_ || inhibited(pitch_deg, bank_deg, weight_on_wheels, air_refuel_door_open))
            return false;
        engaged_ = true;
        return true;
    }

    // Second ARS press or the disengage paddle.
    void disengage()
    {
        engaged_ = false;
    }

    // Aircraft state the recovery law reads.
    struct Flight_state
    {
        double pitch_deg;
        double bank_deg;
        double roll_rate_deg_s;
        double pitch_rate_deg_s;
        bool gear_down;
        bool weight_on_wheels;
        bool air_refuel_door_open;
    };

    // Returns false (and disengages) when PAARS is not functional.
    bool update(const Flight_state& state, Command* command)
    {
        if (!engaged_)
            return false;
        if (inhibited(state.pitch_deg, state.bank_deg, state.weight_on_wheels, state.air_refuel_door_open))
        {
            engaged_ = false;
            return false;
        }
        *command = compute_command(state.pitch_deg, state.bank_deg, state.roll_rate_deg_s, state.pitch_rate_deg_s,
                                   state.gear_down);
        return true;
    }

    void reset()
    {
        engaged_ = false;
    }

private:
    bool engaged_ = false;
};
}  // namespace Paars
