#pragma once

// The public fields describe whether each physical effect is simulated.
// DCS supplies the inverse: unlimited fuel and immortality enable assistance.
struct param_stuff final
{
    bool fuelvalue{true};
    bool invincible_value{true};

    void set_unlimited_fuel(bool enabled) noexcept
    {
        fuelvalue = !enabled;
    }
    void set_immortal(bool enabled) noexcept
    {
        invincible_value = !enabled;
    }
};
