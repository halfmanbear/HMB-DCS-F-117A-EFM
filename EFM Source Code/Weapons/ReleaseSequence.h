#pragma once

// Weapon bay doors, the pickle and release gate sequence, and the laser designator.

namespace weapons
{
    void command_weapon_bay_open(int bay);
    void command_weapon_bay_close(int bay);
    void toggle_weapon_bay(int bay);
    void toggle_all_weapon_bays();
    int selected_weapon_bay();
    void reset_weapon_release_sequence();
    void handle_weapon_release_pressed();
    void handle_weapon_release_released();
    void reset_laser();
    void update_weapon_release_sequence(double dt);
    bool weapon_bay_valid(int bay);
    bool weapon_bay_is_open(int bay);
    bool weapon_bay_is_closed(int bay);
    void update_laser(double dt);
}
