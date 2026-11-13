#include "stdafx.h"
#include "ReleaseSequence.h"

#include <algorithm>
#include <cmath>

#include "Ccrp.h"
#include "../Avionics/CockpitBase_Interop.h"
#include "../Avionics/Weapons_Interop.h"
#include "../Cockpit/ParamBridge.h"
#include "../FlightModel/State.h"

namespace Weapons
{
bool weapon_bay_valid(int bay)
{
    return bay >= 0 && bay < F117::weapon_bay_count;
}

bool weapon_bay_is_open(int bay)
{
    return weapon_bay_valid(bay) && F117::bay_state[bay] >= weapon_bay_open_threshold;
}

bool weapon_bay_is_closed(int bay)
{
    return !weapon_bay_valid(bay) || F117::bay_state[bay] <= weapon_bay_closed_threshold;
}

void command_weapon_bay_open(int bay)
{
    if (weapon_bay_valid(bay))
        F117::bay_cmd[bay] = 1.0f;
}

void command_weapon_bay_close(int bay)
{
    if (weapon_bay_valid(bay))
        F117::bay_cmd[bay] = 0.0f;
}

// A bay toggles on its command, so a second press while it is still travelling reverses it.
void toggle_weapon_bay(int bay)
{
    if (!weapon_bay_valid(bay))
        return;
    if (F117::bay_cmd[bay] > 0.5f)
        command_weapon_bay_close(bay);
    else
        command_weapon_bay_open(bay);
}

// Both bays together: close them if either is open or opening, otherwise open them.
void toggle_all_weapon_bays()
{
    bool any_open = false;
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
        any_open = any_open || F117::bay_cmd[bay] > 0.5f;
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
    {
        if (any_open)
            command_weapon_bay_close(bay);
        else
            command_weapon_bay_open(bay);
    }
}

// The bay of the station the pilot has selected, or -1. Weapons.lua publishes the one-based station.
int selected_weapon_bay()
{
    static void* station_param = nullptr;
    if (!station_param)
        station_param = F117::cockpit_api.getParamHandle("F117_WEAPON_STATION");
    const double raw = station_param ? F117::cockpit_api.getParamNumber(station_param) : 0.0;
    if (!std::isfinite(raw))
        return -1;
    const int bay = static_cast<int>(raw) - 1;
    return weapon_bay_valid(bay) ? bay : -1;
}

void forget_weapon_bay_ownership(int bay)
{
    F117::weapon_bay_owned_by_release[bay] = false;
    F117::weapon_bay_close_pending[bay] = false;
    F117::weapon_bay_close_delay[bay] = 0.0;
}

// Ends any release in progress. The doors the pickle opened close, as in the auto mode of the bay door
// selector, whether the pilot takes them over by hand, changes Master Arm or the session restarts.
void reset_weapon_release_sequence()
{
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
    {
        if (F117::weapon_bay_owned_by_release[bay])
            command_weapon_bay_close(bay);
        forget_weapon_bay_ownership(bay);
    }
    F117::weapon_release_pickle_held = false;
    F117::weapon_release_gate = false;
    F117::weapon_release_gate_fired = false;
    F117::weapon_release_gate_hold = 0.0;
    publish_weapon_release_gate_param();
}

void handle_weapon_release_pressed()
{
    // Only the selected store's bay opens. A new pickle cancels that bay's pending auto-close.
    const int bay = selected_weapon_bay();
    if (weapon_bay_valid(bay))
    {
        F117::weapon_bay_close_pending[bay] = false;
        F117::weapon_bay_close_delay[bay] = 0.0;
    }
    if (F117::weapon_release_pickle_held)
        return;  // key repeat is not new consent
    F117::weapon_release_pickle_held = true;
    F117::weapon_release_gate_fired = false;
    F117::release_retry_delay = 0.0;

    // A bay the pilot already holds open is left alone, and one the pickle opened keeps its ownership, so a
    // re-pickle during a pending auto-close still closes it on the next release.
    if (weapon_bay_valid(bay) && F117::bay_cmd[bay] < 0.5f)
    {
        command_weapon_bay_open(bay);
        F117::weapon_bay_owned_by_release[bay] = true;
    }

    publish_weapon_release_gate_param();
}

void handle_weapon_release_released()
{
    F117::weapon_release_pickle_held = false;
    F117::weapon_release_gate_fired = false;

    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
    {
        if (F117::weapon_bay_owned_by_release[bay])
        {
            if (weapon_bay_is_open(bay))
            {
                F117::weapon_bay_close_pending[bay] = true;
                F117::weapon_bay_close_delay[bay] = weapon_bay_hold_after_release_seconds;
            }
            else
            {
                command_weapon_bay_close(bay);
                F117::weapon_bay_close_pending[bay] = false;
                F117::weapon_bay_close_delay[bay] = 0.0;
            }
        }
    }

    publish_weapon_release_gate_param();
}

// Laser arm + a valid powered designation automatically provides code-1688
// illumination before and after release; Safe or designation clear stops it.
void update_laser(double dt)
{
    F117::laser_window = std::max(0.0, F117::laser_window - dt);
    F117::weapon_flight_time_remaining = std::max(0.0, F117::weapon_flight_time_remaining - dt);

    double lock_point[3] = {0.0, 0.0, 0.0};
    const bool have_lock = Cockpit_interop::get_irads_lock_world_point(lock_point);
    const bool want_laser = Cockpit_interop::is_irads_powered() && F117::laser_armed && have_lock;
    Weapons_interop::update_laser(want_laser, lock_point, Weapons_interop::default_laser_code);

    static void* s_irads_power_param = nullptr;
    publish_named_param(s_irads_power_param, "F117_IRADS_POWER", Cockpit_interop::is_irads_powered() ? 1.0 : 0.0);
    static void* s_irads_zoom_param = nullptr;
    publish_named_param(s_irads_zoom_param, "IRADS_ZOOM", Cockpit_interop::get_irads_zoom_level());
    // World unit vector (x north, y up, z east) of where the IRADS is looking.
    // The HUD/IRADS cursor projects it into the HUD field of view.
    static void* s_irads_los_valid_param = nullptr;
    static void* s_irads_los_params[3] = {nullptr, nullptr, nullptr};
    static const char* const irads_los_names[3] = {"F117_IRADS_LOS_X", "F117_IRADS_LOS_Y", "F117_IRADS_LOS_Z"};
    double los[3] = {0.0, 0.0, 0.0};
    const bool los_valid = Cockpit_interop::get_irads_line_of_sight(los);
    publish_named_param(s_irads_los_valid_param, "F117_IRADS_LOS_VALID", los_valid ? 1.0 : 0.0);
    if (los_valid)
        for (int i = 0; i < 3; ++i)
            publish_named_param(s_irads_los_params[i], irads_los_names[i], los[i]);
    static void* s_laser_armed_param = nullptr;
    static void* s_laser_firing_param = nullptr;
    publish_named_param(s_laser_armed_param, laser_armed_param_name, F117::laser_armed ? 1.0 : 0.0);
    publish_named_param(s_laser_firing_param, laser_firing_param_name, Weapons_interop::is_laser_firing() ? 1.0 : 0.0);
}

void reset_laser()
{
    F117::master_armed = false;
    F117::release_retry_delay = 0.0;
    F117::release_status = 0;
    F117::release_sequence = 0;
    F117::laser_armed = false;
    F117::laser_window = 0.0;
    F117::weapon_flight_time_remaining = 0.0;
    F117::ccrp_time_of_flight = 0.0;
    Weapons_interop::reset();
}

namespace
{
// Everything the status ladder looks at.
struct Release_inputs
{
    bool has_solution;
    bool in_zone;
    bool store_valid;
    int station;
    bool efm_launch_available;
    bool laser_required;
};

// Release status shown to the cockpit: 1 safe, 2 IRADS off, 3 no lock, 4 no store, 5 laser, 7 no solution,
// 8 out of zone, 9 ready, 12 on the ground, 13 launch unavailable, 14 misaligned.
int compute_release_status(const Release_inputs& in)
{
    int status = 9;  // ready: hold consent to release at computed CCRP point
    if (!F117::master_armed)
        status = 1;
    else if (F117::weight_on_wheels || F117::wow_from_draw_args)
        status = 12;
    else if (!Cockpit_interop::is_irads_powered())
        status = 2;
    else if (!Cockpit_interop::has_irads_lock())
        status = 3;
    else if (!in.store_valid || in.station < 0 || in.station > 8)
        status = 4;
    else if (in.laser_required && (!F117::laser_armed || !Weapons_interop::is_laser_firing()))
        status = 5;
    // The pickle opens the selected store's bay itself, so bay position is not a fault. The store
    // is only let go once that bay is open (see the launch condition below).
    else if (!in.has_solution)
        status = 7;
    else if (!g_release_alignment_valid)
        status = 14;
    else if (!in.in_zone)
        status = 8;
    else if (!in.efm_launch_available)
        status = 13;
    return status;
}

// The store selection comes from the cockpit Lua: whether the selected station holds a releasable store, and which.
void read_store_selection(bool& store_valid, int& station)
{
    static void* profile_valid_param = nullptr;
    static void* station_param = nullptr;
    if (!profile_valid_param)
        profile_valid_param = F117::cockpit_api.getParamHandle("F117_STORE_VALID");
    if (!station_param)
        station_param = F117::cockpit_api.getParamHandle("F117_RELEASE_STATION");
    store_valid = profile_valid_param && F117::cockpit_api.getParamNumber(profile_valid_param) > 0.5;
    const double station_raw = station_param ? F117::cockpit_api.getParamNumber(station_param) : -1.0;
    station =
        std::isfinite(station_raw) && station_raw >= 0.0 && station_raw <= 8.0 ? static_cast<int>(station_raw) : -1;
}

// Consent held while the selection moves to another store (station keys): open that store's bay too.
void open_bay_for_held_consent(int station)
{
    if (F117::weapon_release_pickle_held && !F117::weapon_release_gate_fired && weapon_bay_valid(station) &&
        F117::bay_cmd[station] < 0.5f)
    {
        command_weapon_bay_open(station);
        F117::weapon_bay_owned_by_release[station] = true;
        F117::weapon_bay_close_pending[station] = false;
        F117::weapon_bay_close_delay[station] = 0.0;
    }
}

// Releases the store once the status is ready, consent is held and the bay is open. A failed native
// attempt retries after a short delay and does not consume consent.
void attempt_release(int station, int& status)
{
    if (status == 9 && F117::weapon_release_pickle_held && !F117::weapon_release_gate_fired &&
        F117::release_retry_delay <= 0.0 && weapon_bay_is_open(station))
    {
        double point[3] = {};
        if (Cockpit_interop::get_irads_lock_world_point(point) && Weapons_interop::launch_at_point(point, station))
        {
            ++F117::release_sequence;
            F117::weapon_release_gate_fired = true;
            F117::weapon_release_gate_hold = weapon_release_gate_min_hold_seconds;
            for (int i = 0; i < 3; ++i)
                F117::release_target[i] = point[i];
            refine_release_time_of_flight();
            F117::weapon_flight_time_remaining = F117::ccrp_time_of_flight;
            F117::laser_window = F117::ccrp_time_of_flight + laser_overrun_seconds;
            Cockpit_interop::log_message("DELIVERY: confirmed native launch; target snapshot saved");
            if (g_release_snapshot.valid)
            {
                const Release_snapshot& s = g_release_snapshot;
                Cockpit_interop::log_format("DELIVERY TOF: drop=%.0f range=%.0f free_fall=%.2f stock=%.2f used=%.2f",
                                            s.drop_meters, s.horiz_range, s.free_fall_tof, s.stock_tof,
                                            F117::ccrp_time_of_flight);
            }
        }
        else
        {
            status = 11;
            F117::release_retry_delay = 0.25;  // failed native attempt does not consume consent
        }
    }
}

void publish_release_state(int status)
{
    static void* status_param = nullptr;
    static void* arm_param = nullptr;
    static void* flight_param = nullptr;
    static void* min_range_param = nullptr;
    static void* max_range_param = nullptr;
    publish_named_param(min_range_param, "F117_RELEASE_MIN_RANGE", g_release_min_range);
    publish_named_param(max_range_param, "F117_RELEASE_MAX_RANGE", g_release_max_range);
    static void* sequence_param = nullptr;
    publish_named_param(sequence_param, "F117_RELEASE_SEQUENCE", F117::release_sequence);
    publish_named_param(status_param, "F117_RELEASE_STATUS", status);
    publish_named_param(arm_param, "F117_MASTER_ARM", F117::master_armed ? 1.0 : 0.0);
    publish_named_param(flight_param, "F117_STORE_FLIGHT_TIME", F117::weapon_flight_time_remaining);
}

// Each bay's auto-close runs on its own timer, so the first store's door shuts while the pilot
// pickles the next store.
void update_bay_close_timers(double dt)
{
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
    {
        if (!F117::weapon_release_pickle_held && F117::weapon_bay_owned_by_release[bay] && weapon_bay_is_closed(bay))
            forget_weapon_bay_ownership(bay);

        if (F117::weapon_bay_close_pending[bay])
        {
            F117::weapon_bay_close_delay[bay] = std::max(0.0, F117::weapon_bay_close_delay[bay] - dt);
            if (F117::weapon_bay_close_delay[bay] <= 0.0)
            {
                command_weapon_bay_close(bay);
                F117::weapon_bay_close_pending[bay] = false;
            }
        }
    }
}

void publish_bay_positions()
{
    static void* bay_params[F117::weapon_bay_count] = {};
    static const char* const bay_param_names[F117::weapon_bay_count] = {"F117_BAY_L", "F117_BAY_R"};
    for (int bay = 0; bay < F117::weapon_bay_count; ++bay)
        publish_named_param(bay_params[bay], bay_param_names[bay], F117::bay_state[bay]);
}
}  // namespace

void update_weapon_release_sequence(double dt)
{
    update_laser(dt);
    F117::release_retry_delay = std::max(0.0, F117::release_retry_delay - dt);
    double error_meters = 0.0;
    double time_to_go_seconds = 0.0;
    double range_meters = 0.0;
    bool in_zone = false;
    const bool has_solution = Cockpit_interop::has_irads_lock() &&
                              compute_release_solution(&error_meters, &time_to_go_seconds, &range_meters, &in_zone);
    Release_inputs inputs{has_solution,
                          in_zone,
                          false,
                          -1,
                          Weapons_interop::is_launch_available(),
                          store_needs_laser(get_ccrp_profile().kind)};
    read_store_selection(inputs.store_valid, inputs.station);
    const int station = inputs.station;
    int status = compute_release_status(inputs);
    F117::weapon_release_gate_hold = std::max(0.0, F117::weapon_release_gate_hold - dt);
    open_bay_for_held_consent(station);
    attempt_release(station, status);
    if (F117::weapon_release_gate_fired)
        status = 10;
    if (status != F117::release_status)
    {
        Cockpit_interop::log_format("DELIVERY: status=%d station=%d profile=%d consent=%d", status, station,
                                    get_ccrp_profile_index(), F117::weapon_release_pickle_held);
        F117::release_status = status;
    }
    publish_release_state(status);
    F117::weapon_release_gate = F117::weapon_release_gate_hold > 0.0;

    update_bay_close_timers(dt);
    publish_bay_positions();

    static void* s_efm_launch_active_param = nullptr;
    publish_named_param(s_efm_launch_active_param, efm_launch_active_param_name, 1.0);
    publish_release_cue_params();
    publish_weapon_release_gate_param();
}
}  // namespace Weapons
