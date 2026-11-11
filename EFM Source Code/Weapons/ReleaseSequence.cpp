#include "stdafx.h"
#include "ReleaseSequence.h"

#include <algorithm>
#include <cmath>

#include "Ccrp.h"
#include "../Avionics/CockpitBase_Interop.h"
#include "../Avionics/Weapons_Interop.h"
#include "../Cockpit/ParamBridge.h"
#include "../FlightModel/State.h"

namespace weapons
{
bool weapon_bay_valid(int bay)
{
    return bay >= 0 && bay < F117::kWeaponBayCount;
}

bool weapon_bay_is_open(int bay)
{
    return weapon_bay_valid(bay) && F117::bay_state[bay] >= kWeaponBayOpenThreshold;
}

bool weapon_bay_is_closed(int bay)
{
    return !weapon_bay_valid(bay) || F117::bay_state[bay] <= kWeaponBayClosedThreshold;
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
    bool anyOpen = false;
    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
        anyOpen = anyOpen || F117::bay_cmd[bay] > 0.5f;
    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
    {
        if (anyOpen)
            command_weapon_bay_close(bay);
        else
            command_weapon_bay_open(bay);
    }
}

// The bay of the station the pilot has selected, or -1. Weapons.lua publishes the one-based station.
int selected_weapon_bay()
{
    static void* stationParam = nullptr;
    if (!stationParam)
        stationParam = F117::cockpitAPI.getParamHandle("F117_WEAPON_STATION");
    const double raw = stationParam ? F117::cockpitAPI.getParamNumber(stationParam) : 0.0;
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
    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
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

    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
    {
        if (F117::weapon_bay_owned_by_release[bay])
        {
            if (weapon_bay_is_open(bay))
            {
                F117::weapon_bay_close_pending[bay] = true;
                F117::weapon_bay_close_delay[bay] = kWeaponBayHoldAfterReleaseSeconds;
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

    double lockPoint[3] = {0.0, 0.0, 0.0};
    const bool haveLock = CockpitInterop::GetIRADSLockWorldPoint(lockPoint);
    const bool wantLaser = CockpitInterop::IsIRADSPowered() && F117::laser_armed && haveLock;
    WeaponsInterop::UpdateLaser(wantLaser, lockPoint, WeaponsInterop::kDefaultLaserCode);

    static void* s_iradsPowerParam = nullptr;
    publish_named_param(s_iradsPowerParam, "F117_IRADS_POWER", CockpitInterop::IsIRADSPowered() ? 1.0 : 0.0);
    static void* s_iradsZoomParam = nullptr;
    publish_named_param(s_iradsZoomParam, "IRADS_ZOOM", CockpitInterop::GetIRADSZoomLevel());
    // World unit vector (x north, y up, z east) of where the IRADS is looking.
    // The HUD/IRADS cursor projects it into the HUD field of view.
    static void* s_iradsLosValidParam = nullptr;
    static void* s_iradsLosParams[3] = {nullptr, nullptr, nullptr};
    static const char* const kIRADSLosNames[3] = {"F117_IRADS_LOS_X", "F117_IRADS_LOS_Y", "F117_IRADS_LOS_Z"};
    double los[3] = {0.0, 0.0, 0.0};
    const bool losValid = CockpitInterop::GetIRADSLineOfSight(los);
    publish_named_param(s_iradsLosValidParam, "F117_IRADS_LOS_VALID", losValid ? 1.0 : 0.0);
    if (losValid)
        for (int i = 0; i < 3; ++i)
            publish_named_param(s_iradsLosParams[i], kIRADSLosNames[i], los[i]);
    static void* s_laserArmedParam = nullptr;
    static void* s_laserFiringParam = nullptr;
    publish_named_param(s_laserArmedParam, kLaserArmedParamName, F117::laser_armed ? 1.0 : 0.0);
    publish_named_param(s_laserFiringParam, kLaserFiringParamName, WeaponsInterop::IsLaserFiring() ? 1.0 : 0.0);
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
    WeaponsInterop::Reset();
}

void update_weapon_release_sequence(double dt)
{
    update_laser(dt);
    F117::release_retry_delay = std::max(0.0, F117::release_retry_delay - dt);
    double errorMeters = 0.0;
    double timeToGoSeconds = 0.0;
    double rangeMeters = 0.0;
    bool inZone = false;
    const bool hasSolution = CockpitInterop::HasIRADSLock() &&
                             compute_release_solution(&errorMeters, &timeToGoSeconds, &rangeMeters, &inZone);
    static void* profileValidParam = nullptr;
    static void* stationParam = nullptr;
    if (!profileValidParam)
        profileValidParam = F117::cockpitAPI.getParamHandle("F117_STORE_VALID");
    if (!stationParam)
        stationParam = F117::cockpitAPI.getParamHandle("F117_RELEASE_STATION");
    const bool storeValid = profileValidParam && F117::cockpitAPI.getParamNumber(profileValidParam) > 0.5;
    const double stationRaw = stationParam ? F117::cockpitAPI.getParamNumber(stationParam) : -1.0;
    const int station =
        std::isfinite(stationRaw) && stationRaw >= 0.0 && stationRaw <= 8.0 ? static_cast<int>(stationRaw) : -1;
    const bool efmLaunchAvailable = WeaponsInterop::IsLaunchAvailable();
    const bool laserRequired = store_needs_laser(get_ccrp_profile().kind);
    int status = 9;  // ready: hold consent to release at computed CCRP point
    if (!F117::master_armed)
        status = 1;
    else if (F117::weight_on_wheels || F117::wow_from_draw_args)
        status = 12;
    else if (!CockpitInterop::IsIRADSPowered())
        status = 2;
    else if (!CockpitInterop::HasIRADSLock())
        status = 3;
    else if (!storeValid || station < 0 || station > 8)
        status = 4;
    else if (laserRequired && (!F117::laser_armed || !WeaponsInterop::IsLaserFiring()))
        status = 5;
    // The pickle opens the selected store's bay itself, so bay position is not a fault. The store
    // is only let go once that bay is open (see the launch condition below).
    else if (!hasSolution)
        status = 7;
    else if (!g_releaseAlignmentValid)
        status = 14;
    else if (!inZone)
        status = 8;
    else if (!efmLaunchAvailable)
        status = 13;
    F117::weapon_release_gate_hold = std::max(0.0, F117::weapon_release_gate_hold - dt);
    // Consent held while the selection moves to another store (station keys): open that store's bay too.
    if (F117::weapon_release_pickle_held && !F117::weapon_release_gate_fired && weapon_bay_valid(station) &&
        F117::bay_cmd[station] < 0.5f)
    {
        command_weapon_bay_open(station);
        F117::weapon_bay_owned_by_release[station] = true;
        F117::weapon_bay_close_pending[station] = false;
        F117::weapon_bay_close_delay[station] = 0.0;
    }
    if (status == 9 && F117::weapon_release_pickle_held && !F117::weapon_release_gate_fired &&
        F117::release_retry_delay <= 0.0 && weapon_bay_is_open(station))
    {
        double point[3] = {};
        if (CockpitInterop::GetIRADSLockWorldPoint(point) && WeaponsInterop::LaunchAtPoint(point, station))
        {
            ++F117::release_sequence;
            F117::weapon_release_gate_fired = true;
            F117::weapon_release_gate_hold = kWeaponReleaseGateMinHoldSeconds;
            for (int i = 0; i < 3; ++i)
                F117::release_target[i] = point[i];
            F117::weapon_flight_time_remaining = F117::ccrp_time_of_flight;
            F117::laser_window = F117::ccrp_time_of_flight + kLaserOverrunSeconds;
            CockpitInterop::Log_("DELIVERY: confirmed native launch; target snapshot saved");
        }
        else
        {
            status = 11;
            F117::release_retry_delay = 0.25;  // failed native attempt does not consume consent
        }
    }
    if (F117::weapon_release_gate_fired)
        status = 10;
    if (status != F117::release_status)
    {
        CockpitInterop::Log("DELIVERY: status=%d station=%d profile=%d consent=%d", status, station,
                            get_ccrp_profile_index(), F117::weapon_release_pickle_held);
        F117::release_status = status;
    }
    static void* statusParam = nullptr;
    static void* armParam = nullptr;
    static void* flightParam = nullptr;
    static void* minRangeParam = nullptr;
    static void* maxRangeParam = nullptr;
    publish_named_param(minRangeParam, "F117_RELEASE_MIN_RANGE", g_releaseMinRange);
    publish_named_param(maxRangeParam, "F117_RELEASE_MAX_RANGE", g_releaseMaxRange);
    static void* sequenceParam = nullptr;
    publish_named_param(sequenceParam, "F117_RELEASE_SEQUENCE", F117::release_sequence);
    publish_named_param(statusParam, "F117_RELEASE_STATUS", status);
    publish_named_param(armParam, "F117_MASTER_ARM", F117::master_armed ? 1.0 : 0.0);
    publish_named_param(flightParam, "F117_STORE_FLIGHT_TIME", F117::weapon_flight_time_remaining);
    F117::weapon_release_gate = F117::weapon_release_gate_hold > 0.0;

    // Each bay's auto-close runs on its own timer, so the first store's door shuts while the pilot
    // pickles the next store.
    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
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

    static void* bayParams[F117::kWeaponBayCount] = {};
    static const char* const kBayParamNames[F117::kWeaponBayCount] = {"F117_BAY_L", "F117_BAY_R"};
    for (int bay = 0; bay < F117::kWeaponBayCount; ++bay)
        publish_named_param(bayParams[bay], kBayParamNames[bay], F117::bay_state[bay]);

    static void* s_efmLaunchActiveParam = nullptr;
    publish_named_param(s_efmLaunchActiveParam, kEfmLaunchActiveParamName, 1.0);
    publish_release_cue_params();
    publish_weapon_release_gate_param();
}
}  // namespace weapons
