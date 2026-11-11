#include "stdafx.h"
#include "Damage.h"

#include <algorithm>
#include <cmath>
#include "../Diagnostics/LogFile.h"
#include <string>

#include "../Diagnostics/Paths.h"

namespace
{
constexpr double lua_damage_dose = 0.25;
constexpr double wing_damage_dose_scale = 0.6;
constexpr double engine_damage_dose_scale = 0.4;
constexpr double tail_damage_dose_scale = 0.3;
constexpr double cockpit_damage_dose_scale = 0.2;
constexpr double fire_damage_threshold = 0.35;
constexpr int damage_element_nose = 0;
constexpr int damage_element_cockpit = 3;
constexpr int damage_element_nose_gear = 8;
constexpr int damage_element_fuselage_left = 9;
constexpr int damage_element_fuselage_right = 10;  // Also DCS "MAIN".
constexpr int damage_element_engine_left = 11;
constexpr int damage_element_engine_right = 12;
constexpr int damage_element_left_gear = 15;
constexpr int damage_element_right_gear = 16;
constexpr int damage_element_wing_outer_left = 23;
constexpr int damage_element_wing_outer_right = 24;
constexpr int damage_element_wing_inner_left = 35;
constexpr int damage_element_wing_inner_right = 36;
constexpr int damage_element_tail_left = 56;
constexpr int damage_element_tail_right = 57;
constexpr int damage_element_belly = 82;
constexpr int damage_element_nose_wheel = 83;
constexpr int damage_element_left_wheel = 84;
constexpr int damage_element_right_wheel = 85;
constexpr double damage_cell_destroyed_integrity = 0.101;
constexpr double outer_wing_lift_share = 0.30;
constexpr double structural_drag_penalty = 0.30;
constexpr int damage_element_aileron_left = 25;
constexpr int damage_element_aileron_right = 26;
constexpr int damage_element_flap_left = 37;
constexpr int damage_element_flap_right = 38;
constexpr int damage_element_rudder_left = 53;
constexpr int damage_element_rudder_right = 54;

Log_file g_damage_log;
double g_damage_log_timer = 0.0;

bool try_open_damage_log_path(const char* path)
{
    if (!g_damage_log.open(path, false))
    {
        return false;
    }

    g_damage_log.write("=== F-117 Damage Debug Log (DCS-owned visuals v2) ===\n");
    g_damage_log.write("Log opened at: %s\n", path);
    g_damage_log.write("invincible_value at init: %d\n\n", static_cast<int>(F117::param_class.invincible_value));
    return true;
}

void init_damage_log()
{
    if (g_damage_log.is_open())
        return;

    const std::string paths[] = {build_saved_games_dcs_path("DCS", "Logs\\F117_Damage_Debug.log"),
                                 build_saved_games_dcs_path("DCS", "F117_Damage_Debug.log"),
                                 build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Damage_Debug.log"),
                                 build_saved_games_dcs_path("DCS.openbeta", "F117_Damage_Debug.log"),
                                 "F117_Damage_Debug.log"};

    for (const std::string& path : paths)
    {
        if (!path.empty() && try_open_damage_log_path(path.c_str()))
        {
            return;
        }
    }
}

void log_damage_callback_header(int element, double element_integrity_factor)
{
    if (g_damage_log.is_open())
    {
        g_damage_log.write("ed_fm_on_damage called: Element=%d, integrity=%.4f, invincible=%d\n", element,
                           element_integrity_factor, static_cast<int>(F117::param_class.invincible_value));
    }
}

void log_damage_callback_skipped_immortal()
{
    if (g_damage_log.is_open())
    {
        g_damage_log.write("  -> SKIPPED (immortal)\n");
    }
}

double damage_integrity_from_dcs(double reported)
{
    const double integrity = limit(reported, 0.0, 1.0);
    return integrity <= damage_cell_destroyed_integrity ? 0.0 : integrity;
}

// Combines both panels of one wing. A destroyed panel takes the surfaces it carries with it,
// because DCS does not report the dependent cells to the EFM.
void update_wing_from_sections(double& wing, double& inner, double& outer, double& inner_elevon, double& outer_elevon)
{
    wing = std::min(wing, inner * (1.0 - outer_wing_lift_share + outer_wing_lift_share * outer));
    if (inner <= 0.0)
    {
        outer = 0.0;
        inner_elevon = 0.0;
    }
    if (outer <= 0.0)
    {
        outer_elevon = 0.0;
    }
}

const char* apply_damage_for_element(int element, double integrity)
{
    switch (element)
    {
        case damage_element_nose:
            g_damage.nose = std::min(g_damage.nose, integrity);
            return "nose";
        case damage_element_cockpit:
            g_damage.cockpit = std::min(g_damage.cockpit, integrity);
            return "cockpit";
        case damage_element_fuselage_left:
            g_damage.left_fuselage = std::min(g_damage.left_fuselage, integrity);
            return "left fuselage";
        case damage_element_fuselage_right:
            g_damage.right_fuselage = std::min(g_damage.right_fuselage, integrity);
            return "right fuselage";
        case damage_element_belly:
            g_damage.belly = std::min(g_damage.belly, integrity);
            return "weapon bay doors";
        case damage_element_engine_left:
            g_damage.left_engine = std::min(g_damage.left_engine, integrity);
            if (g_damage.left_engine < fire_damage_threshold)
                push_fire_event(8, -4.45, 0.08, -1.7);
            return "left engine";
        case damage_element_engine_right:
            g_damage.right_engine = std::min(g_damage.right_engine, integrity);
            if (g_damage.right_engine < fire_damage_threshold)
                push_fire_event(7, -4.45, 0.08, 1.7);
            return "right engine";
        case damage_element_tail_left:
            g_damage.left_tail_structure = std::min(g_damage.left_tail_structure, integrity);
            if (g_damage.left_tail_structure <= 0.0)
                g_damage.left_tail = 0.0;
            return "left tail structure";
        case damage_element_tail_right:
            g_damage.right_tail_structure = std::min(g_damage.right_tail_structure, integrity);
            if (g_damage.right_tail_structure <= 0.0)
                g_damage.right_tail = 0.0;
            return "right tail structure";
        case damage_element_wing_inner_left:
            g_damage.left_wing_inner = std::min(g_damage.left_wing_inner, integrity);
            update_wing_from_sections(g_damage.left_wing, g_damage.left_wing_inner, g_damage.left_wing_outer,
                                      g_damage.left_inner_elevon, g_damage.left_outer_elevon);
            return "left inner wing";
        case damage_element_wing_inner_right:
            g_damage.right_wing_inner = std::min(g_damage.right_wing_inner, integrity);
            update_wing_from_sections(g_damage.right_wing, g_damage.right_wing_inner, g_damage.right_wing_outer,
                                      g_damage.right_inner_elevon, g_damage.right_outer_elevon);
            return "right inner wing";
        case damage_element_wing_outer_left:
            g_damage.left_wing_outer = std::min(g_damage.left_wing_outer, integrity);
            update_wing_from_sections(g_damage.left_wing, g_damage.left_wing_inner, g_damage.left_wing_outer,
                                      g_damage.left_inner_elevon, g_damage.left_outer_elevon);
            return "left outer wing";
        case damage_element_wing_outer_right:
            g_damage.right_wing_outer = std::min(g_damage.right_wing_outer, integrity);
            update_wing_from_sections(g_damage.right_wing, g_damage.right_wing_inner, g_damage.right_wing_outer,
                                      g_damage.right_inner_elevon, g_damage.right_outer_elevon);
            return "right outer wing";
        case damage_element_nose_gear:
            g_damage.nose_gear = std::min(g_damage.nose_gear, integrity);
            return "nose gear bay";
        case damage_element_left_gear:
            g_damage.left_gear = std::min(g_damage.left_gear, integrity);
            return "left gear bay";
        case damage_element_right_gear:
            g_damage.right_gear = std::min(g_damage.right_gear, integrity);
            return "right gear bay";
        // DCS applies wheel damage through each suspension's damage_element.
        case damage_element_nose_wheel:
            g_damage.nose_wheel = std::min(g_damage.nose_wheel, integrity);
            return "nose wheel";
        case damage_element_left_wheel:
            g_damage.left_wheel = std::min(g_damage.left_wheel, integrity);
            return "left wheel";
        case damage_element_right_wheel:
            g_damage.right_wheel = std::min(g_damage.right_wheel, integrity);
            return "right wheel";
        case damage_element_aileron_left:
            g_damage.left_outer_elevon = std::min(g_damage.left_outer_elevon, integrity);
            return "left outer elevon";
        case damage_element_aileron_right:
            g_damage.right_outer_elevon = std::min(g_damage.right_outer_elevon, integrity);
            return "right outer elevon";
        case damage_element_flap_left:
            g_damage.left_inner_elevon = std::min(g_damage.left_inner_elevon, integrity);
            return "left inner elevon";
        case damage_element_flap_right:
            g_damage.right_inner_elevon = std::min(g_damage.right_inner_elevon, integrity);
            return "right inner elevon";
        case damage_element_rudder_left:
            g_damage.left_tail = std::min(g_damage.left_tail, integrity);
            return "left ruddervator";
        case damage_element_rudder_right:
            g_damage.right_tail = std::min(g_damage.right_tail, integrity);
            return "right ruddervator";
        default:
            return "IGNORED(unhandled)";
    }
}

void log_damage_mapping_result(const char* mapped)
{
    if (g_damage_log.is_open())
    {
        g_damage_log.write("  -> mapped to: %s | State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                           mapped, g_damage.left_wing, g_damage.right_wing, g_damage.left_engine, g_damage.right_engine,
                           g_damage.left_tail, g_damage.right_tail, g_damage.cockpit);
    }
}

void clear_pending_sim_events()
{
    while (!g_sim_events.empty())
    {
        g_sim_events.pop();
    }
}
}  // namespace

// Push a fire event into the simulation event queue
// API layout (wHumanCustomPhysicsAPI.h):
//   [0] = handle, [1-3] = position XYZ, [4-6] = direction XYZ, [7] = speed, [8] = scale
void push_fire_event(int handle, double x, double y, double z)
{
    if (handle < 0 || handle >= 9 || g_damage.fire_active[handle])
        return;
    g_damage.fire_active[handle] = true;
    ed_fm_simulation_event evt = {};
    evt.event_type = ED_FM_EVENT_FIRE;
    evt.event_params[0] = static_cast<float>(handle);  // fire control handle
    evt.event_params[1] = static_cast<float>(x);       // fire origin X (body space)
    evt.event_params[2] = static_cast<float>(y);       // fire origin Y
    evt.event_params[3] = static_cast<float>(z);       // fire origin Z
    evt.event_params[4] = 0.0f;                        // emitter direction X
    evt.event_params[5] = 1.0f;                        // emitter direction Y (up)
    evt.event_params[6] = 0.0f;                        // emitter direction Z
    evt.event_params[7] = 5.0f;                        // particle speed
    evt.event_params[8] = 1.0f;                        // scale (>0 = fire on, <=0 = fire off)
    g_sim_events.push(evt);
}

void apply_lua_damage_hit()
{
    if (F117::param_class.invincible_value == 0)
    {
        return;
    }

    g_damage.left_wing = std::max(0.0, g_damage.left_wing - lua_damage_dose * wing_damage_dose_scale);
    g_damage.right_wing = std::max(0.0, g_damage.right_wing - lua_damage_dose * wing_damage_dose_scale);
    g_damage.left_engine = std::max(0.0, g_damage.left_engine - lua_damage_dose * engine_damage_dose_scale);
    g_damage.right_engine = std::max(0.0, g_damage.right_engine - lua_damage_dose * engine_damage_dose_scale);
    g_damage.left_tail = std::max(0.0, g_damage.left_tail - lua_damage_dose * tail_damage_dose_scale);
    g_damage.right_tail = std::max(0.0, g_damage.right_tail - lua_damage_dose * tail_damage_dose_scale);
    g_damage.cockpit = std::max(0.0, g_damage.cockpit - lua_damage_dose * cockpit_damage_dose_scale);

    if (g_damage.left_engine < fire_damage_threshold)
    {
        push_fire_event(8, -4.45, 0.08, -1.7);
    }
    if (g_damage.right_engine < fire_damage_threshold)
    {
        push_fire_event(7, -4.45, 0.08, 1.7);
    }

    init_damage_log();
    if (g_damage_log.is_open())
    {
        g_damage_log.write(
            "[LUA_DMG] Hit received! dose=%.2f | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
            lua_damage_dose, g_damage.left_wing, g_damage.right_wing, g_damage.left_engine, g_damage.right_engine,
            g_damage.left_tail, g_damage.right_tail, g_damage.cockpit);
    }
}

void update_derived_damage_values()
{
    g_damage.wing_asymmetry = g_damage.left_wing - g_damage.right_wing;
    g_damage.total_wing_loss = 1.0 - (g_damage.left_wing + g_damage.right_wing) / 2.0;
    g_damage.total_tail_loss = 1.0 - (g_damage.left_tail + g_damage.right_tail) / 2.0;
    g_damage.engine_asymmetry = g_damage.left_engine - g_damage.right_engine;
    const double structural_integrity =
        (g_damage.nose + g_damage.left_fuselage + g_damage.right_fuselage + g_damage.belly +
         g_damage.left_tail_structure + g_damage.right_tail_structure + g_damage.left_wing_inner +
         g_damage.right_wing_inner + g_damage.left_wing_outer + g_damage.right_wing_outer) /
        10.0;
    g_damage.structural_drag_factor = 1.0 + structural_drag_penalty * (1.0 - structural_integrity);
}

double get_control_degradation()
{
    double control_degradation = 1.0;
    if (g_damage.cockpit < 0.7)
    {
        control_degradation = g_damage.cockpit / 0.7;
    }

    return control_degradation;
}

void reset_damage_and_events()
{
    clear_pending_sim_events();
    // Explicitly stop existing emitters when DCS repairs/reinitializes the aircraft.
    for (int handle = 0; handle < 9; ++handle)
    {
        if (g_damage.fire_active[handle])
        {
            ed_fm_simulation_event event = {};
            event.event_type = ED_FM_EVENT_FIRE;
            event.event_params[0] = static_cast<float>(handle);
            event.event_params[8] = 0.0f;
            g_sim_events.push(event);
        }
    }
    g_damage = F117::Damage_state();
}

void log_damage_lifecycle_event(const char* label)
{
    init_damage_log();
    if (g_damage_log.is_open())
    {
        g_damage_log.write("\n*** %s called ***\n", label);
    }
}

void log_simulation_frame(double dt, double acceleration_mps2)
{
    static int sim_frame_count = 0;
    sim_frame_count++;
    init_damage_log();
    if (g_damage_log.is_open())
    {
        g_damage_log_timer += dt;
        if (sim_frame_count <= 500 || g_damage_log_timer >= 2.0)
        {
            g_damage_log_timer = 0.0;
            g_damage_log.write(
                "[SIM] frame=%d invincible=%d | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f | "
                "accel=%.1f events=%d WoW=%d\n",
                sim_frame_count, static_cast<int>(F117::param_class.invincible_value), g_damage.left_wing,
                g_damage.right_wing, g_damage.left_engine, g_damage.right_engine, g_damage.left_tail,
                g_damage.right_tail, g_damage.cockpit, acceleration_mps2, static_cast<int>(g_sim_events.size()),
                static_cast<int>(F117::weight_on_wheels));
        }
    }
}

// DCS calls this to dequeue simulation events (fire, smoke, failures)
bool ed_fm_pop_simulation_event(ed_fm_simulation_event& out)
{
    if (g_sim_events.empty())
        return false;
    out = g_sim_events.front();
    g_sim_events.pop();
    if (g_damage_log.is_open())
    {
        g_damage_log.write(
            "ed_fm_pop_simulation_event: type=%u handle=%.0f pos=(%.1f,%.1f,%.1f) dir=(%.1f,%.1f,%.1f) speed=%.1f "
            "scale=%.1f\n",
            out.event_type, out.event_params[0], out.event_params[1], out.event_params[2], out.event_params[3],
            out.event_params[4], out.event_params[5], out.event_params[6], out.event_params[7], out.event_params[8]);
    }
    return true;
}

void ed_fm_on_damage(int element, double remaining_integrity)
{
    init_damage_log();
    log_damage_callback_header(element, remaining_integrity);
    if (!F117::param_class.invincible_value)
    {
        log_damage_callback_skipped_immortal();
        return;
    }
    if (!std::isfinite(remaining_integrity))
        return;
    const double normalized = damage_integrity_from_dcs(remaining_integrity);
    log_damage_mapping_result(apply_damage_for_element(element, normalized));
}

void ed_fm_repair()
{
    reset_damage_and_events();
}

void ed_fm_set_immortal(bool enabled)
{
    F117::param_class.set_immortal(enabled);
    init_damage_log();
    if (g_damage_log.is_open())
    {
        g_damage_log.write("ed_fm_set_immortal called: value=%d -> invincible_value=%d\n", static_cast<int>(enabled),
                           static_cast<int>(F117::param_class.invincible_value));
    }
}

// Returns true when any component has taken damage, triggering the repair process
bool ed_fm_need_to_be_repaired()
{
    return g_damage.left_wing < 1.0 || g_damage.right_wing < 1.0 || g_damage.left_engine < 1.0 ||
           g_damage.right_engine < 1.0 || g_damage.left_tail < 1.0 || g_damage.right_tail < 1.0 ||
           g_damage.cockpit < 1.0 || g_damage.left_outer_elevon < 1.0 || g_damage.right_outer_elevon < 1.0 ||
           g_damage.left_inner_elevon < 1.0 || g_damage.right_inner_elevon < 1.0 || g_damage.nose < 1.0 ||
           g_damage.left_fuselage < 1.0 || g_damage.right_fuselage < 1.0 || g_damage.belly < 1.0 ||
           g_damage.left_tail_structure < 1.0 || g_damage.right_tail_structure < 1.0 ||
           g_damage.left_wing_inner < 1.0 || g_damage.right_wing_inner < 1.0 || g_damage.left_wing_outer < 1.0 ||
           g_damage.right_wing_outer < 1.0 || g_damage.nose_gear < 1.0 || g_damage.left_gear < 1.0 ||
           g_damage.right_gear < 1.0 || g_damage.nose_wheel < 1.0 || g_damage.left_wheel < 1.0 ||
           g_damage.right_wheel < 1.0;
}
