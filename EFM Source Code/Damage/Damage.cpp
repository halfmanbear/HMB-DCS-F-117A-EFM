#include "stdafx.h"
#include "Damage.h"

#include <algorithm>
#include <cmath>
#include "../Diagnostics/LogFile.h"
#include <string>

#include "../Diagnostics/Paths.h"

namespace
{
constexpr double kLuaDamageDose = 0.25;
constexpr double kWingDamageDoseScale = 0.6;
constexpr double kEngineDamageDoseScale = 0.4;
constexpr double kTailDamageDoseScale = 0.3;
constexpr double kCockpitDamageDoseScale = 0.2;
constexpr double kFireDamageThreshold = 0.35;
constexpr int kDamageElementNose = 0;
constexpr int kDamageElementCockpit = 3;
constexpr int kDamageElementNoseGear = 8;
constexpr int kDamageElementFuselageLeft = 9;
constexpr int kDamageElementFuselageRight = 10;  // Also DCS "MAIN".
constexpr int kDamageElementEngineLeft = 11;
constexpr int kDamageElementEngineRight = 12;
constexpr int kDamageElementLeftGear = 15;
constexpr int kDamageElementRightGear = 16;
constexpr int kDamageElementWingOuterLeft = 23;
constexpr int kDamageElementWingOuterRight = 24;
constexpr int kDamageElementWingInnerLeft = 35;
constexpr int kDamageElementWingInnerRight = 36;
constexpr int kDamageElementTailLeft = 56;
constexpr int kDamageElementTailRight = 57;
constexpr int kDamageElementBelly = 82;
constexpr int kDamageElementNoseWheel = 83;
constexpr int kDamageElementLeftWheel = 84;
constexpr int kDamageElementRightWheel = 85;
constexpr double kDamageCellDestroyedIntegrity = 0.101;
constexpr double kOuterWingLiftShare = 0.30;
constexpr double kStructuralDragPenalty = 0.30;
constexpr int kDamageElementAileronLeft = 25;
constexpr int kDamageElementAileronRight = 26;
constexpr int kDamageElementFlapLeft = 37;
constexpr int kDamageElementFlapRight = 38;
constexpr int kDamageElementRudderLeft = 53;
constexpr int kDamageElementRudderRight = 54;

LogFile g_damageLog;
double g_damageLogTimer = 0.0;

bool try_open_damage_log_path(const char* path)
{
    if (!g_damageLog.open(path, false))
    {
        return false;
    }

    g_damageLog.write("=== F-117 Damage Debug Log (DCS-owned visuals v2) ===\n");
    g_damageLog.write("Log opened at: %s\n", path);
    g_damageLog.write("invincible_value at init: %d\n\n", static_cast<int>(F117::param_class.invincible_value));
    return true;
}

void initDamageLog()
{
    if (g_damageLog.is_open())
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
    if (g_damageLog.is_open())
    {
        g_damageLog.write("ed_fm_on_damage called: Element=%d, integrity=%.4f, invincible=%d\n", element,
                          element_integrity_factor, static_cast<int>(F117::param_class.invincible_value));
    }
}

void log_damage_callback_skipped_immortal()
{
    if (g_damageLog.is_open())
    {
        g_damageLog.write("  -> SKIPPED (immortal)\n");
    }
}

double damage_integrity_from_dcs(double reported)
{
    const double integrity = limit(reported, 0.0, 1.0);
    return integrity <= kDamageCellDestroyedIntegrity ? 0.0 : integrity;
}

// Combines both panels of one wing. A destroyed panel takes the surfaces it carries with it,
// because DCS does not report the dependent cells to the EFM.
void update_wing_from_sections(double& wing, double& inner, double& outer, double& innerElevon, double& outerElevon)
{
    wing = std::min(wing, inner * (1.0 - kOuterWingLiftShare + kOuterWingLiftShare * outer));
    if (inner <= 0.0)
    {
        outer = 0.0;
        innerElevon = 0.0;
    }
    if (outer <= 0.0)
    {
        outerElevon = 0.0;
    }
}

const char* apply_damage_for_element(int element, double integrity)
{
    switch (element)
    {
        case kDamageElementNose:
            g_damage.nose = std::min(g_damage.nose, integrity);
            return "nose";
        case kDamageElementCockpit:
            g_damage.cockpit = std::min(g_damage.cockpit, integrity);
            return "cockpit";
        case kDamageElementFuselageLeft:
            g_damage.leftFuselage = std::min(g_damage.leftFuselage, integrity);
            return "left fuselage";
        case kDamageElementFuselageRight:
            g_damage.rightFuselage = std::min(g_damage.rightFuselage, integrity);
            return "right fuselage";
        case kDamageElementBelly:
            g_damage.belly = std::min(g_damage.belly, integrity);
            return "weapon bay doors";
        case kDamageElementEngineLeft:
            g_damage.leftEngine = std::min(g_damage.leftEngine, integrity);
            if (g_damage.leftEngine < kFireDamageThreshold)
                pushFireEvent(8, -4.45, 0.08, -1.7);
            return "left engine";
        case kDamageElementEngineRight:
            g_damage.rightEngine = std::min(g_damage.rightEngine, integrity);
            if (g_damage.rightEngine < kFireDamageThreshold)
                pushFireEvent(7, -4.45, 0.08, 1.7);
            return "right engine";
        case kDamageElementTailLeft:
            g_damage.leftTailStructure = std::min(g_damage.leftTailStructure, integrity);
            if (g_damage.leftTailStructure <= 0.0)
                g_damage.leftTail = 0.0;
            return "left tail structure";
        case kDamageElementTailRight:
            g_damage.rightTailStructure = std::min(g_damage.rightTailStructure, integrity);
            if (g_damage.rightTailStructure <= 0.0)
                g_damage.rightTail = 0.0;
            return "right tail structure";
        case kDamageElementWingInnerLeft:
            g_damage.leftWingInner = std::min(g_damage.leftWingInner, integrity);
            update_wing_from_sections(g_damage.leftWing, g_damage.leftWingInner, g_damage.leftWingOuter,
                                      g_damage.leftInnerElevon, g_damage.leftOuterElevon);
            return "left inner wing";
        case kDamageElementWingInnerRight:
            g_damage.rightWingInner = std::min(g_damage.rightWingInner, integrity);
            update_wing_from_sections(g_damage.rightWing, g_damage.rightWingInner, g_damage.rightWingOuter,
                                      g_damage.rightInnerElevon, g_damage.rightOuterElevon);
            return "right inner wing";
        case kDamageElementWingOuterLeft:
            g_damage.leftWingOuter = std::min(g_damage.leftWingOuter, integrity);
            update_wing_from_sections(g_damage.leftWing, g_damage.leftWingInner, g_damage.leftWingOuter,
                                      g_damage.leftInnerElevon, g_damage.leftOuterElevon);
            return "left outer wing";
        case kDamageElementWingOuterRight:
            g_damage.rightWingOuter = std::min(g_damage.rightWingOuter, integrity);
            update_wing_from_sections(g_damage.rightWing, g_damage.rightWingInner, g_damage.rightWingOuter,
                                      g_damage.rightInnerElevon, g_damage.rightOuterElevon);
            return "right outer wing";
        case kDamageElementNoseGear:
            g_damage.noseGear = std::min(g_damage.noseGear, integrity);
            return "nose gear bay";
        case kDamageElementLeftGear:
            g_damage.leftGear = std::min(g_damage.leftGear, integrity);
            return "left gear bay";
        case kDamageElementRightGear:
            g_damage.rightGear = std::min(g_damage.rightGear, integrity);
            return "right gear bay";
        // DCS applies wheel damage through each suspension's damage_element.
        case kDamageElementNoseWheel:
            g_damage.noseWheel = std::min(g_damage.noseWheel, integrity);
            return "nose wheel";
        case kDamageElementLeftWheel:
            g_damage.leftWheel = std::min(g_damage.leftWheel, integrity);
            return "left wheel";
        case kDamageElementRightWheel:
            g_damage.rightWheel = std::min(g_damage.rightWheel, integrity);
            return "right wheel";
        case kDamageElementAileronLeft:
            g_damage.leftOuterElevon = std::min(g_damage.leftOuterElevon, integrity);
            return "left outer elevon";
        case kDamageElementAileronRight:
            g_damage.rightOuterElevon = std::min(g_damage.rightOuterElevon, integrity);
            return "right outer elevon";
        case kDamageElementFlapLeft:
            g_damage.leftInnerElevon = std::min(g_damage.leftInnerElevon, integrity);
            return "left inner elevon";
        case kDamageElementFlapRight:
            g_damage.rightInnerElevon = std::min(g_damage.rightInnerElevon, integrity);
            return "right inner elevon";
        case kDamageElementRudderLeft:
            g_damage.leftTail = std::min(g_damage.leftTail, integrity);
            return "left ruddervator";
        case kDamageElementRudderRight:
            g_damage.rightTail = std::min(g_damage.rightTail, integrity);
            return "right ruddervator";
        default:
            return "IGNORED(unhandled)";
    }
}

void log_damage_mapping_result(const char* mapped)
{
    if (g_damageLog.is_open())
    {
        g_damageLog.write("  -> mapped to: %s | State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                          mapped, g_damage.leftWing, g_damage.rightWing, g_damage.leftEngine, g_damage.rightEngine,
                          g_damage.leftTail, g_damage.rightTail, g_damage.cockpit);
    }
}

void clear_pending_sim_events()
{
    while (!g_simEvents.empty())
    {
        g_simEvents.pop();
    }
}
}  // namespace

// Push a fire event into the simulation event queue
// API layout (wHumanCustomPhysicsAPI.h):
//   [0] = handle, [1-3] = position XYZ, [4-6] = direction XYZ, [7] = speed, [8] = scale
void pushFireEvent(int handle, double x, double y, double z)
{
    if (handle < 0 || handle >= 9 || g_damage.fireActive[handle])
        return;
    g_damage.fireActive[handle] = true;
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
    g_simEvents.push(evt);
}

void apply_lua_damage_hit()
{
    if (F117::param_class.invincible_value == 0)
    {
        return;
    }

    g_damage.leftWing = std::max(0.0, g_damage.leftWing - kLuaDamageDose * kWingDamageDoseScale);
    g_damage.rightWing = std::max(0.0, g_damage.rightWing - kLuaDamageDose * kWingDamageDoseScale);
    g_damage.leftEngine = std::max(0.0, g_damage.leftEngine - kLuaDamageDose * kEngineDamageDoseScale);
    g_damage.rightEngine = std::max(0.0, g_damage.rightEngine - kLuaDamageDose * kEngineDamageDoseScale);
    g_damage.leftTail = std::max(0.0, g_damage.leftTail - kLuaDamageDose * kTailDamageDoseScale);
    g_damage.rightTail = std::max(0.0, g_damage.rightTail - kLuaDamageDose * kTailDamageDoseScale);
    g_damage.cockpit = std::max(0.0, g_damage.cockpit - kLuaDamageDose * kCockpitDamageDoseScale);

    if (g_damage.leftEngine < kFireDamageThreshold)
    {
        pushFireEvent(8, -4.45, 0.08, -1.7);
    }
    if (g_damage.rightEngine < kFireDamageThreshold)
    {
        pushFireEvent(7, -4.45, 0.08, 1.7);
    }

    initDamageLog();
    if (g_damageLog.is_open())
    {
        g_damageLog.write(
            "[LUA_DMG] Hit received! dose=%.2f | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
            kLuaDamageDose, g_damage.leftWing, g_damage.rightWing, g_damage.leftEngine, g_damage.rightEngine,
            g_damage.leftTail, g_damage.rightTail, g_damage.cockpit);
    }
}

void update_derived_damage_values()
{
    g_damage.wingAsymmetry = g_damage.leftWing - g_damage.rightWing;
    g_damage.totalWingLoss = 1.0 - (g_damage.leftWing + g_damage.rightWing) / 2.0;
    g_damage.totalTailLoss = 1.0 - (g_damage.leftTail + g_damage.rightTail) / 2.0;
    g_damage.engineAsymmetry = g_damage.leftEngine - g_damage.rightEngine;
    const double structuralIntegrity =
        (g_damage.nose + g_damage.leftFuselage + g_damage.rightFuselage + g_damage.belly + g_damage.leftTailStructure +
         g_damage.rightTailStructure + g_damage.leftWingInner + g_damage.rightWingInner + g_damage.leftWingOuter +
         g_damage.rightWingOuter) /
        10.0;
    g_damage.structuralDragFactor = 1.0 + kStructuralDragPenalty * (1.0 - structuralIntegrity);
}

double get_control_degradation()
{
    double controlDegradation = 1.0;
    if (g_damage.cockpit < 0.7)
    {
        controlDegradation = g_damage.cockpit / 0.7;
    }

    return controlDegradation;
}

void reset_damage_and_events()
{
    clear_pending_sim_events();
    // Explicitly stop existing emitters when DCS repairs/reinitializes the aircraft.
    for (int handle = 0; handle < 9; ++handle)
    {
        if (g_damage.fireActive[handle])
        {
            ed_fm_simulation_event event = {};
            event.event_type = ED_FM_EVENT_FIRE;
            event.event_params[0] = static_cast<float>(handle);
            event.event_params[8] = 0.0f;
            g_simEvents.push(event);
        }
    }
    g_damage = F117::DamageState();
}

void log_damage_lifecycle_event(const char* label)
{
    initDamageLog();
    if (g_damageLog.is_open())
    {
        g_damageLog.write("\n*** %s called ***\n", label);
    }
}

void log_simulation_frame(double dt, double acceleration_mps2)
{
    static int simFrameCount = 0;
    simFrameCount++;
    initDamageLog();
    if (g_damageLog.is_open())
    {
        g_damageLogTimer += dt;
        if (simFrameCount <= 500 || g_damageLogTimer >= 2.0)
        {
            g_damageLogTimer = 0.0;
            g_damageLog.write(
                "[SIM] frame=%d invincible=%d | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f | "
                "accel=%.1f events=%d WoW=%d\n",
                simFrameCount, static_cast<int>(F117::param_class.invincible_value), g_damage.leftWing,
                g_damage.rightWing, g_damage.leftEngine, g_damage.rightEngine, g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit, acceleration_mps2, static_cast<int>(g_simEvents.size()),
                static_cast<int>(F117::weight_on_wheels));
        }
    }
}

// DCS calls this to dequeue simulation events (fire, smoke, failures)
bool ed_fm_pop_simulation_event(ed_fm_simulation_event& out)
{
    if (g_simEvents.empty())
        return false;
    out = g_simEvents.front();
    g_simEvents.pop();
    if (g_damageLog.is_open())
    {
        g_damageLog.write(
            "ed_fm_pop_simulation_event: type=%u handle=%.0f pos=(%.1f,%.1f,%.1f) dir=(%.1f,%.1f,%.1f) speed=%.1f "
            "scale=%.1f\n",
            out.event_type, out.event_params[0], out.event_params[1], out.event_params[2], out.event_params[3],
            out.event_params[4], out.event_params[5], out.event_params[6], out.event_params[7], out.event_params[8]);
    }
    return true;
}

void ed_fm_on_damage(int element, double remainingIntegrity)
{
    initDamageLog();
    log_damage_callback_header(element, remainingIntegrity);
    if (!F117::param_class.invincible_value)
    {
        log_damage_callback_skipped_immortal();
        return;
    }
    if (!std::isfinite(remainingIntegrity))
        return;
    const double normalized = damage_integrity_from_dcs(remainingIntegrity);
    log_damage_mapping_result(apply_damage_for_element(element, normalized));
}

void ed_fm_repair()
{
    reset_damage_and_events();
}

void ed_fm_set_immortal(bool enabled)
{
    F117::param_class.set_immortal(enabled);
    initDamageLog();
    if (g_damageLog.is_open())
    {
        g_damageLog.write("ed_fm_set_immortal called: value=%d -> invincible_value=%d\n", static_cast<int>(enabled),
                          static_cast<int>(F117::param_class.invincible_value));
    }
}

// Returns true when any component has taken damage, triggering the repair process
bool ed_fm_need_to_be_repaired()
{
    return g_damage.leftWing < 1.0 || g_damage.rightWing < 1.0 || g_damage.leftEngine < 1.0 ||
           g_damage.rightEngine < 1.0 || g_damage.leftTail < 1.0 || g_damage.rightTail < 1.0 ||
           g_damage.cockpit < 1.0 || g_damage.leftOuterElevon < 1.0 || g_damage.rightOuterElevon < 1.0 ||
           g_damage.leftInnerElevon < 1.0 || g_damage.rightInnerElevon < 1.0 || g_damage.nose < 1.0 ||
           g_damage.leftFuselage < 1.0 || g_damage.rightFuselage < 1.0 || g_damage.belly < 1.0 ||
           g_damage.leftTailStructure < 1.0 || g_damage.rightTailStructure < 1.0 || g_damage.leftWingInner < 1.0 ||
           g_damage.rightWingInner < 1.0 || g_damage.leftWingOuter < 1.0 || g_damage.rightWingOuter < 1.0 ||
           g_damage.noseGear < 1.0 || g_damage.leftGear < 1.0 || g_damage.rightGear < 1.0 || g_damage.noseWheel < 1.0 ||
           g_damage.leftWheel < 1.0 || g_damage.rightWheel < 1.0;
}
