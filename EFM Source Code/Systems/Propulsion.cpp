#include "stdafx.h"
#include "Propulsion.h"

#include <algorithm>
#include <cmath>
#include "../Cockpit/ParamBridge.h"
#include "../Damage/Damage.h"
#include "../Engine/Engine.h"
#include "../Engine/EngineStart.h"
#include "../FlightControls/FlightControls.h"
#include "../FlightModel/State.h"
#include "../Actuators/Actuators.h"

namespace
{
// Engine indications. Start and idle values are from TO 1F-117A-1 page 2-7: EGT peaks at 550-630 C during
// the start and settles at 350-460 C at idle; fuel flow stays at about 600 lb/hr during the start; oil
// pressure peaks at 80-100 psi and settles at 60-80 psi. The maximum values are not from the manual.
constexpr double kIdleEgtCelsius = 420.0;
constexpr double kMaxEgtCelsius = 850.0;
constexpr double kStartEgtCelsius = 600.0;
constexpr double kEgtHeatTau = 2.5;   // s
constexpr double kEgtCoolTau = 15.0;  // s
constexpr double kIdleOilPsi = 70.0;
constexpr double kMaxOilPsi = 90.0;
constexpr double kStartOilPsi = 90.0;
constexpr double kOilTau = 1.5;  // s
constexpr double kKgPerLb = 0.45359237;
constexpr double kStartFuelFlowKgS = 600.0 * kKgPerLb / 3600.0;  // per engine
constexpr double kIdleFuelFlowKgS = 0.2;                         // both engines, TO 1F-117A-1 page A9-3 (1580 lb/hr)
}  // namespace

void update_fuel_system(double dt)
{
    // Fitted to the fuel used in TO 1F-117A-1's acceleration, climb and endurance tables, with the idle flow from
    // page A9-3 (1580 lb/hr). Net thrust at idle is small, so the flow can't simply follow thrust there.
    for (int side = 0; side < 2; ++side)
    {
        const F117::ENGINE_START::Sequencer& start = g_engineStarts.engine[side];
        double flow_kg_s = 0.0;
        if (start.running())
        {
            const double specific_fuel = 0.9928 + 0.0911 * F117::mach +
                                         0.0025 * F117::engines[side].thrustFractionOfMax -
                                         0.1221 * F117::altitude_m / 10000.0;  // kg / kgf / hour
            flow_kg_s = (std::max)(kIdleFuelFlowKgS / 2.0, (std::max)(F117::engine_thrust_N[side], 0.0) /
                                                               kStandardGravity_MPS2 * specific_fuel / 3600.0);
        }
        else if (start.lit())
        {
            flow_kg_s = kStartFuelFlowKgS;
        }
        F117::engine_fuel_flow_kg_s[side] = std::isfinite(flow_kg_s) ? flow_kg_s : 0.0;
    }
    // Unlimited fuel burns nothing, so the quantity and DCS's mass both stay put.
    if (!F117::param_class.fuelvalue || !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(F117::internal_fuel) ||
        F117::internal_fuel <= 0.0)
        return;
    const double requested = (F117::engine_fuel_flow_kg_s[0] + F117::engine_fuel_flow_kg_s[1]) * dt;
    const double consumed = (std::min)(F117::internal_fuel, requested);
    F117::internal_fuel -= consumed;
}

// EGT and oil pressure follow each engine's state with a lag, so the start peak and the settle to idle show.
void update_engine_indications(double dt)
{
    const double ambient = F117::ambientTemperature_DegK > 150.0 ? F117::ambientTemperature_DegK - 273.15 : 15.0;
    const double idleN2 = F117::weight_on_wheels ? F117::ENGINE::GROUND_IDLE_N2 : F117::ENGINE::FLIGHT_IDLE_N2;
    for (int side = 0; side < 2; ++side)
    {
        const F117::ENGINE_START::Sequencer& start = g_engineStarts.engine[side];
        const double n2 = F117::engines[side].N2;
        double egt = ambient;
        double oil = kIdleOilPsi * limit(n2 / idleN2, 0.0, 1.0);  // driven by core rotation
        if (start.running())
        {
            const double aboveIdle = limit((n2 - idleN2) / (F117::ENGINE::MAX_N2 - idleN2), 0.0, 1.0);
            egt = kIdleEgtCelsius + aboveIdle * (kMaxEgtCelsius - kIdleEgtCelsius);
            oil = kIdleOilPsi + aboveIdle * (kMaxOilPsi - kIdleOilPsi);
        }
        else if (start.lit())
        {
            egt = kStartEgtCelsius;
            oil = kStartOilPsi * limit(n2 / F117::ENGINE_START::STARTER_CUTOUT_N2, 0.0, 1.0);
        }
        double& shownEgt = F117::engine_egt_C[side];
        if (!std::isfinite(shownEgt) || !(dt > 0.0))
        {
            shownEgt = egt;  // first frame of a session: hot starts begin at idle, cold ones at ambient
            F117::engine_oil_psi[side] = oil;
            continue;
        }
        const double egtTau = egt > shownEgt ? kEgtHeatTau : kEgtCoolTau;
        shownEgt += (egt - shownEgt) * (1.0 - std::exp(-dt / egtTau));
        F117::engine_oil_psi[side] += (oil - F117::engine_oil_psi[side]) * (1.0 - std::exp(-dt / kOilTau));
    }
}

// Start phase per engine (0 off, 1 motoring, 2 light-off, 3 accelerating, 4 running), the count of starts
// refused for want of start air, and weight on wheels, for the cockpit Lua.
void publish_engine_start_params()
{
    static void* s_leftParam = nullptr;
    static void* s_rightParam = nullptr;
    static void* s_rejectedParam = nullptr;
    static void* s_wowParam = nullptr;
    publish_named_param(s_leftParam, "F117_ENGINE_START_L", g_engineStarts.engine[F117::ENGINE_START::LEFT].phase);
    publish_named_param(s_rightParam, "F117_ENGINE_START_R", g_engineStarts.engine[F117::ENGINE_START::RIGHT].phase);
    publish_named_param(s_rejectedParam, "F117_ENGINE_START_REJECTED", g_engineStarts.rejected);
    publish_named_param(s_wowParam, "F117_WOW", F117::weight_on_wheels ? 1.0 : 0.0);
}

void update_propulsion_and_control_effectiveness(double dt, double tailIntegrity)
{
    const double avgEngineIntegrity = (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
    F117::throttle_state =
        F117::ACTUATORS::drive(F117::ACTUATORS::throttle, F117::throttleInput * avgEngineIntegrity, dt);

    // Each start sequence owns its engine's N2 until the starter cuts out; then the engine model takes over.
    const double integrity[2] = {g_damage.leftEngine, g_damage.rightEngine};
    bool canRun[2];
    double n2[2];
    for (int side = 0; side < 2; ++side)
    {
        canRun[side] = (F117::internal_fuel >= 5.0) && (integrity[side] > 0.05);
        n2[side] = F117::engines[side].N2;
    }
    g_engineStarts.update(dt, n2, external_start_air(), canRun);
    F117::engineswitch = g_engineStarts.any_running();

    F117::thrust_N = 0.0;
    for (int side = 0; side < 2; ++side)
    {
        F117::ENGINE::EngineState& engine = F117::engines[side];
        const F117::ENGINE_START::Sequencer& start = g_engineStarts.engine[side];
        engine.N2 = n2[side];
        if (start.starting())
        {
            F117::engine_thrust_N[side] = 0.0;
            engine.thrustFractionOfMax = 0.0;
        }
        else
        {
            F117::engine_thrust_N[side] = F117::ENGINE::engine_dynamics(
                engine, F117::throttleInput * integrity[side], F117::mach, F117::altitude_m, dt,
                start.running() && canRun[side], F117::weight_on_wheels);
        }
        F117::thrust_N += F117::engine_thrust_N[side];
    }
    update_engine_indications(dt);
    publish_engine_start_params();

    const double wingIntegrity = 1.0 - g_damage.totalWingLoss;
    const double elevonIntegrity =
        (g_damage.leftOuterElevon + g_damage.rightOuterElevon + g_damage.leftInnerElevon + g_damage.rightInnerElevon) /
        4.0;
    F117::aileron_PCT = (F117::aileron_DEG * wingIntegrity * elevonIntegrity) / F117::kAileronFullScale_DEG;
    F117::elevator_PCT = (F117::elevator_DEG * (1.0 - g_damage.totalTailLoss) * elevonIntegrity) / 25.0;

    const double tailAsymmetry = g_damage.leftTail - g_damage.rightTail;
    F117::rudder_PCT = (F117::rudder_DEG * tailIntegrity) / 30.0 + tailAsymmetry * 0.1;
    F117::FLIGHTCONTROLS::update_yaw_debug_snapshot(F117::beta_DEG, F117::yawRate_RPS, F117::rudder_DEG_commanded,
                                                    F117::rudder_DEG, F117::rudder_PCT);
}
