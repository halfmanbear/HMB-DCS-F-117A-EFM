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
constexpr double idle_egt_celsius = 420.0;
constexpr double max_egt_celsius = 850.0;
constexpr double start_egt_celsius = 600.0;
constexpr double egt_heat_tau = 2.5;   // s
constexpr double egt_cool_tau = 15.0;  // s
constexpr double idle_oil_psi = 70.0;
constexpr double max_oil_psi = 90.0;
constexpr double start_oil_psi = 90.0;
constexpr double oil_tau = 1.5;  // s
constexpr double kg_per_lb = 0.45359237;
constexpr double start_fuel_flow_kg_s = 600.0 * kg_per_lb / 3600.0;  // per engine
constexpr double idle_fuel_flow_kg_s = 0.2;  // both engines, TO 1F-117A-1 page A9-3 (1580 lb/hr)
}  // namespace

void update_fuel_system(double dt)
{
    // Fitted to the fuel used in TO 1F-117A-1's acceleration, climb and endurance tables, with the idle flow from
    // page A9-3 (1580 lb/hr). Net thrust at idle is small, so the flow can't simply follow thrust there.
    for (int side = 0; side < 2; ++side)
    {
        const F117::Engine_start::Sequencer& start = g_engine_starts.engine[side];
        double flow_kg_s = 0.0;
        if (start.running())
        {
            const double specific_fuel = 0.9928 + 0.0911 * F117::mach +
                                         0.0025 * F117::engines[side].thrust_fraction_of_max -
                                         0.1221 * F117::altitude_m / 10000.0;  // kg / kgf / hour
            flow_kg_s = (std::max)(idle_fuel_flow_kg_s / 2.0, (std::max)(F117::engine_thrust_n[side], 0.0) /
                                                                  standard_gravity_mps2 * specific_fuel / 3600.0);
        }
        else if (start.lit())
        {
            flow_kg_s = start_fuel_flow_kg_s;
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
    const double ambient = F117::ambient_temperature_deg_k > 150.0 ? F117::ambient_temperature_deg_k - 273.15 : 15.0;
    const double idle_n2 = F117::weight_on_wheels ? F117::Engine::ground_idle_n2 : F117::Engine::flight_idle_n2;
    for (int side = 0; side < 2; ++side)
    {
        const F117::Engine_start::Sequencer& start = g_engine_starts.engine[side];
        const double n2 = F117::engines[side].n2;
        double egt = ambient;
        double oil = idle_oil_psi * limit(n2 / idle_n2, 0.0, 1.0);  // driven by core rotation
        if (start.running())
        {
            const double above_idle = limit((n2 - idle_n2) / (F117::Engine::max_n2 - idle_n2), 0.0, 1.0);
            egt = idle_egt_celsius + above_idle * (max_egt_celsius - idle_egt_celsius);
            oil = idle_oil_psi + above_idle * (max_oil_psi - idle_oil_psi);
        }
        else if (start.lit())
        {
            egt = start_egt_celsius;
            oil = start_oil_psi * limit(n2 / F117::Engine_start::starter_cutout_n2, 0.0, 1.0);
        }
        double& shown_egt = F117::engine_egt_c[side];
        if (!std::isfinite(shown_egt) || !(dt > 0.0))
        {
            shown_egt = egt;  // first frame of a session: hot starts begin at idle, cold ones at ambient
            F117::engine_oil_psi[side] = oil;
        }
        else
        {
            const double egt_tau = egt > shown_egt ? egt_heat_tau : egt_cool_tau;
            shown_egt += (egt - shown_egt) * (1.0 - std::exp(-dt / egt_tau));
            F117::engine_oil_psi[side] += (oil - F117::engine_oil_psi[side]) * (1.0 - std::exp(-dt / oil_tau));
        }
    }
}

// Start phase per engine (0 off, 1 motoring, 2 light-off, 3 accelerating, 4 running), the count of starts
// refused for want of start air, and weight on wheels, for the cockpit Lua.
void publish_engine_start_params()
{
    static void* s_left_param = nullptr;
    static void* s_right_param = nullptr;
    static void* s_rejected_param = nullptr;
    static void* s_wow_param = nullptr;
    publish_named_param(s_left_param, "F117_ENGINE_START_L", g_engine_starts.engine[F117::Engine_start::left].phase);
    publish_named_param(s_right_param, "F117_ENGINE_START_R", g_engine_starts.engine[F117::Engine_start::right].phase);
    publish_named_param(s_rejected_param, "F117_ENGINE_START_REJECTED", g_engine_starts.rejected);
    publish_named_param(s_wow_param, "F117_WOW", F117::weight_on_wheels ? 1.0 : 0.0);
}

void update_propulsion_and_control_effectiveness(double dt, double tail_integrity)
{
    const double avg_engine_integrity = (g_damage.left_engine + g_damage.right_engine) / 2.0;
    F117::throttle_state =
        F117::Actuators::drive(F117::Actuators::throttle, F117::throttle_input * avg_engine_integrity, dt);

    // Each start sequence owns its engine's N2 until the starter cuts out; then the engine model takes over.
    const double integrity[2] = {g_damage.left_engine, g_damage.right_engine};
    bool can_run[2];
    double n2[2];
    for (int side = 0; side < 2; ++side)
    {
        can_run[side] = (F117::internal_fuel >= 5.0) && (integrity[side] > 0.05);
        n2[side] = F117::engines[side].n2;
    }
    g_engine_starts.update(dt, n2, external_start_air(), can_run);
    F117::engineswitch = g_engine_starts.any_running();

    F117::thrust_n = 0.0;
    for (int side = 0; side < 2; ++side)
    {
        F117::Engine::Engine_state& engine = F117::engines[side];
        const F117::Engine_start::Sequencer& start = g_engine_starts.engine[side];
        engine.n2 = n2[side];
        if (start.starting())
        {
            F117::engine_thrust_n[side] = 0.0;
            engine.thrust_fraction_of_max = 0.0;
        }
        else
        {
            F117::engine_thrust_n[side] = F117::Engine::engine_dynamics(
                engine, F117::throttle_input * integrity[side], F117::mach, F117::altitude_m, dt,
                start.running() && can_run[side], F117::weight_on_wheels);
        }
        F117::thrust_n += F117::engine_thrust_n[side];
    }
    update_engine_indications(dt);
    publish_engine_start_params();

    const double wing_integrity = 1.0 - g_damage.total_wing_loss;
    const double elevon_integrity = (g_damage.left_outer_elevon + g_damage.right_outer_elevon +
                                     g_damage.left_inner_elevon + g_damage.right_inner_elevon) /
                                    4.0;
    F117::aileron_pct = (F117::aileron_deg * wing_integrity * elevon_integrity) / F117::aileron_full_scale_deg;
    F117::elevator_pct = (F117::elevator_deg * (1.0 - g_damage.total_tail_loss) * elevon_integrity) / 25.0;

    const double tail_asymmetry = g_damage.left_tail - g_damage.right_tail;
    F117::rudder_pct = (F117::rudder_deg * tail_integrity) / 30.0 + tail_asymmetry * 0.1;
    F117::Flight_controls::update_yaw_debug_snapshot(F117::beta_deg, F117::yaw_rate_rps, F117::rudder_deg_commanded,
                                                     F117::rudder_deg, F117::rudder_pct);
}
