#pragma once

// Flight-model state shared by the translation units of the EFM. Every variable is an
// inline variable, so there is exactly one instance across the DLL.

#include "../EfmMath.h"
#include "../Engine/Engine.h"
#include "../param_functions.h"
#include "../include/Cockpit/CockpitAPI_Declare.h"

// Totals from the last aerodynamic update, with gear, chute, bay and ground effect included.
namespace F117
{
    namespace AERO
    {
        inline double Cx_total = 0.0;              // Drag along the body axis, positive aft
        inline double Cz_total = 0.0;              // Normal force, negative up
        inline double Cm_total = 0.0;
        inline double Cy_total = 0.0;
        inline double Cn_total = 0.0;
        inline double Cl_total = 0.0;
        inline double ClPerAileronDeg = 0.0;       // For the roll law
        inline double CmPerElevatorDeg = -0.0028;  // For the pitch law
        inline double CnPerRudderDeg = -0.002;     // For the yaw damper
        inline double Clp = 0.0;
    }
}

// Body-frame totals are returned at the DCS centre of mass. Moments are supplied separately.
inline Vec3 common_force{}, common_moment{}, center_of_gravity{}, inertia{};
inline Vec3 wind{}, wind_world_cs{}, position_world_cs{}, velocity_world_cs{};

namespace F117 {
    // Environment and measured motion (SI unless a suffix states otherwise).
    inline double ambientTemperature_DegK{}, ambientDensity_KgPerM3{};
    inline double ambientPressure_PA{101325.0}, altitude_m{};
    inline double calibratedAirspeed_KT{}, totalVelocity_MPS{}, dynamicPressure_PA{}, mach{};
    inline double alpha_DEG{}, beta_DEG{}, rollRate_RPS{}, pitchRate_RPS{}, yawRate_RPS{};
    inline double aircraftQuaternionX{}, aircraftQuaternionY{}, aircraftQuaternionZ{}, aircraftQuaternionW{1.0};
    inline double pitch_angle{}, roll_angle{}, vspeed{}, az{}, ay{}, ay_world{};
    inline double weight_N{131222.538};
    inline double surface_height_m{};
    inline bool surface_height_valid{}, altitude_valid{}, weight_on_wheels{}, wow_from_draw_args{};
    inline bool suspension_wow[3]{};

    // Geometry and unit conversions used by the current aero and controller models.
    inline constexpr double pi = 3.14159265358979323846;
    inline constexpr double radiansToDegrees = 180.0 / pi;
    inline constexpr double wingSpan_M = 13.205, wingArea_M2 = 84.77, meanChord_M = 6.422;

    // Pilot requests, commanded surfaces and actual surface feedback.
    inline double pitchTrim{}, rollTrim{}, yawTrim{}, roll_cmd{}, pedInput{};
    inline double elevator_DEG{}, aileron_DEG{}, rudder_DEG{};
    inline double elevator_DEG_commanded{}, aileron_DEG_commanded{}, rudder_DEG_commanded{};
    inline double aileron_PCT{}, rudder_PCT{}, elevator_PCT{};
    inline float elev_pos{}, rudder_pos{};
    inline int alt_hold{}, altroll_hold{}, horiz_hold{};
    inline double DeltaTime{};
    inline bool simInitialized{};

    // Landing gear, steering and ancillary mechanisms.
    inline double gearDown{}, GearCommand{}, WheelBrakeCommand{};
    inline double rolling_friction{0.015};
    // Normal/high-authority steering: TO 1F-117A-1, page 1-29.
    inline constexpr double kNoseWheelYawLow_RAD = 0.1745;
    inline constexpr double kNoseWheelYawHigh_RAD = 0.7854;
    inline constexpr double kNoseWheelYawRangeRate_RADPS = 0.6;
    inline double noseWheelYawRange_RAD{kNoseWheelYawLow_RAD};
    inline bool nwsAuthorityHeld{}, airRefuelDoorOpen{};
    inline double tailhook_command{}, tailhook_pos{}, dragchute_command{}, dragchute{};
    // Weapon bays: 0 is the left bay (station 1, arg 27), 1 is the right bay (station 2, arg 28).
    // 0 is closed and 1 is door open with the trapeze fully lowered.
    inline constexpr int kWeaponBayCount = 2;
    inline float bay_cmd[kWeaponBayCount]{}, bay_state[kWeaponBayCount]{};

    // Engine telemetry and the fuel mass already acknowledged by DCS.
    inline bool engineswitch{true};
    inline double throttle_state{0.1}, throttleInput{0.1}, thrust_N{};
    inline ENGINE::EngineState engines[2]{};
    inline double engine_thrust_N[2]{}, engine_fuel_flow_kg_s[2]{}, engine_egt_C[2]{}, engine_oil_psi[2]{};
    inline double fuel_mass_in_dcs{}, internal_fuel{}, external_fuel{};
    inline EDPARAM cockpitAPI{};
    inline param_stuff param_class{};

    // Weapon delivery state belongs to one aircraft session.
    inline bool weapon_release_pickle_held{}, weapon_release_gate{}, weapon_release_gate_fired{};
    // A bay is owned by the release when the pickle opened it, so the pickle closes it again.
    // A bay the pilot opened by hand is never owned and stays open until the pilot closes it.
    inline bool weapon_bay_owned_by_release[kWeaponBayCount]{}, weapon_bay_close_pending[kWeaponBayCount]{};
    inline bool master_armed{}, laser_armed{};
    inline double weapon_bay_close_delay[kWeaponBayCount]{};
    inline double weapon_release_gate_hold{}, release_retry_delay{};
    inline int release_status{};
    inline unsigned release_sequence{};
    inline double release_target[3]{};
    inline double laser_window{}, weapon_flight_time_remaining{}, ccrp_time_of_flight{};

    struct DamageState {
        double leftWing    = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double rightWing   = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double leftEngine  = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double rightEngine = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double leftTail    = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double rightTail   = 1.0;   // Remaining component integrity; damage is monotonic until repair.
        double cockpit     = 1.0;   // Remaining component integrity; damage is monotonic until repair.

        double leftOuterElevon = 1.0;
        double rightOuterElevon = 1.0;
        double leftInnerElevon = 1.0;
        double rightInnerElevon = 1.0;

        double nose = 1.0;
        double leftFuselage = 1.0;
        double rightFuselage = 1.0;
        double belly = 1.0;
        double leftTailStructure = 1.0;
        double rightTailStructure = 1.0;
        double leftWingInner = 1.0;
        double rightWingInner = 1.0;
        double leftWingOuter = 1.0;
        double rightWingOuter = 1.0;
        double noseGear = 1.0;
        double leftGear = 1.0;
        double rightGear = 1.0;
        double noseWheel = 1.0;
        double leftWheel = 1.0;
        double rightWheel = 1.0;
        bool fireActive[9] = {};

        // Derived values (updated each frame in ed_fm_simulate)
        double structuralDragFactor = 1.0;  // Cx multiplier from airframe section damage
        double wingAsymmetry   = 0.0;  // leftWing - rightWing
        double totalWingLoss   = 0.0;  // average damage across both wings
        double totalTailLoss   = 0.0;  // average damage across both tails
        double engineAsymmetry = 0.0;  // leftEngine - rightEngine
    };
}

// World-axis state captured from DCS and reused during the simulation step.
inline double ax_world{};
inline double az_world{};
inline double vx_world{};
inline double vy_world{};
inline double vz_world{};
