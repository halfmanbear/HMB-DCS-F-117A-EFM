#include "stdafx.h"
#include "F_117_NightHawk.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <queue>
#include <string>

#include "EfmMath.h"
#include "Inputs.h" // DCS and aircraft command protocol
#include "param_functions.h" // Mission options
#include "include/Cockpit/CockpitAPI_Declare.h" // Lua parameter bridge
#include "Aerodynamics/AeroModel.h"
#include "Actuators/Actuators.h" // Airframe motion
#include "Atmosphere/Atmosphere.h" // Air data
#include "Engine/Engine.h" // Installed propulsion
#include "Engine/EngineStart.h"
#include "FlightControls/FlightControls.h" // Control laws
#include "Avionics/Autothrottle.h"
#include "Avionics/Paars.h"
#include "Avionics/JdamEnvelope.h"
#include "Avionics/CockpitBase_Interop.h"
#include "Avionics/Weapons_Interop.h"

using namespace F117; // Shared names used by the subsystem adapters below.


// Totals from the last aerodynamic update, with gear, chute, bay and ground effect included.
namespace F117
{
	namespace AERO
	{
		double Cx_total = 0.0;              // Drag along the body axis, positive aft
		double Cz_total = 0.0;              // Normal force, negative up
		double Cm_total = 0.0;
		double Cy_total = 0.0;
		double Cn_total = 0.0;
		double Cl_total = 0.0;
		double ClPerAileronDeg = 0.0;       // For the roll law
		double CmPerElevatorDeg = -0.0028;  // For the pitch law
		double CnPerRudderDeg = -0.002;     // For the yaw damper
		double Clp = 0.0;
	}
}

// Body-frame totals are returned at the DCS centre of mass. Moments are supplied separately.
Vec3 common_force{}, common_moment{}, center_of_gravity{}, inertia{};
Vec3 wind{}, wind_world_cs{}, position_world_cs{}, velocity_world_cs{};

namespace F117 {
    // Environment and measured motion (SI unless a suffix states otherwise).
    double ambientTemperature_DegK{}, ambientDensity_KgPerM3{};
    double ambientPressure_PA{101325.0}, altitude_m{};
    double calibratedAirspeed_KT{}, totalVelocity_MPS{}, dynamicPressure_PA{}, mach{};
    double alpha_DEG{}, beta_DEG{}, rollRate_RPS{}, pitchRate_RPS{}, yawRate_RPS{};
    double aircraftQuaternionX{}, aircraftQuaternionY{}, aircraftQuaternionZ{}, aircraftQuaternionW{1.0};
    double pitch_angle{}, roll_angle{}, vspeed{}, az{}, ay{}, ay_world{};
    double weight_N{131222.538};
    double surface_height_m{};
    bool surface_height_valid{}, altitude_valid{}, weight_on_wheels{}, wow_from_draw_args{};
    bool suspension_wow[3]{};

    // Geometry and unit conversions used by the current aero and controller models.
    constexpr double pi = 3.14159265358979323846;
    constexpr double radiansToDegrees = 180.0 / pi;
    constexpr double wingSpan_M = 13.205, wingArea_M2 = 84.77, meanChord_M = 6.422;

    // Pilot requests, commanded surfaces and actual surface feedback.
    double pitchTrim{}, rollTrim{}, yawTrim{}, roll_cmd{}, pedInput{};
    double elevator_DEG{}, aileron_DEG{}, rudder_DEG{};
    double elevator_DEG_commanded{}, aileron_DEG_commanded{}, rudder_DEG_commanded{};
    double aileron_PCT{}, rudder_PCT{}, elevator_PCT{};
    float elev_pos{}, rudder_pos{};
    int alt_hold{}, altroll_hold{}, horiz_hold{};
    double DeltaTime{};
    bool simInitialized{};

    // Landing gear, steering and ancillary mechanisms.
    double gearDown{}, GearCommand{}, WheelBrakeCommand{};
    double rolling_friction{0.015};
    // Normal/high-authority steering: TO 1F-117A-1, page 1-29.
    constexpr double kNoseWheelYawLow_RAD = 0.1745;
    constexpr double kNoseWheelYawHigh_RAD = 0.7854;
    constexpr double kNoseWheelYawRangeRate_RADPS = 0.6;
    double noseWheelYawRange_RAD{kNoseWheelYawLow_RAD};
    bool nwsAuthorityHeld{}, airRefuelDoorOpen{};
    double tailhook_command{}, tailhook_pos{}, dragchute_command{}, dragchute{};
    float misc_cmd{}, misc_state{};

    // Engine telemetry and the fuel mass already acknowledged by DCS.
    bool engineswitch{true};
    double throttle_state{0.1}, throttleInput{0.1}, thrust_N{};
    ENGINE::EngineState engines[2]{};
    double engine_thrust_N[2]{}, engine_fuel_flow_kg_s[2]{}, engine_egt_C[2]{}, engine_oil_psi[2]{};
    double fuel_mass_in_dcs{}, internal_fuel{}, external_fuel{};
    EDPARAM cockpitAPI{};
    param_stuff param_class{};

    // Weapon delivery state belongs to one aircraft session.
    bool weapon_release_pickle_held{}, weapon_release_gate{}, weapon_release_gate_fired{};
    bool weapon_bay_owned_by_release{}, weapon_bay_close_pending{}, master_armed{}, laser_armed{};
    double weapon_bay_close_delay{}, weapon_release_gate_hold{}, release_retry_delay{};
    int release_status{};
    unsigned release_sequence{};
    double release_target[3]{};
    double laser_window{}, weapon_flight_time_remaining{}, ccrp_time_of_flight{};

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

static F117::DamageState g_damage;
static std::queue<ed_fm_simulation_event> g_simEvents;
static Autothrottle::Controller g_autothrottle;
static Paars::Controller g_paars;
static F117::ENGINE_START::Starts g_engineStarts;
static bool g_paarsEngagedAutothrottle = false; // PAARS moved the power levers by engaging A/T
static double g_paarsRollRateDegS = 0.0;
static constexpr double kAileronFullScale_DEG = 25.5; // Aileron deflection that aileron_PCT = 1 stands for.
static constexpr double kPitchLawCmPerDeg = 0.0105;   // Elevon pitch power the pitch law's gains were tuned with.
static constexpr double kPitchDamperCmq = -5.0;       // Pitch damping the flight controls add, per unit of q c / 2V.
static void* g_weaponReleaseGateParam = nullptr;
static bool g_weaponReleaseGateParamInitialized = false;
static void* g_iradsLockValidParam = nullptr;
static void* g_iradsLockWorldXParam = nullptr;
static void* g_iradsLockWorldYParam = nullptr;
static void* g_iradsLockWorldZParam = nullptr;
static void* g_iradsLockRangeParam = nullptr;
static void* g_iradsLockAzimuthParam = nullptr;
static void* g_iradsLockElevationParam = nullptr;
static bool g_iradsLockParamsInitialized = false;
static void* g_releaseCueValidParam = nullptr;
static void* g_releaseCueErrorParam = nullptr;
static void* g_releaseCueTimeToGoParam = nullptr;
static void* g_releaseCueRangeParam = nullptr;
static void* g_releaseCueInZoneParam = nullptr;
static bool g_releaseCueParamsInitialized = false;
static void* g_ccrpProfileParam = nullptr;
static bool g_ccrpProfileParamInitialized = false;


namespace
{
    // How a store decides it is in its release zone.
    //  Jdam      GPS bomb, stock DCS min/max release range table
    //  Lgb       laser-guided bomb, window around the ballistic impact, laser must fire
    //  Unguided  free-fall bomb, released on the CCRP cue
    //  Wcmd      wind-corrected dispenser, wide window around the ballistic impact
    //  Glide     glide weapon (JSOW), stock DCS min/max release range table
    //  Missile*  powered missile, slant range and seeker cone
    enum class StoreKind { Jdam, Lgb, Unguided, Wcmd, Glide, MissileIr, MissileLaser };

    struct CcrpStoreModel
    {
        const char* name;
        double massKg;
        double dragCd;
        double referenceAreaM2;
        double releaseWindowMeters;
        double minSpeedMPS;
        double minDropMeters;
        StoreKind kind;
        int envelope;               // JdamEnvelope / GlideEnvelope table, or -1
        double minRangeMeters;      // missiles only
        double maxRangeMeters;      // missiles only
        double seekerHalfAngleRad;  // missiles only
    };

    // Index = F117_CCRP_PROFILE, set by Cockpit/Scripts/Systems/Weapons.lua classify().
    // Keep the two lists in the same order. Profiles 0-4 must never move.
    // Stores 5-13 take mass and caliber from the stock DCS weapon tables; missile
    // ranges and seeker cones come from the stock AGM-65 definitions. The MK-82 and
    // MK-84 drag coefficients are fitted to impacts measured in DCS from 4500 m at
    // 218 m/s (tools/missions/store_drop_test.py in the mod repo).
    constexpr CcrpStoreModel kCcrpStoreModels[] =
    {
        { "GBU-31(V)1/B",   934.0, 0.00264,  0.45,    75.0, 30.0,  10.0, StoreKind::Jdam,          0,    0.0,     0.0, 0.0 },
        { "GBU-31(V)3/B",   981.0, 0.00170,  0.45,    90.0, 30.0,  10.0, StoreKind::Jdam,          1,    0.0,     0.0, 0.0 },
        { "GBU-32(V)2/B",   467.0, 0.00035,  0.30,    65.0, 30.0,  10.0, StoreKind::Jdam,          2,    0.0,     0.0, 0.0 },
        { "GBU-12",         277.0, 0.000413, 0.22,    55.0, 30.0,  10.0, StoreKind::Lgb,          -1,    0.0,     0.0, 0.0 },
        { "GBU-27",         991.0, 0.00071,  0.45,    80.0, 30.0,  10.0, StoreKind::Lgb,          -1,    0.0,     0.0, 0.0 },
        { "GBU-10",         959.0, 0.00071,  0.45,    80.0, 30.0,  10.0, StoreKind::Lgb,          -1,    0.0,     0.0, 0.0 },
        { "MK-82",          228.0, 0.151,    0.0585,  15.0, 30.0,  10.0, StoreKind::Unguided,     -1,    0.0,     0.0, 0.0 },
        { "MK-84",          908.0, 0.256,    0.164,   20.0, 30.0,  10.0, StoreKind::Unguided,     -1,    0.0,     0.0, 0.0 },
        { "CBU-105",        420.0, 0.55,     0.1233, 150.0, 30.0, 300.0, StoreKind::Wcmd,         -1,    0.0,     0.0, 0.0 },
        { "AGM-154A",       485.0, 0.0018,   0.1257,   0.0, 30.0, 300.0, StoreKind::Glide,         0,    0.0,     0.0, 0.0 },
        { "AGM-154B",       485.0, 0.0018,   0.1257,   0.0, 30.0, 300.0, StoreKind::Glide,         1,    0.0,     0.0, 0.0 },
        { "AGM-154C",       484.0, 0.0018,   0.1257,   0.0, 30.0, 300.0, StoreKind::Glide,         0,    0.0,     0.0, 0.0 },
        { "AGM-65D",        218.0, 0.0,      0.073,    0.0, 30.0,  10.0, StoreKind::MissileIr,    -1,  500.0, 12964.0, 0.5 },
        { "AGM-65E",        293.0, 0.0,      0.073,    0.0, 30.0,  10.0, StoreKind::MissileLaser, -1, 1000.0, 11112.0, 0.5 },
    };
    constexpr int kCcrpStoreCount = (int)(sizeof(kCcrpStoreModels) / sizeof(kCcrpStoreModels[0]));

    constexpr bool store_needs_laser(StoreKind kind) { return kind == StoreKind::Lgb || kind == StoreKind::MissileLaser; }
    constexpr bool store_is_missile(StoreKind kind) { return kind == StoreKind::MissileIr || kind == StoreKind::MissileLaser; }
    // Bombs whose impact point comes from the ballistic integration below.
    constexpr bool store_falls_ballistic(StoreKind kind) { return kind != StoreKind::Glide && !store_is_missile(kind); }

    std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath)
    {
        const char* userProfile = getenv("USERPROFILE");
        if (userProfile == nullptr)
        {
            return std::string();
        }

        return std::string(userProfile) + "\\Saved Games\\" + dcsFolderName + "\\" + relativePath;
    }

    constexpr double kGearCommandDown = 1.0;
    constexpr double kGearCommandUp = 0.0;
    constexpr double kGearToggleThreshold = 0.5;
    constexpr double kWeightOnWheelsDrawArgThreshold = 0.5;

    constexpr double kRollTrimStep = 0.02;
    constexpr double kPitchTrimStep = 0.05;
    constexpr double kYawTrimStep = 0.05;
    constexpr double kThrottleStep = 0.5;
    // Fraction of wheel_brake_moment_max in FM/config.lua. Set by the landing distances in TO 1F-117A-1
    // (Figure A8-2), checked with tools/fm_bench/dcs_check.py --landing in the mod repo.
    constexpr double kWheelBrakeOnFriction = 0.32;
    constexpr double kWheelBrakeOffFriction = 0.015;
    constexpr double kToggleLowThreshold = 0.25;
    constexpr double kToggleHighThreshold = 0.75;
    constexpr double kWeaponBayOpenThreshold = 0.995;
    constexpr double kWeaponBayClosedThreshold = 0.005;
    constexpr double kWeaponBayHoldAfterReleaseSeconds = 1.0;
    constexpr double kWeaponReleaseGateMinHoldSeconds = 0.25; // Long enough for cockpit Lua update rates to sample the gate.
    constexpr double kLaserOverrunSeconds = 4.0;              // keep lasing this long past predicted impact
    constexpr char kEfmLaunchActiveParamName[] = "F117_EFM_LAUNCH_ACTIVE";
    constexpr char kLaserArmedParamName[] = "F117_LASER_ARMED";
    constexpr char kLaserFiringParamName[] = "F117_LASER_FIRING";
    constexpr char kWeaponReleaseGateParamName[] = "F117_WEAPON_RELEASE_GATE";
    constexpr char kIRADSLockValidParamName[] = "F117_IRADS_LOCK_VALID";
    constexpr char kIRADSLockWorldXParamName[] = "F117_IRADS_LOCK_X";
    constexpr char kIRADSLockWorldYParamName[] = "F117_IRADS_LOCK_Y";
    constexpr char kIRADSLockWorldZParamName[] = "F117_IRADS_LOCK_Z";
    constexpr char kIRADSLockRangeParamName[] = "F117_IRADS_LOCK_RANGE";
    constexpr char kIRADSLockAzimuthParamName[] = "F117_IRADS_LOCK_AZ";
    constexpr char kIRADSLockElevationParamName[] = "F117_IRADS_LOCK_EL";
    constexpr char kReleaseCueValidParamName[] = "F117_RELEASE_CUE_VALID";
    constexpr char kReleaseCueErrorParamName[] = "F117_RELEASE_CUE_ERROR";
    constexpr char kReleaseCueTimeToGoParamName[] = "F117_RELEASE_CUE_TTG";
    constexpr char kReleaseCueRangeParamName[] = "F117_RELEASE_CUE_RANGE";
    constexpr char kReleaseCueInZoneParamName[] = "F117_RELEASE_CUE_IN_ZONE";
    constexpr char kCcrpProfileParamName[] = "F117_CCRP_PROFILE";
    constexpr double kGravity_MPS2 = 9.81;
    constexpr double kReleaseCueMinSpeedMPS = 30.0;
    constexpr double kReleaseCueMinDropMeters = 10.0;
    constexpr double kAtmosphereScaleHeight_M = 8500.0;
    constexpr double kReleaseIntegrationStepSeconds = 0.02;
    constexpr double kReleaseIntegrationMaxSeconds = 180.0;
    constexpr double kUnguidedMaxCrossTrackMeters = 30.0;  // farther off the line misses the target
    constexpr double kGlideWeaponSpeedMPS = 200.0;         // JSOW mid-glide speed, for the laser/flight timer
    constexpr double kMissileAverageSpeedMPS = 300.0;      // AGM-65 average speed, for the laser/flight timer

    constexpr double kLuaDamageDose = 0.25;
    constexpr double kWingDamageDoseScale = 0.6;
    constexpr double kEngineDamageDoseScale = 0.4;
    constexpr double kTailDamageDoseScale = 0.3;
    constexpr double kCockpitDamageDoseScale = 0.2;
    constexpr double kFireDamageThreshold = 0.35;

    constexpr size_t kMirroredGearDrawArgRequiredSize = 616;

    constexpr int kGearArgNose = 0;
    constexpr int kGearArgLeft = 3;
    constexpr int kGearArgRight = 5;
    constexpr int kWowArgNose = 1;
    constexpr int kWowArgLeft = 4;
    constexpr int kWowArgRight = 6;

    constexpr int kLeftAileronArg = 11;
    constexpr int kRightAileronArg = 12;
    constexpr int kLeftElevatorArg = 15;
    constexpr int kRightElevatorArg = 16;
    constexpr int kLeftRudderArg = 17;
    constexpr int kRightRudderArg = 18;
    constexpr int kTailhookArg = 25;
    constexpr int kWeaponBayArg = 26;
    constexpr int kDragChuteArg = 35;
    constexpr int kMirroredGearArgNose = 611;
    constexpr int kMirroredGearArgLeft = 614;
    constexpr int kMirroredGearArgRight = 616;



    constexpr int kFc3CockpitPitchArg = 1000;
    constexpr int kFc3CockpitRollArg = 1001;
    constexpr int kFc3CockpitThrottleArg = 1002;
    constexpr int kFc3CockpitRudderArg = 1003;

    constexpr double kGroundStartIdleN2 = 62.0;
    constexpr double kAirStartThrottle = 77.5;
    constexpr double kAirStartN2 = 92.0;
    constexpr double kMaxThrustNewtons = 92000.0;
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
    constexpr double kOilTau = 1.5;       // s
    constexpr double kPaPerPsi = 6894.757;
    constexpr double kKgPerLb = 0.45359237;
    constexpr double kStartFuelFlowKgS = 600.0 * kKgPerLb / 3600.0; // per engine
    constexpr double kIdleFuelFlowKgS = 0.2;  // both engines, TO 1F-117A-1 page A9-3 (1580 lb/hr)
    constexpr double kAirframeOxygenSupply = 1000.0;
    constexpr double kDefaultFlowVelocity = 100.0;

    constexpr int kDamageElementNose = 0;
    constexpr int kDamageElementCockpit = 3;
    constexpr int kDamageElementNoseGear = 8;
    constexpr int kDamageElementFuselageLeft = 9;
    constexpr int kDamageElementFuselageRight = 10; // Also DCS "MAIN".
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
    // DCS 2.9 stops reporting a destroyed cell at 0.1, never 0, and sends no callback for its
    // deps_cells. Measured with live blasts in DCS 2.9.31.
    constexpr double kDamageCellDestroyedIntegrity = 0.101;
    // Outer panel share of one wing's planform, from the section stations in the EDM.
    constexpr double kOuterWingLiftShare = 0.30;
    // Added drag with every airframe section destroyed. Gameplay value, not flight-test data.
    constexpr double kStructuralDragPenalty = 0.30;
    constexpr int kDamageElementAileronLeft = 25;
    constexpr int kDamageElementAileronRight = 26;
    constexpr int kDamageElementFlapLeft = 37;
    constexpr int kDamageElementFlapRight = 38;
    constexpr int kDamageElementRudderLeft = 53;
    constexpr int kDamageElementRudderRight = 54;

    bool gear_retraction_blocked()
    {
        return F117::weight_on_wheels || F117::wow_from_draw_args;
    }

    double clamp_gear_command_for_weight_on_wheels(double requestedCommand)
    {
        if (gear_retraction_blocked() && requestedCommand < kGearCommandDown)
        {
            return kGearCommandDown;
        }

        return requestedCommand;
    }

    void set_gear_command(double requestedCommand)
    {
        F117::GearCommand = clamp_gear_command_for_weight_on_wheels(requestedCommand);
    }

    // Weight on wheels is wheel contact only: a compressed strut reported by DCS, or the strut draw args. Lift below
    // weight is not a test for it, because that is also true in the air with the gear down, through the whole climb
    // after takeoff, and it held the gear handle down there.
    void update_weight_on_wheels()
    {
        const bool suspensionWeightOnWheels = F117::suspension_wow[0] || F117::suspension_wow[1] || F117::suspension_wow[2];
        F117::weight_on_wheels = F117::wow_from_draw_args || suspensionWeightOnWheels;
        if (F117::weight_on_wheels)
        {
            F117::GearCommand = kGearCommandDown;
        }
    }
}

// World-axis state captured from DCS and reused during the simulation step.
double ax_world{};
double az_world{};
double vx_world{};
double vy_world{};
double vz_world{};

// Accumulate a body-axis force for the current frame.
// The position parameter is ignored here; forces are applied at the current center of mass.
void add_local_force(const Vec3& force)
{
    common_force = Vec3(common_force.x + force.x, common_force.y + force.y, common_force.z + force.z);
}

// Accumulate a body-axis moment for the current frame.
void add_local_moment(const Vec3& moment)
{
    common_moment = Vec3(common_moment.x + moment.x, common_moment.y + moment.y, common_moment.z + moment.z);
}


// Return the accumulated body-axis force to DCS after the current simulation step.
void ed_fm_add_local_force(double& fx, double& fy, double& fz,
    double& applicationX, double& applicationY, double& applicationZ)
{
    // The component model supplies its moments explicitly; do not add r cross F here.
    const Vec3 totals = common_force;
    const Vec3 application = center_of_gravity;
    fx = totals.x; fy = totals.y; fz = totals.z;
    applicationX = application.x; applicationY = application.y; applicationZ = application.z;
}

void ed_fm_add_global_force(double& fx, double& fy, double& fz,
    double& px, double& py, double& pz)
{
    fx = fy = fz = px = py = pz = 0.0;
}

void ed_fm_add_global_moment(double& mx, double& my, double& mz)
{
    mx = my = mz = 0.0;
}

// Return the accumulated body-axis moment to DCS after the current simulation step.
void ed_fm_add_local_moment(double& mx, double& my, double& mz)
{
    const Vec3 totals = common_moment;
    mx = totals.x; my = totals.y; mz = totals.z;
}



float aoa_filter{}, aos_filter{};

// Forward declarations for damage debug logging
static FILE* g_damageLog = nullptr;
static void initDamageLog();
static double g_damageLogTimer = 0.0;

// Forward declaration for fire event helper
static void pushFireEvent(int handle, double x, double y, double z);

namespace
{
    bool handle_roll_command(int command, float value)
    {
        if (command == trimLeft || command == trimRight)
        {
            F117::rollTrim += command == trimLeft ? kRollTrimStep : -kRollTrimStep;
            return true;
        }
        double demand;
        if (command == JoystickRoll) demand = limit(value, -1.0, 1.0);
        else if (command == RollLeft || command == RollRight)
            demand = (-value + (command == RollLeft ? -0.025 : 0.025)) / 2.0 * 100.0;
        else if (command == RollLeftStop || command == RollRightStop)
        {
            F117::FLIGHTCONTROLS::latStickInput = 0;
            return true;
        }
        else return false;
        F117::roll_cmd = demand;
        F117::FLIGHTCONTROLS::latStickInput = demand;
        return true;
    }

    bool handle_pitch_command(int command, float value)
    {
        if (command == trimUp || command == trimDown)
        {
            F117::pitchTrim += command == trimUp ? -kPitchTrimStep : kPitchTrimStep;
            return true;
        }
        double demand;
        if (command == JoystickPitch) demand = limit(-value, -1.0, 1.0);
        else if (command == PitchUp || command == PitchDown) demand = command == PitchUp ? -1.0 : 1.0;
        else if (command == PitchUpStop || command == PitchDownStop) demand = 0.0;
        else return false;
        F117::FLIGHTCONTROLS::longStickInput = demand;
        return true;
    }

    bool handle_yaw_command(int command, float value)
    {
        if (command == NwsAuthorityOn || command == NwsAuthorityOff)
        {
            F117::nwsAuthorityHeld = command == NwsAuthorityOn;
            return true;
        }
        if (command == ruddertrimLeft || command == ruddertrimRight)
        {
            F117::yawTrim += command == ruddertrimLeft ? kYawTrimStep : -kYawTrimStep;
            return true;
        }
        double demand;
        if (command == JoystickYaw) demand = -value * (501.0 / (beta_DEG * beta_DEG + 500.0));
        else if (command == rudderleft || command == rudderright)
        {
            const double magnitude = value + 101.0 / (beta_DEG * beta_DEG + 100.0);
            demand = command == rudderleft ? magnitude : -magnitude;
        }
        else if (command == rudderleftend || command == rudderrightend) demand = 0.0;
        else return false;
        F117::pedInput = limit(demand, -1.0, 1.0);
        return true;
    }

    void apply_lua_damage_hit()
    {
        if (F117::param_class.invincible_value == 0)
        {
            return;
        }

        g_damage.leftWing = max(0.0, g_damage.leftWing - kLuaDamageDose * kWingDamageDoseScale);
        g_damage.rightWing = max(0.0, g_damage.rightWing - kLuaDamageDose * kWingDamageDoseScale);
        g_damage.leftEngine = max(0.0, g_damage.leftEngine - kLuaDamageDose * kEngineDamageDoseScale);
        g_damage.rightEngine = max(0.0, g_damage.rightEngine - kLuaDamageDose * kEngineDamageDoseScale);
        g_damage.leftTail = max(0.0, g_damage.leftTail - kLuaDamageDose * kTailDamageDoseScale);
        g_damage.rightTail = max(0.0, g_damage.rightTail - kLuaDamageDose * kTailDamageDoseScale);
        g_damage.cockpit = max(0.0, g_damage.cockpit - kLuaDamageDose * kCockpitDamageDoseScale);

        if (g_damage.leftEngine < kFireDamageThreshold)
        {
            pushFireEvent(8, -4.45, 0.08, -1.7);
        }
        if (g_damage.rightEngine < kFireDamageThreshold)
        {
            pushFireEvent(7, -4.45, 0.08, 1.7);
        }

        initDamageLog();
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "[LUA_DMG] Hit received! dose=%.2f | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                kLuaDamageDose,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit);
            fflush(g_damageLog);
        }
    }
    // Lazily resolve and publish a cockpit param by name.
    void publish_named_param(void*& handle, const char* name, double value)
    {
        if (handle == nullptr)
        {
            F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
            if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
                F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
            {
                return;
            }
            handle = F117::cockpitAPI.getParamHandle(name);
        }
        if (handle != nullptr)
        {
            F117::cockpitAPI.setParamNumber(handle, value);
        }
    }

    // Lazily resolve and read a cockpit param that the mod's Lua devices publish. False without the cockpit API.
    bool read_named_param(void*& handle, const char* name, double& value)
    {
        if (handle == nullptr)
        {
            F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
            if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
                F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_parameter_value_to_number == nullptr)
            {
                return false;
            }
            handle = F117::cockpitAPI.getParamHandle(name);
        }
        if (handle == nullptr)
        {
            return false;
        }
        value = F117::cockpitAPI.getParamNumber(handle);
        return true;
    }

    // Start air for the air turbine starters from the APU or an external air cart. Crossbleed from a running
    // engine is handled by ENGINE_START::Starts. The mod's Systems/Electrical.lua owns both: F117_APU_STATE 2 is
    // the APU running at normal output pressure, and F117_GROUND_AIR 1 is an air cart connected.
    bool external_start_air()
    {
        static void* s_apuStateParam = nullptr;
        static void* s_groundAirParam = nullptr;
        double apu = 0.0;
        double cart = 0.0;
        const bool apuAir = read_named_param(s_apuStateParam, "F117_APU_STATE", apu) && apu > 1.5;
        const bool cartAir = read_named_param(s_groundAirParam, "F117_GROUND_AIR", cart) && cart > 0.5;
        return apuAir || cartAir;
    }

    void initialize_weapon_release_gate_param()
    {
        if (g_weaponReleaseGateParamInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_weaponReleaseGateParam = F117::cockpitAPI.getParamHandle(kWeaponReleaseGateParamName);
        g_weaponReleaseGateParamInitialized = (g_weaponReleaseGateParam != nullptr);
    }

    void publish_weapon_release_gate_param()
    {
        initialize_weapon_release_gate_param();
        if (g_weaponReleaseGateParam != nullptr)
        {
            F117::cockpitAPI.setParamNumber(g_weaponReleaseGateParam, F117::weapon_release_gate ? 1.0 : 0.0);
        }
    }

    void initialize_irads_lock_params()
    {
        if (g_iradsLockParamsInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_iradsLockValidParam = F117::cockpitAPI.getParamHandle(kIRADSLockValidParamName);
        g_iradsLockWorldXParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldXParamName);
        g_iradsLockWorldYParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldYParamName);
        g_iradsLockWorldZParam = F117::cockpitAPI.getParamHandle(kIRADSLockWorldZParamName);
        g_iradsLockRangeParam = F117::cockpitAPI.getParamHandle(kIRADSLockRangeParamName);
        g_iradsLockAzimuthParam = F117::cockpitAPI.getParamHandle(kIRADSLockAzimuthParamName);
        g_iradsLockElevationParam = F117::cockpitAPI.getParamHandle(kIRADSLockElevationParamName);
        g_iradsLockParamsInitialized =
            (g_iradsLockValidParam != nullptr) &&
            (g_iradsLockWorldXParam != nullptr) &&
            (g_iradsLockWorldYParam != nullptr) &&
            (g_iradsLockWorldZParam != nullptr) &&
            (g_iradsLockRangeParam != nullptr) &&
            (g_iradsLockAzimuthParam != nullptr) &&
            (g_iradsLockElevationParam != nullptr);
    }

    void initialize_release_cue_params()
    {
        if (g_releaseCueParamsInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_releaseCueValidParam = F117::cockpitAPI.getParamHandle(kReleaseCueValidParamName);
        g_releaseCueErrorParam = F117::cockpitAPI.getParamHandle(kReleaseCueErrorParamName);
        g_releaseCueTimeToGoParam = F117::cockpitAPI.getParamHandle(kReleaseCueTimeToGoParamName);
        g_releaseCueRangeParam = F117::cockpitAPI.getParamHandle(kReleaseCueRangeParamName);
        g_releaseCueInZoneParam = F117::cockpitAPI.getParamHandle(kReleaseCueInZoneParamName);
        g_releaseCueParamsInitialized =
            (g_releaseCueValidParam != nullptr) &&
            (g_releaseCueErrorParam != nullptr) &&
            (g_releaseCueTimeToGoParam != nullptr) &&
            (g_releaseCueRangeParam != nullptr) &&
            (g_releaseCueInZoneParam != nullptr);
    }

    void initialize_ccrp_profile_param()
    {
        if (g_ccrpProfileParamInitialized)
        {
            return;
        }

        F117::cockpitAPI.ed_param_api = ed_get_cockpit_param_api();
        if (F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_get_parameter_handle == nullptr ||
            F117::cockpitAPI.ed_param_api.pfn_ed_cockpit_update_parameter_with_number == nullptr)
        {
            return;
        }

        g_ccrpProfileParam = F117::cockpitAPI.getParamHandle(kCcrpProfileParamName);
        g_ccrpProfileParamInitialized = (g_ccrpProfileParam != nullptr);
    }

    int get_ccrp_profile_index()
    {
        initialize_ccrp_profile_param();
        if (!g_ccrpProfileParamInitialized)
        {
            return 0;
        }

        const double rawIndex = F117::cockpitAPI.getParamNumber(g_ccrpProfileParam);
        int profileIndex = std::isfinite(rawIndex) && rawIndex>=0.0 && rawIndex<kCcrpStoreCount ? (int)floor(rawIndex + 0.5) : 0;
        const int profileCount = kCcrpStoreCount;
        if (profileIndex < 0)
        {
            profileIndex = 0;
        }
        if (profileIndex >= profileCount)
        {
            profileIndex = profileCount - 1;
        }

        return profileIndex;
    }

    const CcrpStoreModel& get_ccrp_profile()
    {
        return kCcrpStoreModels[get_ccrp_profile_index()];
    }

    Vec3 get_world_wind_vector()
    {
        return wind_world_cs;
    }

    bool simulate_bomb_impact_point(const CcrpStoreModel& profile, const Vec3& releasePos, const Vec3& releaseVel, const Vec3& worldWind, double targetAltMeters, Vec3* outImpactPos, double* outImpactTime)
    {
        if (outImpactPos == nullptr || outImpactTime == nullptr || profile.massKg <= 0.0 || profile.referenceAreaM2 <= 0.0)
        {
            return false;
        }

        if (releasePos.y <= targetAltMeters)
        {
            return false;
        }

        const double altitudeReference = max(releasePos.y, 0.0);
        const double seaLevelDensity = max(F117::ambientDensity_KgPerM3, 0.05) * exp(altitudeReference / kAtmosphereScaleHeight_M);

        Vec3 pos = releasePos;
        Vec3 vel = releaseVel;
        Vec3 prevPos = pos;
        double elapsed = 0.0;

        while (elapsed < kReleaseIntegrationMaxSeconds)
        {
            prevPos = pos;

            Vec3 relVel{};
            relVel.x = vel.x - worldWind.x;
            relVel.y = vel.y - worldWind.y;
            relVel.z = vel.z - worldWind.z;

            const double relSpeed = sqrt((relVel.x * relVel.x) + (relVel.y * relVel.y) + (relVel.z * relVel.z));
            Vec3 acc{};
            acc.x = 0.0;
            acc.y = -kGravity_MPS2;
            acc.z = 0.0;

            if (relSpeed > 0.25)
            {
                const double rho = seaLevelDensity * exp(-max(pos.y, 0.0) / kAtmosphereScaleHeight_M);
                const double dragScale = 0.5 * rho * profile.dragCd * profile.referenceAreaM2 / profile.massKg;
                const double dragAccel = dragScale * relSpeed * relSpeed;
                const double invRelSpeed = 1.0 / relSpeed;
                acc.x -= dragAccel * relVel.x * invRelSpeed;
                acc.y -= dragAccel * relVel.y * invRelSpeed;
                acc.z -= dragAccel * relVel.z * invRelSpeed;
            }

            vel.x += acc.x * kReleaseIntegrationStepSeconds;
            vel.y += acc.y * kReleaseIntegrationStepSeconds;
            vel.z += acc.z * kReleaseIntegrationStepSeconds;

            pos.x += vel.x * kReleaseIntegrationStepSeconds;
            pos.y += vel.y * kReleaseIntegrationStepSeconds;
            pos.z += vel.z * kReleaseIntegrationStepSeconds;
            elapsed += kReleaseIntegrationStepSeconds;

            if (pos.y <= targetAltMeters)
            {
                const double denom = prevPos.y - pos.y;
                const double fraction = (fabs(denom) > 1.0e-6) ? ((prevPos.y - targetAltMeters) / denom) : 1.0;
                outImpactPos->x = prevPos.x + ((pos.x - prevPos.x) * fraction);
                outImpactPos->y = targetAltMeters;
                outImpactPos->z = prevPos.z + ((pos.z - prevPos.z) * fraction);
                *outImpactTime = max(0.0, elapsed - kReleaseIntegrationStepSeconds + (kReleaseIntegrationStepSeconds * fraction));
                return true;
            }
        }

        return false;
    }

    bool g_releaseAlignmentValid = false;
    double g_releaseMinRange = 0.0, g_releaseMaxRange = 0.0;

    // Provisional guided-store envelope around the ballistic prediction, not a
    // measured DCS DLZ. Keep the ballistic error/TTG as cues, not a pinpoint gate.
    bool guided_release_in_zone(StoreKind kind, double ballisticRange,
        double alongTrack, double crossTrack, double minimumWindow, bool* aligned)
    {
        if (!aligned) return false;
        *aligned = false;
        if (!store_falls_ballistic(kind) || kind == StoreKind::Unguided || !std::isfinite(ballisticRange) ||
            !std::isfinite(alongTrack) || !std::isfinite(crossTrack) ||
            !std::isfinite(minimumWindow) || ballisticRange <= 0.0 ||
            alongTrack <= 0.0 || crossTrack < 0.0 || minimumWindow < 0.0) return false;
        const bool jdam = kind == StoreKind::Jdam;
        const double lateralRatio = jdam ? 0.577350269 : 0.267949192; // 30 / 15 degrees
        *aligned = crossTrack <= max(50.0, alongTrack * lateralRatio);
        const double rangeMargin = max(minimumWindow, ballisticRange * (jdam ? 0.35 : 0.20));
        const double horizontalRange = sqrt(alongTrack * alongTrack + crossTrack * crossTrack);
        return *aligned && alongTrack >= max(1.0, ballisticRange - rangeMargin) &&
            horizontalRange <= ballisticRange + rangeMargin;
    }

    bool compute_release_solution(double* outErrorMeters, double* outTimeToGoSeconds, double* outRangeMeters, bool* outInZone)
    {
        if (outErrorMeters == nullptr || outTimeToGoSeconds == nullptr || outRangeMeters == nullptr || outInZone == nullptr)
        {
            return false;
        }

        g_releaseAlignmentValid = false;
        g_releaseMinRange = g_releaseMaxRange = 0.0;
        double targetWorld[3] = {0.0, 0.0, 0.0};
        if (!CockpitInterop::GetIRADSLockWorldPoint(targetWorld))
        {
            return false;
        }

        for (int i=0;i<3;++i) if (!std::isfinite(targetWorld[i])) return false;
        if (!std::isfinite(vx_world) || !std::isfinite(vy_world) || !std::isfinite(vz_world) ||
            !std::isfinite(position_world_cs.x) || !std::isfinite(position_world_cs.y) || !std::isfinite(position_world_cs.z)) return false;
        const CcrpStoreModel& profile = get_ccrp_profile();
        const Vec3 releaseVel{ vx_world, vy_world, vz_world };
        const double dx = targetWorld[0] - position_world_cs.x;
        const double dz = targetWorld[2] - position_world_cs.z;
        const double horizSpeed = sqrt((releaseVel.x * releaseVel.x) + (releaseVel.z * releaseVel.z));
        const double horizRange = sqrt((dx * dx) + (dz * dz));
        const double dropMeters = position_world_cs.y - targetWorld[1];

        if (horizSpeed < max(kReleaseCueMinSpeedMPS, profile.minSpeedMPS) || dropMeters < max(kReleaseCueMinDropMeters, profile.minDropMeters))
        {
            return false;
        }

        const double vx = releaseVel.x / horizSpeed;
        const double vz = releaseVel.z / horizSpeed;
        const double alongTrack = (dx * vx) + (dz * vz);
        if (alongTrack <= 0.0)
        {
            return false;
        }

        const double crossTrackSq = (horizRange * horizRange) - (alongTrack * alongTrack);
        const double crossTrack = sqrt((crossTrackSq > 0.0) ? crossTrackSq : 0.0);
        const Vec3 worldWind = get_world_wind_vector();
        const double airX=releaseVel.x-worldWind.x, airY=releaseVel.y-worldWind.y, airZ=releaseVel.z-worldWind.z;
        const double airSpeed=std::sqrt(airX*airX+airY*airY+airZ*airZ);
        const StoreKind kind = profile.kind;
        double impactAlongTrack = 0.0, errorMeters = 0.0, timeToGo = 0.0;
        if (store_falls_ballistic(kind))
        {
            Vec3 impactPos{};
            double impactTime = 0.0;
            if (!simulate_bomb_impact_point(profile, position_world_cs, releaseVel, worldWind, targetWorld[1], &impactPos, &impactTime))
            {
                return false;
            }
            F117::ccrp_time_of_flight = impactTime;
            const Vec3 impactDelta
            {
                impactPos.x - position_world_cs.x,
                impactPos.y - position_world_cs.y,
                impactPos.z - position_world_cs.z
            };
            impactAlongTrack = (impactDelta.x * vx) + (impactDelta.z * vz);
            errorMeters = alongTrack - impactAlongTrack;
            timeToGo = (errorMeters > 0.0) ? (errorMeters / max(horizSpeed, profile.minSpeedMPS)) : 0.0;
        }
        bool inZone = false;
        if (kind == StoreKind::Jdam || kind == StoreKind::Glide)
        {
            // Weapon-authored launch tables, interpolated in target-relative
            // height and true airspeed. No extrapolation outside the table domain.
            const bool tabled = kind == StoreKind::Jdam
                ? JdamEnvelope::lookup(profile.envelope,dropMeters,airSpeed,&g_releaseMinRange,&g_releaseMaxRange)
                : GlideEnvelope::lookup(profile.envelope,dropMeters,airSpeed,&g_releaseMinRange,&g_releaseMaxRange);
            if (!tabled) return false;
            g_releaseAlignmentValid = crossTrack<=max(50.0,alongTrack*0.577350269);
            inZone = g_releaseAlignmentValid && horizRange>=g_releaseMinRange && horizRange<=g_releaseMaxRange;
            if (kind == StoreKind::Glide)
            {
                // A glide weapon has no ballistic impact: cue the distance still to close
                // to the maximum range, and time the flight at a typical glide speed.
                errorMeters = max(0.0, horizRange - g_releaseMaxRange);
                timeToGo = errorMeters / max(horizSpeed, profile.minSpeedMPS);
                F117::ccrp_time_of_flight = horizRange / kGlideWeaponSpeedMPS;
            }
        }
        else if (kind == StoreKind::Unguided)
        {
            // True CCRP: the release fires as the predicted impact crosses the target.
            g_releaseAlignmentValid = crossTrack <= kUnguidedMaxCrossTrackMeters;
            inZone = g_releaseAlignmentValid && fabs(errorMeters) <= profile.releaseWindowMeters;
            g_releaseMinRange = max(1.0, impactAlongTrack - profile.releaseWindowMeters);
            g_releaseMaxRange = impactAlongTrack + profile.releaseWindowMeters;
        }
        else if (store_is_missile(kind))
        {
            // Powered missile: target inside the seeker cone around the flight path
            // and slant range inside the stock launch limits.
            const double slant = sqrt(horizRange * horizRange + dropMeters * dropMeters);
            const double speed = sqrt(releaseVel.x * releaseVel.x + releaseVel.y * releaseVel.y + releaseVel.z * releaseVel.z);
            const double cosine = (slant > 1.0 && speed > 1.0)
                ? (dx * releaseVel.x - dropMeters * releaseVel.y + dz * releaseVel.z) / (slant * speed) : -1.0;
            g_releaseAlignmentValid = cosine >= cos(profile.seekerHalfAngleRad);
            g_releaseMinRange = profile.minRangeMeters;
            g_releaseMaxRange = profile.maxRangeMeters;
            inZone = g_releaseAlignmentValid && slant >= g_releaseMinRange && slant <= g_releaseMaxRange;
            errorMeters = max(0.0, slant - g_releaseMaxRange);
            timeToGo = errorMeters / max(horizSpeed, profile.minSpeedMPS);
            F117::ccrp_time_of_flight = slant / kMissileAverageSpeedMPS;
        }
        else
        {
            inZone = guided_release_in_zone(kind, impactAlongTrack,
                alongTrack, crossTrack, profile.releaseWindowMeters, &g_releaseAlignmentValid);
            g_releaseMinRange=max(1.0,impactAlongTrack-max(profile.releaseWindowMeters,impactAlongTrack*0.20));
            g_releaseMaxRange=impactAlongTrack+max(profile.releaseWindowMeters,impactAlongTrack*0.20);
        }
        static bool logged = false, lastZone = false, lastAligned = false;
        static unsigned diagnosticFrames = 0;
        if (!logged || inZone != lastZone || g_releaseAlignmentValid != lastAligned ||
            (F117::weapon_release_pickle_held && ++diagnosticFrames >= 600))
        {
            char message[320];
            sprintf_s(message, "DELIVERY ZONE: profile=%d along=%.1f cross=%.1f ballistic=%.1f error=%.1f aligned=%d zone=%d min=%.1f max=%.1f drop=%.1f speed=%.1f",
                get_ccrp_profile_index(), alongTrack, crossTrack, impactAlongTrack, errorMeters,
                g_releaseAlignmentValid, inZone, g_releaseMinRange, g_releaseMaxRange, dropMeters, horizSpeed);
            CockpitInterop::Log_(message);
            logged = true; lastZone = inZone; lastAligned = g_releaseAlignmentValid; diagnosticFrames = 0;
        }

        *outErrorMeters = errorMeters;
        *outTimeToGoSeconds = timeToGo;
        *outRangeMeters = alongTrack;
        *outInZone = inZone;
        return true;
    }

    void publish_release_cue_params()
    {
        initialize_release_cue_params();
        if (!g_releaseCueParamsInitialized)
        {
            return;
        }

        double errorMeters = 0.0;
        double timeToGoSeconds = 0.0;
        double rangeMeters = 0.0;
        bool inZone = false;
        const bool valid = CockpitInterop::HasIRADSLock() &&
            compute_release_solution(&errorMeters, &timeToGoSeconds, &rangeMeters, &inZone);

        if (valid)
        {
            F117::cockpitAPI.setParamNumber(g_releaseCueValidParam, 1.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueErrorParam, errorMeters);
            F117::cockpitAPI.setParamNumber(g_releaseCueTimeToGoParam, timeToGoSeconds);
            F117::cockpitAPI.setParamNumber(g_releaseCueRangeParam, rangeMeters);
            F117::cockpitAPI.setParamNumber(g_releaseCueInZoneParam, inZone ? 1.0 : 0.0);
        }
        else
        {
            F117::cockpitAPI.setParamNumber(g_releaseCueValidParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueErrorParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueTimeToGoParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueRangeParam, 0.0);
            F117::cockpitAPI.setParamNumber(g_releaseCueInZoneParam, 0.0);
        }
    }

    void publish_irads_lock_params()
    {
        initialize_irads_lock_params();
        if (!g_iradsLockParamsInitialized)
        {
            return;
        }

        // Parameter payloads are published before their sequence/ack markers.
        // A pending ray is never a valid designation or release target.
        static void* request[7]={};
        static void* result[5]={};
        static const char* requestNames[]={"F117_TERRAIN_OX","F117_TERRAIN_OY","F117_TERRAIN_OZ","F117_TERRAIN_DX","F117_TERRAIN_DY","F117_TERRAIN_DZ","F117_TERRAIN_REQUEST"};
        static const char* resultNames[]={"F117_TERRAIN_X","F117_TERRAIN_Y","F117_TERRAIN_Z","F117_TERRAIN_HIT","F117_TERRAIN_ACK"};
        bool handles=true;
        for(int i=0;i<7;++i) { if(!request[i]) request[i]=F117::cockpitAPI.getParamHandle(requestNames[i]);handles=handles&&request[i]; }
        for(int i=0;i<5;++i) { if(!result[i]) result[i]=F117::cockpitAPI.getParamHandle(resultNames[i]);handles=handles&&result[i]; }
        if(handles) {
            unsigned sequence=0;double ray[6]={};
            bool pending=CockpitInterop::GetIRADSTerrainRequest(&sequence,ray);
            if(pending && F117::cockpitAPI.getParamNumber(result[4])==sequence) {
                double point[3];for(int i=0;i<3;++i) point[i]=F117::cockpitAPI.getParamNumber(result[i]);
                CockpitInterop::CompleteIRADSTerrainRequest(sequence,F117::cockpitAPI.getParamNumber(result[3])==1.0,point);
                pending=false;
            }
            if(pending) for(int i=0;i<6;++i) F117::cockpitAPI.setParamNumber(request[i],ray[i]);
            F117::cockpitAPI.setParamNumber(request[6],pending?sequence:0.0);
        }

        double world[3] = {0.0, 0.0, 0.0};
        double polar[3] = {0.0, 0.0, 0.0};
        const bool valid = CockpitInterop::HasIRADSLock() &&
            CockpitInterop::GetIRADSLockWorldPoint(world) &&
            CockpitInterop::GetIRADSLockPolar(polar);

        if (valid)
        {
            F117::cockpitAPI.setParamNumber(g_iradsLockValidParam, 1.0);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldXParam, world[0]);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldYParam, world[1]);
            F117::cockpitAPI.setParamNumber(g_iradsLockWorldZParam, world[2]);
            F117::cockpitAPI.setParamNumber(g_iradsLockRangeParam, polar[2]);
            F117::cockpitAPI.setParamNumber(g_iradsLockAzimuthParam, polar[0]);
            F117::cockpitAPI.setParamNumber(g_iradsLockElevationParam, polar[1]);
        }
        else
        {
            F117::cockpitAPI.setParamNumber(g_iradsLockValidParam, 0.0);
        }
    }

    bool weapon_bay_is_open()
    {
        return F117::misc_state >= kWeaponBayOpenThreshold;
    }

    bool weapon_bay_is_closed()
    {
        return F117::misc_state <= kWeaponBayClosedThreshold;
    }

    void command_weapon_bay_open()
    {
        F117::misc_cmd = 1.0f;
    }

    void command_weapon_bay_close()
    {
        F117::misc_cmd = 0.0f;
    }

    void reset_weapon_release_sequence()
    {
        F117::weapon_release_pickle_held = false;
        F117::weapon_release_gate = false;
        F117::weapon_release_gate_fired = false;
        F117::weapon_bay_owned_by_release = false;
        F117::weapon_bay_close_pending = false;
        F117::weapon_bay_close_delay = 0.0;
        F117::weapon_release_gate_hold = 0.0;
        publish_weapon_release_gate_param();
    }

    void handle_weapon_release_pressed()
    {
        F117::weapon_bay_close_pending = false;
        F117::weapon_bay_close_delay = 0.0;
        if (F117::weapon_release_pickle_held) return; // key repeat is not new consent
        F117::weapon_release_pickle_held = true;
        F117::weapon_release_gate_fired = false;
        F117::release_retry_delay = 0.0;

        // If the bay is already open, leave ownership as-is so a re-pickle during a
        // pending auto-close still closes the bay on the next release.
        if (!weapon_bay_is_open())
        {
            command_weapon_bay_open();
            F117::weapon_bay_owned_by_release = true;
        }

        publish_weapon_release_gate_param();
    }

    void handle_weapon_release_released()
    {
        F117::weapon_release_pickle_held = false;
        F117::weapon_release_gate_fired = false;

        if (F117::weapon_bay_owned_by_release)
        {
            if (weapon_bay_is_open())
            {
                F117::weapon_bay_close_pending = true;
                F117::weapon_bay_close_delay = kWeaponBayHoldAfterReleaseSeconds;
            }
            else
            {
                command_weapon_bay_close();
                F117::weapon_bay_close_pending = false;
                F117::weapon_bay_close_delay = 0.0;
            }
        }
        else
        {
            F117::weapon_bay_close_pending = false;
            F117::weapon_bay_close_delay = 0.0;
        }

        publish_weapon_release_gate_param();
    }

    // Laser arm + a valid powered designation automatically provides code-1688
    // illumination before and after release; Safe or designation clear stops it.
    void update_laser(double dt)
    {
        F117::laser_window = max(0.0, F117::laser_window - dt);
        F117::weapon_flight_time_remaining = max(0.0, F117::weapon_flight_time_remaining - dt);

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
            for (int i = 0; i < 3; ++i) publish_named_param(s_iradsLosParams[i], kIRADSLosNames[i], los[i]);
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
        F117::release_retry_delay = max(0.0, F117::release_retry_delay - dt);
        double errorMeters=0.0, timeToGoSeconds=0.0, rangeMeters=0.0;
        bool inZone=false;
        const bool hasSolution = CockpitInterop::HasIRADSLock() &&
            compute_release_solution(&errorMeters,&timeToGoSeconds,&rangeMeters,&inZone);
        static void* profileValidParam=nullptr;
        static void* stationParam=nullptr;
        if (!profileValidParam) profileValidParam=F117::cockpitAPI.getParamHandle("F117_STORE_VALID");
        if (!stationParam) stationParam=F117::cockpitAPI.getParamHandle("F117_RELEASE_STATION");
        const bool storeValid=profileValidParam && F117::cockpitAPI.getParamNumber(profileValidParam)>0.5;
        const double stationRaw=stationParam ? F117::cockpitAPI.getParamNumber(stationParam) : -1.0;
        const int station=std::isfinite(stationRaw) && stationRaw>=0.0 && stationRaw<=8.0 ? static_cast<int>(stationRaw) : -1;
        const bool efmLaunchAvailable=WeaponsInterop::IsLaunchAvailable();
        const bool laserRequired=store_needs_laser(get_ccrp_profile().kind);
        int status=9; // ready: hold consent to release at computed CCRP point
        if (!F117::master_armed) status=1;
        else if (F117::weight_on_wheels || F117::wow_from_draw_args) status=12;
        else if (!CockpitInterop::IsIRADSPowered()) status=2;
        else if (!CockpitInterop::HasIRADSLock()) status=3;
        else if (!storeValid || station<0 || station>8) status=4;
        else if (laserRequired && (!F117::laser_armed || !WeaponsInterop::IsLaserFiring())) status=5;
        else if (!weapon_bay_is_open()) status=6;
        else if (!hasSolution) status=7;
        else if (!g_releaseAlignmentValid) status=14;
        else if (!inZone) status=8;
        else if (!efmLaunchAvailable) status=13;
        F117::weapon_release_gate_hold=max(0.0,F117::weapon_release_gate_hold-dt);
        if (status==9 && F117::weapon_release_pickle_held && !F117::weapon_release_gate_fired && F117::release_retry_delay<=0.0)
        {
            double point[3]={};
            if (CockpitInterop::GetIRADSLockWorldPoint(point) && WeaponsInterop::LaunchAtPoint(point,station))
            {
                ++F117::release_sequence;
                F117::weapon_release_gate_fired=true;
                F117::weapon_release_gate_hold=kWeaponReleaseGateMinHoldSeconds;
                for (int i=0;i<3;++i) F117::release_target[i]=point[i];
                F117::weapon_flight_time_remaining=F117::ccrp_time_of_flight;
                F117::laser_window=F117::ccrp_time_of_flight+kLaserOverrunSeconds;
                CockpitInterop::Log_("DELIVERY: confirmed native launch; target snapshot saved");
            }
            else
            {
                status=11;
                F117::release_retry_delay=0.25; // failed native attempt does not consume consent
            }
        }
        if (F117::weapon_release_gate_fired) status=10;
        if (status!=F117::release_status)
        {
            char message[128];
            sprintf_s(message,"DELIVERY: status=%d station=%d profile=%d consent=%d",status,station,get_ccrp_profile_index(),F117::weapon_release_pickle_held);
            CockpitInterop::Log_(message);
            F117::release_status=status;
        }
        static void* statusParam=nullptr; static void* armParam=nullptr; static void* flightParam=nullptr;
        static void* minRangeParam=nullptr; static void* maxRangeParam=nullptr;
        publish_named_param(minRangeParam,"F117_RELEASE_MIN_RANGE",g_releaseMinRange);
        publish_named_param(maxRangeParam,"F117_RELEASE_MAX_RANGE",g_releaseMaxRange);
        static void* sequenceParam=nullptr;
        publish_named_param(sequenceParam,"F117_RELEASE_SEQUENCE",F117::release_sequence);
        publish_named_param(statusParam,"F117_RELEASE_STATUS",status);
        publish_named_param(armParam,"F117_MASTER_ARM",F117::master_armed?1.0:0.0);
        publish_named_param(flightParam,"F117_STORE_FLIGHT_TIME",F117::weapon_flight_time_remaining);
        F117::weapon_release_gate=F117::weapon_release_gate_hold>0.0;

        if (!F117::weapon_release_pickle_held &&
            F117::weapon_bay_owned_by_release &&
            weapon_bay_is_closed())
        {
            F117::weapon_bay_owned_by_release = false;
            F117::weapon_bay_close_pending = false;
            F117::weapon_bay_close_delay = 0.0;
        }

        if (!F117::weapon_release_pickle_held && F117::weapon_bay_close_pending)
        {
            F117::weapon_bay_close_delay = max(0.0, F117::weapon_bay_close_delay - dt);
            if (F117::weapon_bay_close_delay <= 0.0)
            {
                command_weapon_bay_close();
                F117::weapon_bay_close_pending = false;
            }
        }

        static void* s_efmLaunchActiveParam = nullptr;
        publish_named_param(s_efmLaunchActiveParam, kEfmLaunchActiveParamName, 1.0);
        publish_release_cue_params();
        publish_weapon_release_gate_param();
    }
    bool handle_engine_and_throttle_command(int command, float value)
    {
        using namespace F117::ENGINE_START;
        if (command == EnginesOff || command == LeftEngineOff || command == RightEngineOff)
        {
            if (command != RightEngineOff) g_engineStarts.request_stop(LEFT);
            if (command != LeftEngineOff) g_engineStarts.request_stop(RIGHT);
            F117::engineswitch = g_engineStarts.any_running();
            if (!F117::engineswitch) F117::throttleInput = 0.0;
            return true;
        }
        if (command == EnginesOn)
        {
            const double cores[] = {F117::engines[LEFT].N2, F117::engines[RIGHT].N2};
            g_engineStarts.request_both(cores, external_start_air());
            return true;
        }
        if (command == LeftEngineOn || command == RightEngineOn)
        {
            const int side = command == LeftEngineOn ? LEFT : RIGHT;
            g_engineStarts.request_start(side, F117::engines[side].N2, external_start_air());
            return true;
        }
        if (command == JoystickThrottle)
        {
            if (F117::engineswitch)
            {
                const double percent = limit(((-value + 1.0) * 0.5) * 100.0, 0.0, 100.0);
                if (g_autothrottle.accept_axis(percent)) F117::throttleInput = percent;
            }
            return true;
        }
        if (command != ThrottleIncrease && command != ThrottleDecrease) return false;
        if (F117::engineswitch && (command == ThrottleDecrease || F117::internal_fuel >= 5.0))
        {
            g_autothrottle.pilot_throttle_step();
            const double step = command == ThrottleIncrease ? kThrottleStep : -kThrottleStep;
            F117::throttleInput = limit(F117::throttleInput + step, 0.0, 100.0);
        }
        return true;
    }

    bool handle_actuator_toggle_command(int command)
    {
        switch (command)
        {
        case tailhook:
            if (F117::ACTUATORS::tailhook.position < kToggleLowThreshold)
            {
                F117::tailhook_command = 1.0;
            }
            else if (F117::ACTUATORS::tailhook.position > kToggleHighThreshold)
            {
                F117::tailhook_command = 0.0;
            }
            return true;
        case dragChute:
            if (F117::ACTUATORS::dragChute.position < kToggleLowThreshold)
            {
                F117::dragchute_command = 1.0;
            }
            else if (F117::ACTUATORS::dragChute.position > kToggleHighThreshold)
            {
                F117::dragchute_command = 0.0;
            }
            printf("Drag chute = %f \n", dragchute_command);
            return true;
        case geardown:
            set_gear_command(kGearCommandDown);
            return true;
        case gearup:
            set_gear_command(kGearCommandUp);
            return true;
        case geartoggle:
            if (F117::ACTUATORS::gear.position > kGearToggleThreshold)
            {
                set_gear_command(kGearCommandUp);
            }
            else if (F117::ACTUATORS::gear.position < kGearToggleThreshold)
            {
                set_gear_command(kGearCommandDown);
            }
            return true;
        case WheelBrakeOn:
            F117::rolling_friction = kWheelBrakeOnFriction;
            return true;
        case WheelBrakeOff:
            F117::rolling_friction = kWheelBrakeOffFriction;
            return true;
        case AirRefuel:
            F117::airRefuelDoorOpen = !F117::airRefuelDoorOpen;
            return true;
        case bombay:
            reset_weapon_release_sequence();
            if (misc_state < 0.5)
            {
                command_weapon_bay_open();
            }
            else if (misc_state > 0.5)
            {
                command_weapon_bay_close();
            }
            return true;
        case bombayOpen:
            reset_weapon_release_sequence();
            command_weapon_bay_open();
            return true;
        case bombayClose:
            reset_weapon_release_sequence();
            command_weapon_bay_close();
            return true;
        case pickleOn:
        case weaponReleaseHoldOn:
            handle_weapon_release_pressed();
            return true;
        case pickleOff:
        case weaponReleaseHoldOff:
            handle_weapon_release_released();
            return true;
        case MasterArmToggle:
            F117::master_armed = !F117::master_armed;
            reset_weapon_release_sequence(); // require fresh consent after changing arm state
            return true;
        case IRADSLaserArmToggle:
            F117::laser_armed = !F117::laser_armed;
            return true;
        case luaDamageHit:
            apply_lua_damage_hit();
            return true;
        }

        return false;
    }

    bool handle_autopilot_command(int command)
    {
        int* selected = nullptr;
        if (command == autopilot_alt) selected = &alt_hold;
        if (command == autopilot_horiz) selected = &horiz_hold;
        if (command == autopilot_alt_roll) selected = &altroll_hold;
        if (!selected && command != autopilot_reset) return false;

        const bool engage = selected && *selected == 0;
        alt_hold = horiz_hold = altroll_hold = 0;
        if (engage) *selected = 1;
        if (command == autopilot_reset)
        {
            g_autothrottle.disconnect_button();
            g_paars.disengage();
            g_paarsEngagedAutothrottle = false;
        }
        return true;
    }

    // Either engine running.
    bool engines_running()
    {
        return F117::engineswitch && (F117::internal_fuel >= 5.0);
    }

    bool handle_autothrottle_command(int command)
    {
        switch (command)
        {
        case AutothrottleEngage: // A/T engage switch
            g_paarsEngagedAutothrottle = false;
            if (g_autothrottle.engaged())
            {
                g_autothrottle.disengage();
            }
            else
            {
                g_autothrottle.engage(F117::calibratedAirspeed_KT, F117::throttleInput,
                    !F117::weight_on_wheels, engines_running());
            }
            return true;
        case AutothrottleDisconnect: // inboard throttle disengage button
            g_paarsEngagedAutothrottle = false;
            g_autothrottle.disconnect_button();
            return true;
        }

        return false;
    }

    // ARS deselected or not functional: a pilot-engaged A/T stays engaged (manual 4-6);
    // the A/T PAARS engaged to move the power levers is released.
    void release_paars()
    {
        g_paars.disengage();
        if (g_paarsEngagedAutothrottle)
        {
            g_autothrottle.disengage();
            g_paarsEngagedAutothrottle = false;
        }
    }

    bool handle_paars_command(int command)
    {
        if (command != PaarsToggle) // stick ARS switch
        {
            return false;
        }
        if (g_paars.engaged())
        {
            release_paars();
        }
        else if (g_paars.engage(F117::pitch_angle, F117::roll_angle, F117::weight_on_wheels, F117::airRefuelDoorOpen)
            && !g_autothrottle.engaged())
        {
            g_paarsEngagedAutothrottle = g_autothrottle.engage(F117::calibratedAirspeed_KT, F117::throttleInput,
                !F117::weight_on_wheels, engines_running());
        }
        return true;
    }

    void update_paars()
    {
        Paars::Command command{};
        const bool active = g_paars.update(F117::pitch_angle, F117::roll_angle,
            F117::rollRate_RPS * F117::radiansToDegrees, F117::pitchRate_RPS * F117::radiansToDegrees,
            F117::gearDown > 0.5, F117::weight_on_wheels, F117::airRefuelDoorOpen, &command);
        if (!active && g_paarsEngagedAutothrottle)
        {
            release_paars();
        }
        F117::FLIGHTCONTROLS::set_recovery_nz_command(active, command.nz);
        g_paarsRollRateDegS = active ? command.rollRateDegS : 0.0;

        double pulloutFt = 0.0;
        if (active)
        {
            const double speed = sqrt(vx_world * vx_world + vy_world * vy_world + vz_world * vz_world);
            const double flightPathDeg = speed > 1.0 ? asin(limit(vy_world / speed, -1.0, 1.0)) * F117::radiansToDegrees : 0.0;
            pulloutFt = Paars::displayed_pullout_altitude_ft(Paars::minimum_pullout_altitude_ft(
                F117::altitude_m * 3.28084, F117::totalVelocity_MPS, flightPathDeg, F117::roll_angle));
        }
        static void* s_activeParam = nullptr;
        static void* s_pulloutParam = nullptr;
        publish_named_param(s_activeParam, "F117_ARS_ACTIVE", active ? 1.0 : 0.0);
        publish_named_param(s_pulloutParam, "F117_ARS_MIN_ALT", pulloutFt);
    }

    void update_autothrottle(double dt)
    {
        F117::calibratedAirspeed_KT = Autothrottle::calibrated_airspeed_kt(
            F117::totalVelocity_MPS, F117::ambientTemperature_DegK, F117::ambientPressure_PA);

        // Air data for the cockpit displays. With wind set, DCS's own true airspeed reads ground speed and its
        // indicated airspeed reads high, so the HUD and gauges use the airspeed this model flies on instead.
        static void* s_airDataValidParam = nullptr;
        static void* s_casParam = nullptr;
        static void* s_tasParam = nullptr;
        const bool airDataValid = std::isfinite(F117::calibratedAirspeed_KT) && std::isfinite(F117::totalVelocity_MPS);
        publish_named_param(s_casParam, "F117_CAS_MPS", airDataValid ? F117::calibratedAirspeed_KT / 1.943844 : 0.0);
        publish_named_param(s_tasParam, "F117_TAS_MPS", airDataValid ? F117::totalVelocity_MPS : 0.0);
        publish_named_param(s_airDataValidParam, "F117_AIR_DATA_VALID", airDataValid ? 1.0 : 0.0);
        F117::throttleInput = g_autothrottle.update(dt, F117::calibratedAirspeed_KT, F117::throttleInput,
            !F117::weight_on_wheels, engines_running());

        static void* s_engagedParam = nullptr;
        static void* s_discParam = nullptr;
        static void* s_casCommandParam = nullptr;
        publish_named_param(s_engagedParam, "F117_AT_ENGAGED", g_autothrottle.engaged() ? 1.0 : 0.0);
        publish_named_param(s_discParam, "F117_AT_DISC", g_autothrottle.disconnect_light() ? 1.0 : 0.0);
        publish_named_param(s_casCommandParam, "F117_AT_CAS_COMMAND", g_autothrottle.target_cas_kt());
    }

void log_simulation_frame(double dt)
{
    static int simFrameCount = 0;
    simFrameCount++;
    initDamageLog();
    if (g_damageLog)
    {
        g_damageLogTimer += dt;
        if (simFrameCount <= 500 || g_damageLogTimer >= 2.0)
        {
            g_damageLogTimer = 0.0;
            double accelMagLog = sqrt(ax_world * ax_world + F117::ay_world * F117::ay_world + az_world * az_world);
            fprintf(g_damageLog,
                "[SIM] frame=%d invincible=%d | LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f | accel=%.1f events=%d WoW=%d\n",
                simFrameCount,
                (int)F117::param_class.invincible_value,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit,
                accelMagLog,
                (int)g_simEvents.size(),
                (int)F117::weight_on_wheels);
            fflush(g_damageLog);
        }
    }
}

void begin_simulation_frame()
{
    common_force = Vec3();
    common_moment = Vec3();
}

void update_derived_damage_values()
{
    g_damage.wingAsymmetry = g_damage.leftWing - g_damage.rightWing;
    g_damage.totalWingLoss = 1.0 - (g_damage.leftWing + g_damage.rightWing) / 2.0;
    g_damage.totalTailLoss = 1.0 - (g_damage.leftTail + g_damage.rightTail) / 2.0;
    g_damage.engineAsymmetry = g_damage.leftEngine - g_damage.rightEngine;
    const double structuralIntegrity = (g_damage.nose + g_damage.leftFuselage + g_damage.rightFuselage
        + g_damage.belly + g_damage.leftTailStructure + g_damage.rightTailStructure
        + g_damage.leftWingInner + g_damage.rightWingInner
        + g_damage.leftWingOuter + g_damage.rightWingOuter) / 10.0;
    g_damage.structuralDragFactor = 1.0 + kStructuralDragPenalty * (1.0 - structuralIntegrity);
}

void update_total_velocity_from_airmass()
{
    Vec3 airspeed;
    airspeed.x = velocity_world_cs.x - wind.x;
    airspeed.y = velocity_world_cs.y - wind.y;
    airspeed.z = velocity_world_cs.z - wind.z;

    F117::totalVelocity_MPS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);
    if (F117::totalVelocity_MPS < 0.003)
    {
        F117::totalVelocity_MPS = 0.003; // ~0.01 ft/s equivalent minimum
    }
}

void update_atmosphere_from_total_velocity(double* airData)
{
    F117::ATMOS::atmos(F117::ambientTemperature_DegK, F117::ambientDensity_KgPerM3,
        F117::totalVelocity_MPS, airData);
    F117::dynamicPressure_PA = airData[0];
    F117::mach = airData[1];
}

void update_autopilot_hold_state()
{
    if (g_damage.cockpit <= 0.7)
    {
        alt_hold = horiz_hold = altroll_hold = 0;
        return;
    }
    if (horiz_hold)
        pitchTrim = 2.0 * pitch_angle;
    else if (alt_hold || altroll_hold)
        pitchTrim = limit(vspeed, -10.0, 10.0);
    if (horiz_hold || altroll_hold) rollTrim = 0.1 * roll_angle;
}


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
            const double specific_fuel = 0.9928 + 0.0911 * F117::mach + 0.0025 * F117::engines[side].thrustFractionOfMax - 0.1221 * F117::altitude_m / 10000.0; // kg / kgf / hour
            flow_kg_s = (std::max)(kIdleFuelFlowKgS / 2.0, (std::max)(F117::engine_thrust_N[side], 0.0) / 9.81 * specific_fuel / 3600.0);
        }
        else if (start.lit())
        {
            flow_kg_s = kStartFuelFlowKgS;
        }
        F117::engine_fuel_flow_kg_s[side] = std::isfinite(flow_kg_s) ? flow_kg_s : 0.0;
    }
    // Unlimited fuel burns nothing, so the quantity and DCS's mass both stay put.
    if (!F117::param_class.fuelvalue || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(F117::internal_fuel) || F117::internal_fuel <= 0.0) return;
    const double requested = (F117::engine_fuel_flow_kg_s[0] + F117::engine_fuel_flow_kg_s[1]) * dt;
    const double consumed = (std::min)(F117::internal_fuel, requested);
    F117::internal_fuel -= consumed;
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

void update_flight_control_commands(double dt, double controlDegradation)
{
    aoa_filter = 1;

    F117::FLIGHTCONTROLS::update_pitch_mode_auto_inputs(F117::gearDown, F117::airRefuelDoorOpen);
    // Suspension contact only, with the gear down. The strut draw args in weight_on_wheels aren't used here.
    F117::FLIGHTCONTROLS::update_ground_contact(
        F117::gearDown > 0.5 && (F117::suspension_wow[0] || F117::suspension_wow[1] || F117::suspension_wow[2]), dt);

    // The pitch law's gains are in degrees of an elevon giving kPitchLawCmPerDeg of pitching moment. This turns
    // its command into the deflection that gives the same moment on this airframe at this AOA, as the roll law
    // does with its rolling moment.
    const double pitchLawElevonScale = limit(kPitchLawCmPerDeg / (std::max)(std::abs(F117::AERO::CmPerElevatorDeg), 1e-4), 1.0, 8.0);
    F117::elevator_DEG_commanded = -(F117::FLIGHTCONTROLS::fcs_pitch_controller(
        F117::FLIGHTCONTROLS::longStickInput * controlDegradation,
        0.0,
        F117::alpha_DEG,
        F117::pitchRate_RPS * F117::radiansToDegrees,
        (F117::az / 9.81),
        0.0,
        F117::dynamicPressure_PA,
        dt,
        F117::roll_angle,
        F117::pitch_angle,
        F117::rollRate_RPS * F117::radiansToDegrees,
        F117::totalVelocity_MPS,
        F117::mach,
        F117::thrust_N,
        F117::AERO::Cx_total)) * pitchLawElevonScale;
    // Pitch damper. The tailless airframe has little natural pitch damping (Cmq about -2.2). The manual describes
    // the flight controls as giving essentially dead-beat damping, so the elevons add more. Nose-up pitch rate gives trailing edge down.
    // In a steady pull the pitch law's integrator takes up whatever the damper adds, so the elevon there is set by
    // the airframe's own moments and the damper costs no lift.
    {
        const double speed = (std::max)(F117::totalVelocity_MPS, 30.0);
        const double addedCm = kPitchDamperCmq * F117::meanChord_M / (2.0 * speed) * F117::pitchRate_RPS;
        F117::elevator_DEG_commanded += addedCm / (std::min)(F117::AERO::CmPerElevatorDeg, -1e-4);
    }
    // Pitch axis: Loschke specifies -25° TEU (nose-up) to +37.5° TED (nose-down) for pitch.
    // Physical actuator stop is ±45° — pitch authority does not reach the stop.
    F117::elevator_DEG = limit(F117::elevator_DEG_commanded + F117::pitchTrim, -25.0, 37.5);

    // What the roll law needs to know about the airframe at this flight condition. It uses the inertia DCS is flying,
    // so changing moment_of_inertia in the mod's FM/config.lua doesn't change the loop's bandwidth.
    const double rollMomentPerSurfaceDeg = (std::max)(std::abs(F117::AERO::ClPerAileronDeg), 0.0002);
    const double rollInertia = inertia.x > 1.0 ? inertia.x : 40000.0;
    const double rollAccelPerSurfaceDeg = rollMomentPerSurfaceDeg * F117::dynamicPressure_PA * F117::wingArea_M2 * F117::wingSpan_M / rollInertia * F117::radiansToDegrees;
    const double surfacePerRollRate = std::abs(F117::AERO::Clp) * F117::wingSpan_M / (2.0 * (std::max)(F117::totalVelocity_MPS, 30.0)) / F117::radiansToDegrees / rollMomentPerSurfaceDeg;

    F117::aileron_DEG_commanded = F117::FLIGHTCONTROLS::fcs_roll_controller(
        F117::FLIGHTCONTROLS::latStickInput * controlDegradation,
        F117::FLIGHTCONTROLS::longStickForce,
        F117::ay / 9.81,
        F117::pedInput,
        F117::beta_DEG,
        F117::roll_angle,
        F117::rollRate_RPS * F117::radiansToDegrees,
        g_paarsRollRateDegS, // PAARS roll-rate command, added to the pilot's
        F117::dynamicPressure_PA,
        rollAccelPerSurfaceDeg,
        surfacePerRollRate,
        F117::gearDown > 0.5,
        F117::airRefuelDoorOpen,
        dt);

    // Real F-117 pitch/roll mixer: pitch has priority.
    // Each elevon physical stop is ±45°. Pitch occupies elevator_DEG of that travel.
    // Roll gets whatever remains: max roll deflection = 45° - |elevator_DEG|.
    // This means rolling pull-outs automatically have less roll authority at high pitch demand,
    // exactly as the real aircraft (and consistent with Loschke's mixer description).
    const double maxRollDEG = (std::max)(0.0, 45.0 - std::abs(F117::elevator_DEG));
    F117::aileron_DEG = limit(F117::aileron_DEG_commanded + F117::rollTrim, -maxRollDEG, maxRollDEG);

    F117::dragchute = F117::ACTUATORS::drive_drag_chute(
        F117::dragchute_command,
        dt,
        F117::totalVelocity_MPS,
        F117::weight_on_wheels);

    // Keep the high-beta/high-yaw-rate rudder limiter symmetric left vs right.
    const double betaMagnitude_DEG = std::abs(beta_DEG);
    const double yawRateMagnitude_DEG_s = std::abs(yawRate_RPS * radiansToDegrees);
    aos_filter = static_cast<float>(1.0 + (betaMagnitude_DEG * betaMagnitude_DEG) / 7500.0 * (1.0 + yawRateMagnitude_DEG_s / 90.0));

    F117::rudder_DEG_commanded = F117::FLIGHTCONTROLS::fcs_yaw_controller(
        F117::pedInput,
        0.0,
        F117::beta_DEG,
        F117::yawRate_RPS * F117::radiansToDegrees,
        // Body yaw rate of a steady level turn at this bank and pitch: g sin(bank) cos(pitch) / V.
        9.81 * std::sin(F117::roll_angle / F117::radiansToDegrees) * std::cos(F117::pitch_angle / F117::radiansToDegrees)
            / (std::max)(F117::totalVelocity_MPS, 30.0) * F117::radiansToDegrees,
        F117::rollRate_RPS * F117::radiansToDegrees,
        F117::pitchRate_RPS * F117::radiansToDegrees,
        F117::FLIGHTCONTROLS::alphaFiltered,
        F117::dynamicPressure_PA,
        F117::gearDown > 0.5,
        dt);
    F117::rudder_DEG = F117::yawTrim + F117::rudder_DEG_commanded;
    // 25 deg lets full pedal reach the 8 deg of steady sideslip on TO 1F-117A-1 page 6-3 at low speed.
    F117::rudder_DEG = limit(F117::rudder_DEG, -25.0 / aos_filter, 25.0 / aos_filter);
}

double update_airframe_actuators(double dt)
{
    F117::elev_pos = static_cast<float>(F117::ACTUATORS::drive(F117::ACTUATORS::elevator,
        F117::FLIGHTCONTROLS::longStickInput / aoa_filter + (F117::pitchTrim / 15),
        dt));

    const double tailIntegrity = (g_damage.leftTail + g_damage.rightTail) / 2.0;
    F117::rudder_pos = static_cast<float>(F117::ACTUATORS::drive(F117::ACTUATORS::rudder, F117::pedInput, dt) * tailIntegrity);

    const double noseWheelRangeTarget = F117::nwsAuthorityHeld ? F117::kNoseWheelYawHigh_RAD : F117::kNoseWheelYawLow_RAD;
    const double noseWheelRangeStep = F117::kNoseWheelYawRangeRate_RADPS * dt;
    F117::noseWheelYawRange_RAD += limit(noseWheelRangeTarget - F117::noseWheelYawRange_RAD, -noseWheelRangeStep, noseWheelRangeStep);

    F117::GearCommand = clamp_gear_command_for_weight_on_wheels(F117::GearCommand);
    F117::gearDown = F117::ACTUATORS::drive_gear(F117::GearCommand, dt, gear_retraction_blocked());

    F117::tailhook_pos = F117::ACTUATORS::drive(F117::ACTUATORS::tailhook, F117::tailhook_command, dt);
    F117::misc_state = static_cast<float>(F117::ACTUATORS::drive(F117::ACTUATORS::weaponBay, F117::misc_cmd, dt));

    return tailIntegrity;
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
        double oil = kIdleOilPsi * limit(n2 / idleN2, 0.0, 1.0); // driven by core rotation
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
            shownEgt = egt; // first frame of a session: hot starts begin at idle, cold ones at ambient
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
    F117::throttle_state = F117::ACTUATORS::drive(F117::ACTUATORS::throttle, F117::throttleInput * avgEngineIntegrity, dt);

    // Each start sequence owns its engine's N2 until the starter cuts out; then the engine model takes over.
    const double integrity[2] = { g_damage.leftEngine, g_damage.rightEngine };
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
            F117::engine_thrust_N[side] = F117::ENGINE::engine_dynamics(engine, F117::throttleInput * integrity[side],
                F117::mach, F117::altitude_m, dt, start.running() && canRun[side], F117::weight_on_wheels);
        }
        F117::thrust_N += F117::engine_thrust_N[side];
    }
    update_engine_indications(dt);
    publish_engine_start_params();

    const double wingIntegrity = 1.0 - g_damage.totalWingLoss;
    const double elevonIntegrity = (g_damage.leftOuterElevon + g_damage.rightOuterElevon
        + g_damage.leftInnerElevon + g_damage.rightInnerElevon) / 4.0;
    F117::aileron_PCT = (F117::aileron_DEG * wingIntegrity * elevonIntegrity) / kAileronFullScale_DEG;
    F117::elevator_PCT = (F117::elevator_DEG * (1.0 - g_damage.totalTailLoss) * elevonIntegrity) / 25.0;

    const double tailAsymmetry = g_damage.leftTail - g_damage.rightTail;
    F117::rudder_PCT = (F117::rudder_DEG * tailIntegrity) / 30.0 + tailAsymmetry * 0.1;
    F117::FLIGHTCONTROLS::update_yaw_debug_snapshot(
        F117::beta_DEG,
        F117::yawRate_RPS,
        F117::rudder_DEG_commanded,
        F117::rudder_DEG,
        F117::rudder_PCT);
}


// Fraction of induced drag remaining near the ground (Wieselberger): 0 on the ground, 1 in free air.
double induced_drag_ground_effect()
{
    if (!F117::surface_height_valid || !F117::altitude_valid) return 1.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    const double ratio = 16.0 * clearance / F117::wingSpan_M;
    return ratio * ratio / (1.0 + ratio * ratio);
}

// Fraction of extra lift near the ground, at the same AOA. TO 1F-117A-1 (Appendix A3 introduction) says ground effect
// and the second engine's thrust together make liftoff AOA about 2 deg less than the 8.5 deg reference. 20% at the
// height of the aircraft origin on its wheels (2.12 m) gives that at the 46,000 lb, 194 KCAS takeoff example.
// The gain halves for every 0.15 span of height. That is a choice: the manual gives one point, and published data
// for low aspect ratio delta wings varies widely.
double lift_ground_effect_gain()
{
    if (!F117::surface_height_valid || !F117::altitude_valid) return 0.0;
    const double clearance = (std::max)(0.0, F117::altitude_m - F117::surface_height_m);
    constexpr double kGainOnWheels = 0.20;
    constexpr double kHeightOnWheels_M = 2.12;
    constexpr double kHalvingSpans = 0.15;
    return kGainOnWheels * exp2(-(clearance - kHeightOnWheels_M) / (kHalvingSpans * F117::wingSpan_M));
}

void update_aerodynamic_coefficients()
{
    // On wing area. Set with the brakes by the landing distances in TO 1F-117A-1 (Figure A8-2).
    const double CDchute = 0.20 * F117::ACTUATORS::dragChute.position;
    const double Cxchute = CDchute * cos(F117::pi / 180.0);

    const double CDbay = 0.015 * F117::ACTUATORS::weaponBay.position;
    const double Czbay = -(CDbay * sin(F117::pi / 180.0));
    const double Cxbay = CDbay * cos(F117::pi / 180.0);

    // On wing area. TO 1F-117A-1 Figure 3-5 gives best glides of about 8.5 gear up and 5.9 gear down, so lowering
    // the gear adds about 44% to the drag. On this airframe's drag that is about 0.016.
    const double CDGear = 0.016 * F117::gearDown;
    const double CzGear = -(CDGear * sin(F117::pi / 180.0));
    const double CxGear = CDGear * cos(F117::pi / 180.0);

    // Control surfaces as the damage model leaves them.
    F117::AERO::Input in;
    in.alpha_DEG = F117::alpha_DEG;
    in.beta_DEG = F117::beta_DEG;
    in.speed_MPS = F117::totalVelocity_MPS;
    in.rollRate_RPS = F117::rollRate_RPS;
    in.pitchRate_RPS = F117::pitchRate_RPS;
    in.yawRate_RPS = F117::yawRate_RPS;
    in.elevator_DEG = F117::elevator_PCT * 25.0;
    in.aileron_DEG = F117::aileron_PCT * kAileronFullScale_DEG;
    in.rudder_DEG = F117::rudder_PCT * 30.0;
    in.cgX_M = center_of_gravity.x;
    in.cgY_M = center_of_gravity.y;
    const F117::AERO::Output aero = F117::AERO::evaluate(in);

    const double alpha1_DEG_Limited = limit(F117::alpha_DEG, -90.0, 90.0);

    // Gear down, TO 1F-117A-1 Figure 2-2 needs more AOA at high lift than the gear-up line gives.
    const double gearDownLiftLoss = 0.0075 * F117::gearDown * (std::max)(alpha1_DEG_Limited - 6.0, 0.0);
    // Cz is negative for upward lift.
    const double Cz = -aero.CN + gearDownLiftLoss;
    // Ground effect adds lift at right angles to the airflow, so the drag terms below still use the free-air Cz.
    const double CzGroundEffect = lift_ground_effect_gain() * Cz;

    // Cd0 and the wave drag below Mlimit are fitted to TO 1F-117A-1 by tools/fm_bench/fit_performance.py in the
    // mod repository. Normal force acts along the body axis here, so tilting it with AOA supplies most of the
    // induced drag the data shows, as on a sharp-edged wing. Cz is negative for upward lift, so the lift terms are
    // the remainder: less drag than the tilt alone at low lift and more at high lift.
    const double Cd0 = 0.017152 + 0.038229 * Cz + 0.012645 * Cz * Cz;

    // Ground effect relieves part of that tilted normal force.
    const double phi_GE = induced_drag_ground_effect();
    // The second term cancels the drag the extra ground-effect lift would gain from being tilted with the body axis.
    const double Cd_induced = ((1.0 - phi_GE) * Cz + CzGroundEffect) * tan(limit(alpha1_DEG_Limited, -89.0, 89.0) * F117::pi / 180.0);

    double Cd_wave = 0.0;
    const double Mdiv = 0.85;
    // TO 1F-117A-1 limits the aircraft to Mach 0.90 and has no data above it. Published figures put the
    // maximum level speed between Mach 0.92 and 0.95; this barrier sets it near 0.92 at its best altitude.
    const double Mlimit = 0.91;

    if (F117::mach > Mdiv)
    {
        if (F117::mach < 1.05)
        {
            const double div_factor = (F117::mach - Mdiv) / (1.05 - Mdiv);
            Cd_wave = 0.007159 * div_factor * div_factor;

            if (F117::mach > Mlimit)
            {
                const double barrier_factor = (F117::mach - Mlimit) / (1.05 - Mlimit);
                Cd_wave += 0.096098 * barrier_factor * barrier_factor;
            }
        }
        else
        {
            Cd_wave = 0.12;
        }
    }

    F117::AERO::Cx_total = Cd0 + Cd_induced + Cd_wave + CxGear + Cxchute + Cxbay;
    F117::AERO::Cz_total = Cz + CzGroundEffect + CzGear + Czbay;
    F117::AERO::Cm_total = aero.Cm;
    F117::AERO::Cy_total = aero.CY;
    F117::AERO::Cn_total = aero.Cn;
    F117::AERO::Cl_total = aero.Cl;
    F117::AERO::ClPerAileronDeg = aero.ClPerAileronDeg;
    F117::AERO::CmPerElevatorDeg = aero.CmPerElevatorDeg;
    F117::AERO::CnPerRudderDeg = aero.CnPerRudderDeg;
    F117::AERO::Clp = aero.Clp;

    F117::FLIGHTCONTROLS::update_aero_debug_snapshot(
        aero.CyPerRudderDeg,
        aero.CnPerRudderDeg,
        aero.ClPerRudderDeg,
        aero.CnPerBetaDeg,
        aero.ClPerBetaDeg,
        aero.CY,
        aero.Cn,
        aero.Cl);
}

void apply_aerodynamic_forces_and_thrust()
{
    const double qS = F117::wingArea_M2 * F117::dynamicPressure_PA; // N

    Vec3 cy_force(0.0, 0.0, F117::AERO::Cy_total * qS);
    add_local_force(cy_force);

    Vec3 cx_force(-F117::AERO::Cx_total * qS * g_damage.structuralDragFactor, 0, 0);
    add_local_force(cx_force);

    const double liftDamageFactor = 1.0 - g_damage.totalWingLoss;
    Vec3 cz_force(0.0, -F117::AERO::Cz_total * qS * liftDamageFactor, 0.0);
    add_local_force(cz_force);

    const double qS_b = qS * F117::wingSpan_M; // N·m
    double rollMoment = F117::AERO::Cl_total * qS_b;
    rollMoment += g_damage.wingAsymmetry * qS_b * 0.05;


    const double pitchDamageFactor = 1.0 - g_damage.totalTailLoss * 0.5;
    const double noseDownBias = -g_damage.totalWingLoss * qS_b * 0.08;
    const double pitchMoment = F117::AERO::Cm_total * qS * F117::meanChord_M * pitchDamageFactor + noseDownBias;

    const double qS_span = qS_b; // N·m (same as qS_b for yaw)
    double yawMoment = -F117::AERO::Cn_total * qS_span;
    const double tailAsymDamage = g_damage.leftTail - g_damage.rightTail;
    yawMoment += tailAsymDamage * qS_span * 0.03;
    add_local_moment({rollMoment, yawMoment, pitchMoment});

    const double thrust_cant_up_deg = 0.1;
    const double thrust_toe_out_deg = 10;
    const double cant_up_rad = thrust_cant_up_deg * F117::pi / 180.0;
    const double toe_out_rad = thrust_toe_out_deg * F117::pi / 180.0;

    const double leftThrust = F117::engine_thrust_N[F117::ENGINE_START::LEFT] * g_damage.leftEngine;
    const double rightThrust = F117::engine_thrust_N[F117::ENGINE_START::RIGHT] * g_damage.rightEngine;

    const double thrust_x_L = leftThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_L = leftThrust * sin(cant_up_rad);
    const double thrust_z_L = leftThrust * cos(cant_up_rad) * sin(toe_out_rad);

    const double thrust_x_R = rightThrust * cos(cant_up_rad) * cos(toe_out_rad);
    const double thrust_y_R = rightThrust * sin(cant_up_rad);
    const double thrust_z_R = rightThrust * cos(cant_up_rad) * sin(toe_out_rad);

    Vec3 thrust_force_L(thrust_x_L, thrust_y_L, thrust_z_L);
    add_local_force(thrust_force_L);

    Vec3 thrust_force_R(thrust_x_R, thrust_y_R, -thrust_z_R);
    add_local_force(thrust_force_R);
}

}

// Main FM update called once per DCS simulation step.
// `dt` is the current frame step in seconds.
static void simulate_aircraft_frame(double dt)
{

	CockpitInterop::ForceIRADSIndicatorActive();
	CockpitInterop::UpdateIRADSSensorFrame(dt,
		position_world_cs.x, position_world_cs.y, position_world_cs.z,
		F117::aircraftQuaternionX, F117::aircraftQuaternionY,
		F117::aircraftQuaternionZ, F117::aircraftQuaternionW);
	publish_irads_lock_params();

log_simulation_frame(dt);

// Clear the per-frame force and moment accumulators.
begin_simulation_frame();
update_derived_damage_values();

// Compute air-relative speed using the current wind estimate.
update_total_velocity_from_airmass();

// Update Mach and dynamic pressure from the atmosphere model.
double airData[2]{};
update_atmosphere_from_total_velocity(airData);

//-----CONTROL DYNAMICS------------------------
//
// While PAARS flies the recovery the autopilot holds stay selected but do not act.
if (!g_paars.engaged())
{
    update_autopilot_hold_state();
}
update_autothrottle(dt);
update_paars();

// Fuel system
update_fuel_system(dt);

// Cockpit damage reduces effective control input below 0.7 integrity.
double controlDegradation = get_control_degradation();

// Run the tuned pitch/roll/yaw control laws and clamp the commanded surfaces.
update_flight_control_commands(dt, controlDegradation);

double tailIntegrity = update_airframe_actuators(dt);
update_weapon_release_sequence(dt);
update_propulsion_and_control_effectiveness(dt, tailIntegrity);
	// Engine and yaw debug logging
		{
			static std::ofstream engineDebugLog;
			static int engineLogCounter = 0;
			static bool engineLogInitialized = false;

			if (!engineLogInitialized)
			{
				const std::string savedGamesLogPath =
					build_saved_games_dcs_path("DCS", "Logs\\F117_Engine_Debug.csv");
				if (!savedGamesLogPath.empty())
				{
					engineDebugLog.open(savedGamesLogPath);
				}
				if (!engineDebugLog.is_open())
				{
					const std::string openBetaLogPath =
						build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Engine_Debug.csv");
					if (!openBetaLogPath.empty())
					{
						engineDebugLog.open(openBetaLogPath);
					}
				}
				if (!engineDebugLog.is_open())
				{
					engineDebugLog.open("F117_Engine_Debug.csv");
				}
				if (engineDebugLog.is_open())
				{
					engineDebugLog << "engine1_thrust_N,engine2_thrust_N,total_thrust_N,throttle_pct,N2_rpm_pct,mach,velocity_mps,altitude_m,"
					               << "pedInput,rudder_DEG_cmd,rudder_DEG,rudder_PCT,rudder_pos,beta_DEG,yawRate_RPS,yawTrim,aileron_DEG,aileron_PCT\n";
				}
				engineLogInitialized = true;
			}

			engineLogCounter++;
			if (engineDebugLog.is_open() && (engineLogCounter % 100 == 0))
			{
				engineDebugLog << F117::engine_thrust_N[0] << ","
				               << F117::engine_thrust_N[1] << ","
				               << F117::thrust_N << ","
				               << F117::throttleInput << ","
				               << 0.5 * (F117::engines[0].N2 + F117::engines[1].N2) << ","
				               << F117::mach << ","
				               << F117::totalVelocity_MPS << ","
				               << F117::altitude_m << ","
				               << F117::pedInput << ","
				               << F117::rudder_DEG_commanded << ","
				               << F117::rudder_DEG << ","
				               << F117::rudder_PCT << ","
				               << F117::rudder_pos << ","
				               << F117::beta_DEG << ","
				               << F117::yawRate_RPS << ","
				               << F117::yawTrim << ","
				               << F117::aileron_DEG << ","
				               << F117::aileron_PCT << "\n";
				engineDebugLog.flush();
			}
		}


// Aerodynamic coefficient update and force accumulation
update_aerodynamic_coefficients();
apply_aerodynamic_forces_and_thrust();
		// Mark subsystem initialization complete after the first successful simulation frame.
		F117::simInitialized = F117::ACTUATORS::initialized = F117::FLIGHTCONTROLS::simInitialized = true;

		update_weight_on_wheels();

}

void ed_fm_simulate(double dt)
{
    DeltaTime = dt;
    simulate_aircraft_frame(dt);
}

void ed_fm_set_surface(double h, double h_obj, unsigned /*surface_type*/,
                       double /*normal_x*/, double /*normal_y*/, double /*normal_z*/)
{
    F117::surface_height_valid = std::isfinite(h);
    if (F117::surface_height_valid)
        F117::surface_height_m = std::isfinite(h_obj) ? (std::max)(h, h_obj) : h;
}

void ed_fm_set_atmosphere(double altitude, double temperature, double /*soundSpeed*/,
    double density, double pressure, double windX, double windY, double windZ)
{
    // Air data and ballistic wind are separate from the body-frame wind callback.
    wind_world_cs = {windX, windY, windZ};
    F117::altitude_valid = std::isfinite(altitude);
    F117::altitude_m = altitude;
    F117::ambientPressure_PA = pressure;
    F117::ambientDensity_KgPerM3 = density;
    F117::ambientTemperature_DegK = temperature;
}

void ed_fm_set_current_mass_state(double massKg, double cgX, double cgY, double cgZ,
    double rollInertia, double yawInertia, double pitchInertia)
{
    center_of_gravity = {cgX, cgY, cgZ};
    inertia = {rollInertia, yawInertia, pitchInertia};
    // Preserve the controller's calibrated mass-to-load scale during this source replacement.
    constexpr double controllerWeightPerKg = 9.98665002864;
    F117::weight_N = controllerWeightPerKg * massKg;
}
void ed_fm_set_current_state(double accelX, double accelY, double accelZ,
    double speedX, double speedY, double speedZ, double posX, double posY, double posZ,
    double /*angularAccelX*/, double /*angularAccelY*/, double /*angularAccelZ*/,
    double /*rateX*/, double /*rateY*/, double /*rateZ*/,
    double qx, double qy, double qz, double qw)
{
    position_world_cs = {posX, posY, posZ};
    vx_world = speedX; vy_world = speedY; vz_world = speedZ;
    ax_world = accelX; F117::ay_world = accelY; az_world = accelZ;
    F117::aircraftQuaternionX = qx; F117::aircraftQuaternionY = qy;
    F117::aircraftQuaternionZ = qz; F117::aircraftQuaternionW = qw;
    // Existing hold-law vertical-speed reference, in m/s.
    F117::vspeed = speedY - 3.25;
}

double ax_body{};
double ay_body{};
double az_body{};
double vx_body{};
double vy_body{};
double vz_body{};

void ed_fm_set_current_state_body_axis(double accelX, double accelY, double accelZ,
    double speedX, double speedY, double speedZ, double windX, double windY, double windZ,
    double /*angularAccelX*/, double /*angularAccelY*/, double /*angularAccelZ*/,
    double rateX, double rateY, double rateZ, double /*heading*/, double pitchRad, double rollRad,
    double attackRad, double slipRad)
{
    // ED uses X forward, Y up, Z right. The control laws use positive yaw to the right.
    wind = {windX, windY, windZ};
    velocity_world_cs = {speedX, speedY, speedZ}; // Historical name: this snapshot is body-frame.
    vx_body = speedX; vy_body = speedY; vz_body = speedZ;
    ax_body = accelX; ay_body = accelY; az_body = accelZ;
    F117::rollRate_RPS = rateX; F117::pitchRate_RPS = rateZ; F117::yawRate_RPS = -rateY;
    F117::pitch_angle = radiansToDegrees * pitchRad;
    F117::roll_angle = radiansToDegrees * rollRad;
    F117::alpha_DEG = radiansToDegrees * attackRad;
    F117::beta_DEG = radiansToDegrees * slipRad;
    // Retain the load-axis names used by the controller interface.
    F117::az = accelY; F117::ay = accelZ;
}


void ed_fm_set_command(int command, float value)
{
    // Route a command to its first owner. IRADS has priority over flight controls.
    using AxisHandler = bool (*)(int, float);
    const AxisHandler axes[] = {CockpitInterop::HandleIRADSCommand, handle_roll_command,
        handle_pitch_command, handle_yaw_command, handle_engine_and_throttle_command};
    for (const auto handle : axes) if (handle(command, value)) return;
    using SwitchHandler = bool (*)(int);
    const SwitchHandler switches[] = {handle_actuator_toggle_command, handle_autopilot_command,
        handle_autothrottle_command, handle_paars_command};
    for (const auto handle : switches) if (handle(command)) return;
    if (command == resetTrim) pitchTrim = rollTrim = yawTrim = 0.0;
}


bool ed_fm_change_mass(double& delta_mass, double& delta_mass_pos_x,
    double& delta_mass_pos_y, double& delta_mass_pos_z,
    double& delta_mass_moment_of_inertia_x, double& delta_mass_moment_of_inertia_y,
    double& delta_mass_moment_of_inertia_z)
{
    // DCS repeatedly drains this callback. Acknowledge each fuel delta exactly once.
    const double pendingKg = F117::internal_fuel - F117::fuel_mass_in_dcs;
    const bool report = std::isfinite(pendingKg) && std::abs(pendingKg) >= 0.01;
    delta_mass = report ? pendingKg : 0.0;
    delta_mass_pos_x = 0.0; delta_mass_pos_z = 0.0;
    delta_mass_pos_y = report ? 0.1 : 0.0; // Empty-aircraft CG in FM/config.lua.
    delta_mass_moment_of_inertia_x = 0.0;
    delta_mass_moment_of_inertia_y = 0.0;
    delta_mass_moment_of_inertia_z = 0.0;
    if (report) F117::fuel_mass_in_dcs = F117::internal_fuel;
    return report;
}

void ed_fm_set_internal_fuel(double kilograms)
{
    F117::internal_fuel = kilograms;
}
double ed_fm_get_internal_fuel()
{
    return F117::internal_fuel; // The F-117 carries no external fuel tanks.
}
void ed_fm_set_external_fuel(int /*station*/, double /*kilograms*/,
    double /*x*/, double /*y*/, double /*z*/)
{
    // No supported loadout has an external tank; DCS owns all weapon masses.
}
double ed_fm_get_external_fuel ()
{
    return 0.0; // No external tank stations.
}

void ed_fm_refueling_add_fuel(double kilograms)
{
    F117::internal_fuel = F117::internal_fuel + kilograms;
}


namespace
{


    void sync_gear_draw_args(float* drawargs)
    {
        if (!F117::simInitialized)
        {
            for (const int argument : {kGearArgNose, kGearArgLeft, kGearArgRight})
                drawargs[argument] = static_cast<float>(F117::ACTUATORS::gear.position);
            return;
        }
        F117::ACTUATORS::gear.position = drawargs[kGearArgRight];
    }

    void update_weight_on_wheels_from_draw_args(float* drawargs)
    {
        const float contact = drawargs[kWowArgNose] + drawargs[kWowArgLeft] + drawargs[kWowArgRight];
        F117::wow_from_draw_args = contact > kWeightOnWheelsDrawArgThreshold;
        if (!F117::wow_from_draw_args) return;
        F117::weight_on_wheels = F117::wow_from_draw_args;
        F117::GearCommand = kGearCommandDown;
    }

    void update_control_surface_draw_args(float* drawargs)
    {
        drawargs[kLeftAileronArg] = static_cast<float>(limit((-aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75));
        drawargs[kRightAileronArg] = static_cast<float>(limit((aileron_PCT + (rollTrim / 10) / (F117::mach + 1)), -0.75, 0.75));
        drawargs[kLeftElevatorArg] = static_cast<float>(limit(-elev_pos / (mach + 1), -0.6, 0.6));
        drawargs[kRightElevatorArg] = static_cast<float>(limit(-elev_pos / (mach + 1), -0.6, 0.6));
        drawargs[kLeftRudderArg] = static_cast<float>(limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75));
        drawargs[kRightRudderArg] = static_cast<float>(limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75));
    }

    void update_misc_draw_args(float* drawargs, size_t size)
    {
        drawargs[kDragChuteArg] = static_cast<float>(ACTUATORS::dragChute.position);
        drawargs[kTailhookArg] = static_cast<float>(ACTUATORS::tailhook.position);
        drawargs[kWeaponBayArg] = static_cast<float>(limit(F117::misc_state, 0.0, 1.0));

        if (size > kMirroredGearDrawArgRequiredSize)
        {
            drawargs[kMirroredGearArgNose] = drawargs[kGearArgNose];
            drawargs[kMirroredGearArgLeft] = drawargs[kGearArgLeft];
            drawargs[kMirroredGearArgRight] = drawargs[kGearArgRight];
        }
    }



    void update_fc3_cockpit_draw_args(float* drawargs)
    {
        drawargs[kFc3CockpitRollArg] = static_cast<float>(limit(F117::FLIGHTCONTROLS::latStickInput, -1.0, 1.0));
        drawargs[kFc3CockpitPitchArg] = static_cast<float>(limit(-F117::FLIGHTCONTROLS::longStickInput, -1.0, 1.0));
        drawargs[kFc3CockpitThrottleArg] = static_cast<float>(limit(F117::throttleInput, -1.0, 1.0));
        drawargs[kFc3CockpitRudderArg] = static_cast<float>(limit(F117::pedInput, -1.0, 1.0));
        drawargs[kGearArgNose] = static_cast<float>(F117::GearCommand);
        drawargs[kGearArgLeft] = static_cast<float>(F117::GearCommand);
        drawargs[kGearArgRight] = static_cast<float>(F117::GearCommand);
    }
}


void ed_fm_set_draw_args_v2(float* drawargs, size_t size) // Sync external-model draw args between DCS and the FM state.
{
    // DCS owns damage visibility arguments. FM damage arrives via ed_fm_on_damage.
    sync_gear_draw_args(drawargs);
    update_weight_on_wheels_from_draw_args(drawargs);
    update_control_surface_draw_args(drawargs);
    update_misc_draw_args(drawargs, size);
}


// FC3 cockpit draw-arg callback used for stick, rudder pedal, throttle, and gear-handle animation.

void ed_fm_set_fc3_cockpit_draw_args_v2(float* arguments, size_t /*count*/)
{
    update_fc3_cockpit_draw_args(arguments);
}

void ed_fm_configure(const char * cfg_path)
{
	// Reserved DCS FM configuration hook; unused by this module.
}


namespace
{
    bool try_get_suspension_param(unsigned index, double& value)
    {
        const struct { unsigned id; double reading; } suspension[] = {
            {ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT, 0.0},
            {ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
            {ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT, F117::rolling_friction},
            {ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE, 0.0},
            {ED_FM_SUSPENSION_0_WHEEL_YAW, F117::rudder_pos * F117::noseWheelYawRange_RAD},
            {ED_FM_ANTI_SKID_ENABLE, 1.0},
            {ED_FM_SUSPENSION_0_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_1_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_2_GEAR_POST_STATE, F117::ACTUATORS::gear.position},
            {ED_FM_SUSPENSION_0_DOWN_LOCK, F117::ACTUATORS::gear.position},
            {ED_FM_FC3_GEAR_HANDLE_POS, F117::GearCommand},
        };
        for (const auto& parameter : suspension)
            if (parameter.id == index) { value = parameter.reading; return true; }
        return false;
    }

    bool try_get_engine_param(unsigned index, double& value)
    {
        if (index > ED_FM_END_ENGINE_BLOCK)
        {
            return false;
        }
        switch (index)
        {
        case ED_FM_FC3_THROTTLE_LEFT:
        case ED_FM_FC3_THROTTLE_RIGHT:
            value = limit(F117::throttleInput / 100.0, 0.0, 1.0);
            return true;
        }

        // Each engine has a block of parameters. Block 0 is the APU, which isn't reported; 1 is left, 2 is right.
        const unsigned blockSize = ED_FM_ENGINE_1_RPM - ED_FM_ENGINE_0_RPM;
        const unsigned block = index / blockSize;
        const unsigned field = index - block * blockSize;
        if (block == 0)
        {
            switch (field)
            {
            case ED_FM_ENGINE_0_RPM:
            case ED_FM_ENGINE_0_RELATED_RPM:
            case ED_FM_ENGINE_0_THRUST:
            case ED_FM_ENGINE_0_RELATED_THRUST:
                value = 0.0;
                return true;
            }
            return false;
        }
        if (block > 2)
        {
            return false;
        }
        const int side = block == 1 ? F117::ENGINE_START::LEFT : F117::ENGINE_START::RIGHT;
        // N2 is reported whenever the core turns: on the starter, while lit, and spooling down after shutdown.
        const double normN2 = limit(F117::engines[side].N2 / 100.0, 0.0, 1.0);
        switch (field)
        {
        case ED_FM_ENGINE_0_RPM:
        case ED_FM_ENGINE_0_RELATED_RPM:
        case ED_FM_ENGINE_0_CORE_RELATED_RPM:
            value = normN2;
            return true;
        case ED_FM_ENGINE_0_THRUST:
            value = F117::engine_thrust_N[side];
            return true;
        case ED_FM_ENGINE_0_RELATED_THRUST:
            value = limit(F117::engine_thrust_N[side] / (kMaxThrustNewtons / 2.0), 0.0, 1.0);
            return true;
        case ED_FM_ENGINE_0_TEMPERATURE:
            value = std::isfinite(F117::engine_egt_C[side]) ? F117::engine_egt_C[side] : 0.0;
            return true;
        case ED_FM_ENGINE_0_OIL_PRESSURE:
            value = F117::engine_oil_psi[side] * kPaPerPsi;
            return true;
        case ED_FM_ENGINE_0_FUEL_FLOW:
            value = F117::engine_fuel_flow_kg_s[side];
            return true;
        case ED_FM_ENGINE_0_COMBUSTION:
            value = g_engineStarts.engine[side].lit() ? 1.0 : 0.0;
            return true;
        }

        return false;
    }

    bool try_get_misc_param(unsigned index, double& value)
    {
        const struct { unsigned id; double reading; } cockpit[] = {
            {ED_FM_FUEL_INTERNAL_FUEL, F117::internal_fuel},
            {ED_FM_FUEL_TOTAL_FUEL, F117::internal_fuel},
            {ED_FM_OXYGEN_SUPPLY, kAirframeOxygenSupply},
            {ED_FM_FLOW_VELOCITY, kDefaultFlowVelocity},
            {ED_FM_FC3_STICK_PITCH, F117::FLIGHTCONTROLS::longStickInput},
            {ED_FM_FC3_STICK_ROLL, F117::FLIGHTCONTROLS::latStickInput},
            {ED_FM_FC3_RUDDER_PEDALS, F117::pedInput},
            {ED_FM_FC3_AUTOPILOT_STATUS, static_cast<double>(F117::alt_hold)},
        };
        for (const auto& parameter : cockpit)
            if (parameter.id == index) { value = parameter.reading; return true; }
        return false;
    }
}


double ed_fm_get_param(unsigned index)
{
    double value = 0.0;

    if (try_get_suspension_param(index, value))
    {
        return value;
    }

    if (try_get_engine_param(index, value))
    {
        return value;
    }

    if (try_get_misc_param(index, value))
    {
        return value;
    }

    return 0;
}



namespace
{
    void clear_pending_sim_events()
    {
        while (!g_simEvents.empty())
        {
            g_simEvents.pop();
        }
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
        if (g_damageLog)
        {
            fprintf(g_damageLog, "\n*** %s called ***\n", label);
            fflush(g_damageLog);
        }
    }

    void reset_session_controls()
    {
        F117::simInitialized = F117::ACTUATORS::initialized = false;
        F117::FLIGHTCONTROLS::reset_runtime_state();
        F117::alt_hold = F117::altroll_hold = F117::horiz_hold = 0;
        F117::airRefuelDoorOpen = false;
        F117::pitchTrim = F117::rollTrim = F117::yawTrim = 0.0;
        F117::pedInput = F117::WheelBrakeCommand = 0.0;
        F117::nwsAuthorityHeld = false;
        F117::noseWheelYawRange_RAD = F117::kNoseWheelYawLow_RAD;
        for (double* input : {&F117::FLIGHTCONTROLS::latStickInput,
            &F117::FLIGHTCONTROLS::longStickInput, &F117::FLIGHTCONTROLS::longStickForce}) *input = 0.0;
        F117::throttleInput = F117::throttle_state = F117::thrust_N = 0.0;
        F117::engineswitch = false;
        g_engineStarts.reset(false);
        for (int side = 0; side < 2; ++side)
        {
            F117::engines[side] = F117::ENGINE::EngineState();
            F117::engine_thrust_N[side] = F117::engine_fuel_flow_kg_s[side] = F117::engine_oil_psi[side] = 0.0;
            F117::engine_egt_C[side] = NAN;
        }
        F117::elev_pos = F117::rudder_pos = 0.0f;
        F117::elevator_DEG = F117::aileron_DEG = F117::rudder_DEG = 0.0;
        F117::elevator_DEG_commanded = F117::aileron_DEG_commanded = F117::rudder_DEG_commanded = 0.0;
        F117::elevator_PCT = F117::aileron_PCT = F117::rudder_PCT = 0.0;
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::tailhook_command = F117::tailhook_pos = 0.0;
        F117::dragchute_command = F117::dragchute = 0.0;
        F117::misc_cmd = F117::misc_state = 0.0;
        F117::surface_height_valid = F117::altitude_valid = false;
        for (bool& wow : F117::suspension_wow) wow = false;
        F117::ACTUATORS::reset_all();
        g_autothrottle.reset();
        g_paars.reset();
        g_paarsEngagedAutothrottle = false;
        g_paarsRollRateDegS = 0.0;
    }

    void apply_ground_start_state(bool enginesRunning)
    {
        reset_session_controls();
        F117::ACTUATORS::gear.set(kGearCommandDown);
        F117::gearDown = kGearCommandDown;
        F117::GearCommand = kGearCommandDown;
        F117::weight_on_wheels = F117::wow_from_draw_args = true;
        F117::throttleInput = 0.0;
        F117::engineswitch = enginesRunning;
        g_engineStarts.reset(enginesRunning);
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::engines[0].N2 = F117::engines[1].N2 = enginesRunning ? kGroundStartIdleN2 : 0.0;
        reset_damage_and_events();
        reset_weapon_release_sequence();
        reset_laser();
    }

    void apply_air_start_state()
    {
        reset_session_controls();
        F117::gearDown = kGearCommandUp;
        F117::GearCommand = kGearCommandUp;
        F117::weight_on_wheels = F117::wow_from_draw_args = false;
        F117::throttleInput = kAirStartThrottle;
        F117::throttle_state = kAirStartThrottle;
        g_engineStarts.reset(true);
        F117::engineswitch = g_engineStarts.any_running();
        F117::rolling_friction = kWheelBrakeOffFriction;
        F117::engines[0].N2 = F117::engines[1].N2 = kAirStartN2;
        reset_damage_and_events();
        reset_weapon_release_sequence();
        reset_laser();
    }
}

// Reset transient FM state when the aircraft is destroyed, restarted, or unloaded.

void ed_fm_release(void)
{
    reset_session_controls();
    // DCS will collect the next aircraft's initial fuel mass afresh.
    F117::fuel_mass_in_dcs = F117::DeltaTime = 0.0;
    F117::gearDown = F117::GearCommand = 0.0;
    F117::weight_on_wheels = F117::wow_from_draw_args = false;
    reset_damage_and_events();
    reset_weapon_release_sequence();
    reset_laser();
    CockpitInterop::ResetIRADSSession();
}


// Conditions to make the screen shake in first-person view.
double ed_fm_get_shake_amplitude(void)
{
    const double engineLoss = 1.0 - (g_damage.leftEngine + g_damage.rightEngine) / 2.0;
    // Highest-priority active source wins, matching the aircraft's existing shake schedule.
    const struct { bool active; double amplitude; } sources[] = {
        {g_damage.leftWing < 0.2 || g_damage.rightWing < 0.2, 10.0},
        {g_damage.cockpit < 0.5, 2.0 * (1.0 - g_damage.cockpit)},
        {engineLoss > 0.3, engineLoss * 5.0},
        {F117::az > 80.0, F117::az / 150.0},
        {F117::mach >= 0.1 && F117::alpha_DEG > 45.0, F117::alpha_DEG / 100.0},
        {F117::mach >= 0.05 && F117::beta_DEG > 15.0, F117::beta_DEG / 50.0},
    };
    for (const auto& source : sources) if (source.active) return source.amplitude;
    return 0.0;
}

// Optional DCS easy-flight hook. This FM currently leaves game-mode behavior unchanged.
void ed_fm_set_easy_flight(bool /*enabled*/)
{
    // There is one control-law configuration for both DCS difficulty settings.
}

// DCS hook for the unlimited-fuel option.
void ed_fm_unlimited_fuel(bool enabled)
{
    F117::param_class.set_unlimited_fuel(enabled);
}

// Cold-start initialization.

void ed_fm_cold_start()
{
    apply_ground_start_state(false);
    log_damage_lifecycle_event("ed_fm_cold_start");
    CockpitInterop::InjectIRADSSensor_Late();
}


// Ground hot-start initialization.

void ed_fm_hot_start()
{
    apply_ground_start_state(true);
    log_damage_lifecycle_event("ed_fm_hot_start");
    CockpitInterop::InjectIRADSSensor_Late();
}


// Air-start initialization.

void ed_fm_hot_start_in_air()
{
    apply_air_start_state();
    log_damage_lifecycle_event("ed_fm_hot_start_in_air");
    CockpitInterop::InjectIRADSSensor_Late();
}


// Push a fire event into the simulation event queue
// API layout (wHumanCustomPhysicsAPI.h):
//   [0] = handle, [1-3] = position XYZ, [4-6] = direction XYZ, [7] = speed, [8] = scale
static void pushFireEvent(int handle, double x, double y, double z)
{
    if (handle < 0 || handle >= 9 || g_damage.fireActive[handle]) return;
    g_damage.fireActive[handle] = true;
	ed_fm_simulation_event evt = {};
	evt.event_type = ED_FM_EVENT_FIRE;
	evt.event_params[0] = (float)handle;  // fire control handle
	evt.event_params[1] = (float)x;       // fire origin X (body space)
	evt.event_params[2] = (float)y;       // fire origin Y
	evt.event_params[3] = (float)z;       // fire origin Z
	evt.event_params[4] = 0.0f;           // emitter direction X
	evt.event_params[5] = 1.0f;           // emitter direction Y (up)
	evt.event_params[6] = 0.0f;           // emitter direction Z
	evt.event_params[7] = 5.0f;           // particle speed
	evt.event_params[8] = 1.0f;           // scale (>0 = fire on, <=0 = fire off)
	g_simEvents.push(evt);
}

// DCS calls this to dequeue simulation events (fire, smoke, failures)
bool ed_fm_pop_simulation_event(ed_fm_simulation_event & out)
{
	if (g_simEvents.empty()) return false;
	out = g_simEvents.front();
	g_simEvents.pop();
	if (g_damageLog)
	{
		fprintf(g_damageLog, "ed_fm_pop_simulation_event: type=%u handle=%.0f pos=(%.1f,%.1f,%.1f) dir=(%.1f,%.1f,%.1f) speed=%.1f scale=%.1f\n",
			out.event_type, out.event_params[0],
			out.event_params[1], out.event_params[2], out.event_params[3],
			out.event_params[4], out.event_params[5], out.event_params[6],
			out.event_params[7], out.event_params[8]);
		fflush(g_damageLog);
	}
	return true;
}


namespace
{
    bool try_open_damage_log_path(const char* path)
    {
        g_damageLog = fopen(path, "w");
        if (!g_damageLog)
        {
            return false;
        }

        fprintf(g_damageLog, "=== F-117 Damage Debug Log (DCS-owned visuals v2) ===\n");
        fprintf(g_damageLog, "Log opened at: %s\n", path);
        fprintf(g_damageLog, "invincible_value at init: %d\n\n", static_cast<int>(F117::param_class.invincible_value));
        fflush(g_damageLog);
        return true;
    }

    void log_damage_callback_header(int element, double element_integrity_factor)
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "ed_fm_on_damage called: Element=%d, integrity=%.4f, invincible=%d\n",
                element, element_integrity_factor, static_cast<int>(F117::param_class.invincible_value));
            fflush(g_damageLog);
        }
    }

    void log_damage_callback_skipped_immortal()
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog, "  -> SKIPPED (immortal)\n");
            fflush(g_damageLog);
        }
    }

    double damage_integrity_from_dcs(double reported)
    {
        const double integrity = limit(reported, 0.0, 1.0);
        return integrity <= kDamageCellDestroyedIntegrity ? 0.0 : integrity;
    }

    // Combines both panels of one wing. A destroyed panel takes the surfaces it carries with it,
    // because DCS does not report the dependent cells to the EFM.
    void update_wing_from_sections(double& wing, double& inner, double& outer,
        double& innerElevon, double& outerElevon)
    {
        wing = min(wing, inner * (1.0 - kOuterWingLiftShare + kOuterWingLiftShare * outer));
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
            g_damage.nose = min(g_damage.nose, integrity);
            return "nose";
        case kDamageElementCockpit:
            g_damage.cockpit = min(g_damage.cockpit, integrity);
            return "cockpit";
        case kDamageElementFuselageLeft:
            g_damage.leftFuselage = min(g_damage.leftFuselage, integrity);
            return "left fuselage";
        case kDamageElementFuselageRight:
            g_damage.rightFuselage = min(g_damage.rightFuselage, integrity);
            return "right fuselage";
        case kDamageElementBelly:
            g_damage.belly = min(g_damage.belly, integrity);
            return "weapon bay doors";
        case kDamageElementEngineLeft:
            g_damage.leftEngine = min(g_damage.leftEngine, integrity);
            if (g_damage.leftEngine < kFireDamageThreshold)
                pushFireEvent(8, -4.45, 0.08, -1.7);
            return "left engine";
        case kDamageElementEngineRight:
            g_damage.rightEngine = min(g_damage.rightEngine, integrity);
            if (g_damage.rightEngine < kFireDamageThreshold)
                pushFireEvent(7, -4.45, 0.08, 1.7);
            return "right engine";
        case kDamageElementTailLeft:
            g_damage.leftTailStructure = min(g_damage.leftTailStructure, integrity);
            if (g_damage.leftTailStructure <= 0.0)
                g_damage.leftTail = 0.0;
            return "left tail structure";
        case kDamageElementTailRight:
            g_damage.rightTailStructure = min(g_damage.rightTailStructure, integrity);
            if (g_damage.rightTailStructure <= 0.0)
                g_damage.rightTail = 0.0;
            return "right tail structure";
        case kDamageElementWingInnerLeft:
            g_damage.leftWingInner = min(g_damage.leftWingInner, integrity);
            update_wing_from_sections(g_damage.leftWing, g_damage.leftWingInner, g_damage.leftWingOuter,
                g_damage.leftInnerElevon, g_damage.leftOuterElevon);
            return "left inner wing";
        case kDamageElementWingInnerRight:
            g_damage.rightWingInner = min(g_damage.rightWingInner, integrity);
            update_wing_from_sections(g_damage.rightWing, g_damage.rightWingInner, g_damage.rightWingOuter,
                g_damage.rightInnerElevon, g_damage.rightOuterElevon);
            return "right inner wing";
        case kDamageElementWingOuterLeft:
            g_damage.leftWingOuter = min(g_damage.leftWingOuter, integrity);
            update_wing_from_sections(g_damage.leftWing, g_damage.leftWingInner, g_damage.leftWingOuter,
                g_damage.leftInnerElevon, g_damage.leftOuterElevon);
            return "left outer wing";
        case kDamageElementWingOuterRight:
            g_damage.rightWingOuter = min(g_damage.rightWingOuter, integrity);
            update_wing_from_sections(g_damage.rightWing, g_damage.rightWingInner, g_damage.rightWingOuter,
                g_damage.rightInnerElevon, g_damage.rightOuterElevon);
            return "right outer wing";
        case kDamageElementNoseGear:
            g_damage.noseGear = min(g_damage.noseGear, integrity);
            return "nose gear bay";
        case kDamageElementLeftGear:
            g_damage.leftGear = min(g_damage.leftGear, integrity);
            return "left gear bay";
        case kDamageElementRightGear:
            g_damage.rightGear = min(g_damage.rightGear, integrity);
            return "right gear bay";
        // DCS applies wheel damage through each suspension's damage_element.
        case kDamageElementNoseWheel:
            g_damage.noseWheel = min(g_damage.noseWheel, integrity);
            return "nose wheel";
        case kDamageElementLeftWheel:
            g_damage.leftWheel = min(g_damage.leftWheel, integrity);
            return "left wheel";
        case kDamageElementRightWheel:
            g_damage.rightWheel = min(g_damage.rightWheel, integrity);
            return "right wheel";
        case kDamageElementAileronLeft:
            g_damage.leftOuterElevon = min(g_damage.leftOuterElevon, integrity);
            return "left outer elevon";
        case kDamageElementAileronRight:
            g_damage.rightOuterElevon = min(g_damage.rightOuterElevon, integrity);
            return "right outer elevon";
        case kDamageElementFlapLeft:
            g_damage.leftInnerElevon = min(g_damage.leftInnerElevon, integrity);
            return "left inner elevon";
        case kDamageElementFlapRight:
            g_damage.rightInnerElevon = min(g_damage.rightInnerElevon, integrity);
            return "right inner elevon";
        case kDamageElementRudderLeft:
            g_damage.leftTail = min(g_damage.leftTail, integrity);
            return "left ruddervator";
        case kDamageElementRudderRight:
            g_damage.rightTail = min(g_damage.rightTail, integrity);
            return "right ruddervator";
        default:
            return "IGNORED(unhandled)";
        }
    }

    void log_damage_mapping_result(const char* mapped)
    {
        if (g_damageLog)
        {
            fprintf(g_damageLog,
                "  -> mapped to: %s | State: LW=%.2f RW=%.2f LE=%.2f RE=%.2f LT=%.2f RT=%.2f CP=%.2f\n",
                mapped,
                g_damage.leftWing, g_damage.rightWing,
                g_damage.leftEngine, g_damage.rightEngine,
                g_damage.leftTail, g_damage.rightTail,
                g_damage.cockpit);
            fflush(g_damageLog);
        }
    }
}


static void initDamageLog()
{
    if (g_damageLog) return;

    const std::string paths[] = {
        build_saved_games_dcs_path("DCS", "Logs\\F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS", "F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS.openbeta", "Logs\\F117_Damage_Debug.log"),
        build_saved_games_dcs_path("DCS.openbeta", "F117_Damage_Debug.log"),
        "F117_Damage_Debug.log"
    };

    for (const std::string& path : paths)
    {
        if (!path.empty() && try_open_damage_log_path(path.c_str()))
        {
            return;
        }
    }
}


// Damage callback. DCS reports remaining integrity from 1.0 intact down to 0.1 destroyed.

void ed_fm_on_damage(int element, double remainingIntegrity)
{
    initDamageLog();
    log_damage_callback_header(element, remainingIntegrity);
    if (!F117::param_class.invincible_value)
    {
        log_damage_callback_skipped_immortal();
        return;
    }
    if (!std::isfinite(remainingIntegrity)) return;
    const double normalized = damage_integrity_from_dcs(remainingIntegrity);
    log_damage_mapping_result(apply_damage_for_element(element, normalized));
}


// Restore repairable FM damage state and clear queued events.

void ed_fm_repair()
{
    reset_damage_and_events();
}


void ed_fm_set_immortal(bool enabled)
{
	F117::param_class.set_immortal(enabled);
	initDamageLog();
	if (g_damageLog)
	{
		fprintf(g_damageLog, "ed_fm_set_immortal called: value=%d -> invincible_value=%d\n",
			(int)enabled, (int)F117::param_class.invincible_value);
		fflush(g_damageLog);
	}
}

// Returns true when any component has taken damage, triggering the repair process
bool ed_fm_need_to_be_repaired()
{
	return g_damage.leftWing    < 1.0
		|| g_damage.rightWing   < 1.0
		|| g_damage.leftEngine  < 1.0
		|| g_damage.rightEngine < 1.0
		|| g_damage.leftTail    < 1.0
		|| g_damage.rightTail   < 1.0
		|| g_damage.cockpit     < 1.0
        || g_damage.leftOuterElevon < 1.0
        || g_damage.rightOuterElevon < 1.0
        || g_damage.leftInnerElevon < 1.0
        || g_damage.rightInnerElevon < 1.0
        || g_damage.nose < 1.0
        || g_damage.leftFuselage < 1.0
        || g_damage.rightFuselage < 1.0
        || g_damage.belly < 1.0
        || g_damage.leftTailStructure < 1.0
        || g_damage.rightTailStructure < 1.0
        || g_damage.leftWingInner < 1.0
        || g_damage.rightWingInner < 1.0
        || g_damage.leftWingOuter < 1.0
        || g_damage.rightWingOuter < 1.0
        || g_damage.noseGear < 1.0
        || g_damage.leftGear < 1.0
        || g_damage.rightGear < 1.0
        || g_damage.noseWheel < 1.0
        || g_damage.leftWheel < 1.0
        || g_damage.rightWheel < 1.0;
}

// -----------------------------------------------------------------------
// New API functions (DCS 2.5.7+ / July 2025 SDK) - stubs
// DCS calls these via GetProcAddress; returning safely is sufficient if unused.
// -----------------------------------------------------------------------

// Called asynchronously with cloud/precipitation density at the aircraft position.
void ed_fm_set_clouds_density(const atmo_clouds_and_precipation & /*info*/)
{
}

// DCS requests the set of world points at which it should sample wind for this FM.
// Set in_out.field = nullptr to opt out.
void ed_fm_wind_vector_field_update_request(wind_vector_field & in_out)
{
	in_out.field = nullptr;
	in_out.field_points_count = 0;
}

// Called after DCS has filled all wind sample points requested above.
void ed_fm_wind_vector_field_done()
{
}

// DCS pushes in-game events (e.g. carrier cat/trap) to the FM.
bool ed_fm_push_simulation_event(const ed_fm_simulation_event & /*in*/)
{
	return false;
}

// DCS feeds back per-gear suspension state each frame.
// idx: 0 = nose gear, 1 = left main, 2 = right main.
void ed_fm_suspension_feedback(int idx, const ed_fm_suspension_info * info)
{
    if (!info || idx < 0 || idx > 2) return;
    F117::suspension_wow[idx] = (info->struct_compression > 0.001);
}

// Leading-edge vortex and wing vapour. Shaped from two sources:
//
// The aircraft model. The planform is one straight 67.3 deg leading edge from
// the nose apex to the wingtip, at y = -0.28 m, so the local semispan is
// s(x) = 0.418 * (9.95 - x). The edge reaches the tip at x = -6.3 m (6.57 m
// out) and the tip trailing edge ends near x = -7.0 m. Upper-surface heights
// under the vortex (kSurf80 / kSurf90, at 80% and 90% of s) were sampled from
// the model.
//
// Vermeersch, "Investigation of the F117A Vortical Flow Characteristics"
// (MIT, 1993; docs/reference in the mod repo):
//   - the vortex starts at the nose apex and runs just inboard of the leading
//     edge all the way to the tip (Fig. 6.24);
//   - wind-tunnel smoke shows it clearly at 7 and 10 deg AoA, lying close to
//     the surface; it grows stronger and lifts away from 15 deg;
//   - breakdown sits just aft of the trailing edge at 15 deg and moves forward
//     with AoA: X/L = 0.775 at 20, 0.68 at 25, 0.555 at 30 deg (Fig. 6.25,
//     X measured from the nose, L the aircraft length);
//   - the breakdown point is unsteady; it is a bubble at 20 deg and a spiral
//     at 25 and 30 deg.
//
// Visible vapour also needs the pressure drop in the core to reach
// condensation, so opacity scales with dynamic pressure times AoA and fades
// with altitude.
bool ed_fm_LERX_vortex_update(unsigned idx, LERX_vortex& out)
{
    if (idx > 1)
        return false;

    out.version = 1; // per-point opacity, scaled by out.opacity
    out.spline_point_size_in_bytes = sizeof(LERX_vortex_spline_point);

    auto detach = [&out]() {
        out.spline = nullptr;
        out.spline_points_count = 0;
        return true;
    };
    auto clampf = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    auto smoothstep = [&clampf](float e0, float e1, float v) {
        const float t = clampf((v - e0) / (e1 - e0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };

    // DCS calls this once per side each frame; advance the shared clock once.
    static float vortex_time = 0.0f;
    if (idx == 0)
        vortex_time += (float)F117::DeltaTime;
    const float time = vortex_time;

    if (F117::weight_on_wheels || F117::totalVelocity_MPS < 35.0)
        return detach();

    const float aoa = (float)F117::alpha_DEG;
    const float beta = (float)F117::beta_DEG;
    const float vel = (float)F117::totalVelocity_MPS;
    float rho = (float)F117::ambientDensity_KgPerM3;
    if (rho < 0.1f) rho = 1.225f;

    // Vortex forms from about 5 deg and is well established by 8-10 deg.
    const float vortex_strength = 1.0f / (1.0f + expf(-(aoa - 6.0f) * 0.9f));
    if (vortex_strength < 0.02f)
        return detach();

    // =========================================================================
    // ALTITUDE-DEPENDENT CONDENSATION FADE
    //
    // The temperature/dew-point spread widens with altitude, so condensation
    // gets harder. Ambient density stands in for both pressure and
    // temperature; altitude gives a hard ceiling for non-standard days.
    //   Sea level ~1.0, 10,000 ft ~0.85, 20,000 ft ~0.40, 30,000 ft ~0.10
    // =========================================================================
    const float rho_ratio = rho / 1.225f;
    float humidity_factor = powf(rho_ratio, 3.5f);
    if (rho_ratio > 0.95f) // low-altitude moisture boost, below ~500 m
        humidity_factor += (rho_ratio - 0.95f) / 0.05f * 0.08f;
    const float alt_m = (float)F117::altitude_m;
    if (alt_m > 8000.0f)
        humidity_factor *= (std::max)(0.0f, 1.0f - (alt_m - 8000.0f) / 2000.0f);
    humidity_factor = clampf(humidity_factor, 0.0f, 1.0f);

    // Core suction scales with q * CL, and CL with AoA. A slow approach at
    // 10 deg shows little vapour; a fast, loaded turn shows a lot.
    const float q = 0.5f * rho * vel * vel;
    const float suction = q * clampf(aoa, 0.0f, 25.0f) / 12.0f;
    const float condensation = 0.25f + 0.75f * smoothstep(3000.0f, 12000.0f, suction);

    // Sideslip: the windward vortex is stronger and breaks down earlier.
    const float side_bias = (idx == 1) ? beta : -beta;
    const float asym = clampf(1.0f + side_bias * 0.12f, 0.15f, 1.85f);

    // Photographs of high-g passes show one condensation cloud over the aft
    // body and no separate vortex tubes. The mod's overwing vapour effect
    // draws that cloud from about 8 deg AoA and at full strength from 11.5 deg
    // (OpacityOverPower in Effects/f-117-nighthawk_overwingVapor.lua), so the
    // vortices fade out over the same range.
    const float cloud_takeover = smoothstep(8.0f, 11.5f, aoa);

    const float kOpacityMax = 0.20f;
    out.opacity = kOpacityMax * vortex_strength * condensation * humidity_factor * asym
                * (1.0f - cloud_takeover);
    if (out.opacity < 0.004f)
        return detach();

    // =========================================================================
    // GEOMETRY (body frame: x forward, y up, z right; metres)
    // =========================================================================
    const float kNoseX = 10.02f;        // nose tip
    const float kLength = 19.43f;       // reference length for breakdown X/L
    const float kApexX = 9.95f;         // leading edge line meets the centreline
    const float kSemispanPerM = 0.418f; // tan(90 - 67.3 deg)
    const float kLeY = -0.28f;          // leading edge height
    const float kTipX = -6.30f;         // leading edge reaches the tip
    const float kTeX = -7.00f;          // tip trailing edge

    static const int kSurfN = 17;
    static const float kSurfX[kSurfN] = {
        8.5f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f,
        0.0f, -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -6.3f };
    static const float kSurf90[kSurfN] = {
        -0.22f, -0.18f, -0.16f, -0.10f, -0.04f, -0.02f, -0.17f, -0.15f, -0.14f,
        -0.12f, -0.11f, -0.09f, -0.10f, -0.12f, -0.15f, -0.17f, -0.18f };
    static const float kSurf80[kSurfN] = {
        -0.22f, -0.09f, 0.01f, 0.09f, 0.17f, 0.22f, 0.11f, -0.03f, 0.00f,
        0.03f, 0.03f, -0.01f, -0.05f, -0.09f, -0.15f, -0.23f, -0.27f };

    // Upper-surface height at station x and fraction f of the local semispan.
    auto surface_y = [&](float x, float f) -> float {
        if (x < kTeX)
            return -1000.0f; // behind the wing: nothing to clear
        x = clampf(x, kSurfX[kSurfN - 1], kSurfX[0]);
        int k = 0;
        while (k < kSurfN - 2 && x < kSurfX[k + 1])
            ++k;
        const float u = (kSurfX[k] - x) / (kSurfX[k] - kSurfX[k + 1]);
        const float y90 = kSurf90[k] + (kSurf90[k + 1] - kSurf90[k]) * u;
        const float y80 = kSurf80[k] + (kSurf80[k + 1] - kSurf80[k]) * u;
        if (f >= 0.9f)
            return y90 + (kLeY - y90) * clampf((f - 0.9f) / 0.1f, 0.0f, 1.0f);
        return y80 + (y90 - y80) * clampf((f - 0.8f) / 0.1f, 0.0f, 1.0f);
    };

    // Conical scaling with local semispan. At low AoA the core hugs the
    // surface close to the edge; with AoA it moves inboard, grows and lifts.
    const float a = clampf(aoa, 0.0f, 35.0f);
    const float f_core = 0.90f - 0.003f * a;      // 0.87 s at 10 deg, 0.81 s at 30
    const float radius_k = 0.045f + 0.0025f * a;  // vapour radius / s
    const float gap_k = 0.0025f * a;              // core clearance / s
    const float trail_slope = 0.5f * tanf(a * 0.0174533f); // wake rises toward the freestream

    // =========================================================================
    // BREAKDOWN: X/L through Vermeersch Fig. 6.25 (20, 25, 30 deg) and the
    // text's "just aft of the trailing edge" at 15 deg, extended linearly.
    // =========================================================================
    float burst_xl;
    if (aoa < 20.0f)
        burst_xl = 0.775f - 0.022f * (aoa - 20.0f);
    else if (aoa < 25.0f)
        burst_xl = 0.775f - 0.019f * (aoa - 20.0f);
    else
        burst_xl = 0.680f - 0.025f * (aoa - 25.0f);
    burst_xl -= 0.006f * side_bias;
    burst_xl += 0.012f * sinf(time * 1.7f + idx * 2.1f) + 0.006f * sinf(time * 4.3f + idx * 0.7f);
    burst_xl = clampf(burst_xl, 0.40f, 1.40f);
    const float burst_x = kNoseX - burst_xl * kLength;
    const bool spiral = aoa > 22.5f; // bubble at 20 deg, spiral at 25 and 30

    // =========================================================================
    // PATH: apex region to a short wake behind the tip
    // =========================================================================
    static constexpr int N = 12;
    const float x_first = 8.5f, x_last = -9.0f;
    float bx[N], by[N], bz[N], br[N], dist[N];
    for (int i = 0; i < N; ++i)
    {
        const float x = x_first + (x_last - x_first) * (float)i / (float)(N - 1);
        const float xs = (std::max)(x, kTipX);
        const float s = kSemispanPerM * (kApexX - xs);
        const float r = radius_k * s;
        const float surf = (std::max)(surface_y(x, f_core), surface_y(x, f_core - radius_k));
        float y = surf + r + gap_k * s;
        if (x < kTipX)
        {
            const float y_tip = (std::max)(surface_y(kTipX, f_core), surface_y(kTipX, f_core - radius_k))
                + r + gap_k * s;
            y = y_tip + (kTipX - x) * trail_slope;
        }
        bx[i] = x; by[i] = y; bz[i] = f_core * s; br[i] = r;
        dist[i] = (i == 0) ? 0.0f : dist[i - 1] + sqrtf(
            (bx[i] - bx[i - 1]) * (bx[i] - bx[i - 1]) +
            (by[i] - by[i - 1]) * (by[i] - by[i - 1]) +
            (bz[i] - bz[i - 1]) * (bz[i] - bz[i - 1]));
    }

    // Distance along the spline to the breakdown point.
    float burst_d;
    float burst_r;
    if (burst_x >= bx[0])
    {
        burst_d = 0.0f;
        burst_r = br[0];
    }
    else if (burst_x <= bx[N - 1])
    {
        burst_d = dist[N - 1] + (bx[N - 1] - burst_x);
        burst_r = br[N - 1];
    }
    else
    {
        int k = 0;
        while (bx[k + 1] > burst_x)
            ++k;
        const float u = (bx[k] - burst_x) / (bx[k] - bx[k + 1]);
        burst_d = dist[k] + (dist[k + 1] - dist[k]) * u;
        burst_r = br[k] + (br[k + 1] - br[k]) * u;
    }
    out.explosion_start = (std::max)(0.5f, burst_d);

    static LERX_vortex_spline_point vortex_buffers[2][N];
    LERX_vortex_spline_point* sp = vortex_buffers[idx];
    const float zs = (idx == 0) ? -1.0f : 1.0f;

    for (int i = 0; i < N; ++i)
    {
        const float t = (float)i / (float)(N - 1);
        const float excess = (std::max)(0.0f, dist[i] - burst_d);
        const float post = clampf(excess / 0.8f, 0.0f, 1.0f); // blend over 0.8 m

        // Radius: conical before breakdown, rapid wake-type growth after.
        float radius = br[i];
        float y = by[i];
        if (excess > 0.0f)
        {
            radius = (std::min)(1.8f, burst_r + 0.35f * excess);
            y += 0.12f * excess;
        }

        // Motion: slight core precession before breakdown; after it, a
        // swirling spiral or a pulsing bubble.
        const float pre_amp = 0.04f * br[i];
        float jy = (1.0f - post) * pre_amp * (sinf(time * 6.0f + i * 1.9f) + 0.5f * sinf(time * 11.0f + i * 3.1f));
        float jz = (1.0f - post) * pre_amp * 0.5f * sinf(time * 7.0f + i * 2.3f);
        if (post > 0.0f)
        {
            const float amp = post * radius * (spiral ? 0.30f : 0.12f);
            const float phase = time * (spiral ? 9.0f : 5.0f) - excess * 2.5f + i * 0.9f;
            jy += amp * sinf(phase) + 0.3f * amp * sinf(time * 23.0f + i * 4.1f);
            jz += amp * cosf(phase) + 0.3f * amp * cosf(time * 17.0f + i * 5.7f);
        }

        // Opacity shape, 0..1, scaled by out.opacity.
        //   - fades in aft of the nose as suction builds (photos show vapour
        //     from about the inlet back);
        //   - off the wing the pressure recovers and unburst vapour clears;
        //   - burst vapour turns to a thin haze.
        float shape = smoothstep(8.0f, 4.5f, bx[i]);
        if (bx[i] < kTeX && excess <= 0.0f)
            shape *= expf(-(kTeX - bx[i]) / 1.5f);
        shape *= 1.0f - post * 0.75f * (1.0f - expf(-excess / 2.0f));

        const float noise1 = sinf(t * 4.5f + time * 1.3f) * sinf(t * 7.2f - time * 0.9f);
        const float noise2 = sinf(t * 13.0f + time * 3.7f + i * 0.8f) * cosf(t * 9.1f + time * 2.1f);
        const float noise3 = sinf(t * 29.0f + time * 11.0f + i * 2.4f);
        const float density = clampf(0.80f + 0.12f * noise1 + 0.06f * noise2 + 0.02f * noise3
            + post * 0.15f * sinf(time * 19.0f + i * 3.3f), 0.35f, 1.0f);

        sp[i].radius = radius * (1.0f + 0.10f * noise1 + 0.05f * noise2);
        sp[i].opacity = clampf(shape * density, 0.0f, 1.0f);
        sp[i].pos[0] = bx[i];
        sp[i].pos[1] = y + jy;
        sp[i].pos[2] = (bz[i] + jz) * zs;

        // Unit tangent along the undisturbed path.
        const int i0 = (i == 0) ? 0 : i - 1;
        const int i1 = (i == N - 1) ? N - 1 : i + 1;
        float dx = bx[i1] - bx[i0], dy = by[i1] - by[i0], dz = bz[i1] - bz[i0];
        float mag = sqrtf(dx * dx + dy * dy + dz * dz);
        if (mag < 1e-6f) mag = 1.0f;
        sp[i].vel[0] = dx / mag;
        sp[i].vel[1] = dy / mag;
        sp[i].vel[2] = dz / mag * zs;
    }

    out.spline = sp;
    out.spline_points_count = N;
    return true;
}
