#pragma once

// Continuously computed release point (CCRP) model for the bay stores, and the cockpit parameters that
// publish its release cue and the IRADS lock.

#include <cstddef>

#include "../EfmMath.h"

namespace weapons
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

    constexpr double kGravity_MPS2 = 9.81;
    constexpr double kReleaseCueMinSpeedMPS = 30.0;
    constexpr double kReleaseCueMinDropMeters = 10.0;
    constexpr double kAtmosphereScaleHeight_M = 8500.0;
    constexpr double kReleaseIntegrationStepSeconds = 0.02;
    constexpr double kReleaseIntegrationMaxSeconds = 180.0;
    constexpr double kUnguidedMaxCrossTrackMeters = 30.0;  // farther off the line misses the target
    constexpr double kGlideWeaponSpeedMPS = 200.0;         // JSOW mid-glide speed, for the laser/flight timer
    constexpr double kMissileAverageSpeedMPS = 300.0;      // AGM-65 average speed, for the laser/flight timer

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

    // Release alignment from the last solution, read by the release sequence.
    extern bool g_releaseAlignmentValid;
    extern double g_releaseMinRange, g_releaseMaxRange;

    void publish_weapon_release_gate_param();
    int get_ccrp_profile_index();
    const CcrpStoreModel& get_ccrp_profile();
    bool compute_release_solution(double* outErrorMeters, double* outTimeToGoSeconds, double* outRangeMeters, bool* outInZone);
    void publish_release_cue_params();
    void publish_irads_lock_params();
}
