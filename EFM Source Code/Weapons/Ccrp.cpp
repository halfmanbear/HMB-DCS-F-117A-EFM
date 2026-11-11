#include "stdafx.h"
#include "Ccrp.h"

#include <algorithm>
#include <cmath>

#include "../Avionics/CockpitBase_Interop.h"
#include "../Avionics/JdamEnvelope.h"
#include "../Avionics/Weapons_Interop.h"
#include "../Cockpit/ParamBridge.h"
#include "../FlightModel/State.h"

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

namespace weapons
{
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
    g_iradsLockParamsInitialized = (g_iradsLockValidParam != nullptr) && (g_iradsLockWorldXParam != nullptr) &&
                                   (g_iradsLockWorldYParam != nullptr) && (g_iradsLockWorldZParam != nullptr) &&
                                   (g_iradsLockRangeParam != nullptr) && (g_iradsLockAzimuthParam != nullptr) &&
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
    g_releaseCueParamsInitialized = (g_releaseCueValidParam != nullptr) && (g_releaseCueErrorParam != nullptr) &&
                                    (g_releaseCueTimeToGoParam != nullptr) && (g_releaseCueRangeParam != nullptr) &&
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
    int profileIndex = std::isfinite(rawIndex) && rawIndex >= 0.0 && rawIndex < kCcrpStoreCount
                           ? static_cast<int>(std::floor(rawIndex + 0.5))
                           : 0;
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

bool simulate_bomb_impact_point(const CcrpStoreModel& profile, const Vec3& releasePos, const Vec3& releaseVel,
                                const Vec3& worldWind, double targetAltMeters, Vec3* outImpactPos,
                                double* outImpactTime)
{
    if (outImpactPos == nullptr || outImpactTime == nullptr || profile.massKg <= 0.0 || profile.referenceAreaM2 <= 0.0)
    {
        return false;
    }

    if (releasePos.y <= targetAltMeters)
    {
        return false;
    }

    const double altitudeReference = std::max(releasePos.y, 0.0);
    const double seaLevelDensity =
        std::max(F117::ambientDensity_KgPerM3, 0.05) * exp(altitudeReference / kAtmosphereScaleHeight_M);

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
            const double rho = seaLevelDensity * exp(-std::max(pos.y, 0.0) / kAtmosphereScaleHeight_M);
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
            *outImpactTime =
                std::max(0.0, elapsed - kReleaseIntegrationStepSeconds + (kReleaseIntegrationStepSeconds * fraction));
            return true;
        }
    }

    return false;
}

bool g_releaseAlignmentValid = false;
double g_releaseMinRange = 0.0;
double g_releaseMaxRange = 0.0;

// Provisional guided-store envelope around the ballistic prediction, not a
// measured DCS DLZ. Keep the ballistic error/TTG as cues, not a pinpoint gate.
bool guided_release_in_zone(StoreKind kind, double ballisticRange, double alongTrack, double crossTrack,
                            double minimumWindow, bool* aligned)
{
    if (!aligned)
        return false;
    *aligned = false;
    if (!store_falls_ballistic(kind) || kind == StoreKind::Unguided || !std::isfinite(ballisticRange) ||
        !std::isfinite(alongTrack) || !std::isfinite(crossTrack) || !std::isfinite(minimumWindow) ||
        ballisticRange <= 0.0 || alongTrack <= 0.0 || crossTrack < 0.0 || minimumWindow < 0.0)
        return false;
    const bool jdam = kind == StoreKind::Jdam;
    const double lateralRatio = jdam ? 0.577350269 : 0.267949192;  // 30 / 15 degrees
    *aligned = crossTrack <= std::max(50.0, alongTrack * lateralRatio);
    const double rangeMargin = std::max(minimumWindow, ballisticRange * (jdam ? 0.35 : 0.20));
    const double horizontalRange = sqrt(alongTrack * alongTrack + crossTrack * crossTrack);
    return *aligned && alongTrack >= std::max(1.0, ballisticRange - rangeMargin) &&
           horizontalRange <= ballisticRange + rangeMargin;
}

bool compute_release_solution(double* outErrorMeters, double* outTimeToGoSeconds, double* outRangeMeters,
                              bool* outInZone)
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

    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(targetWorld[i]))
            return false;
    if (!std::isfinite(vx_world) || !std::isfinite(vy_world) || !std::isfinite(vz_world) ||
        !std::isfinite(position_world_cs.x) || !std::isfinite(position_world_cs.y) ||
        !std::isfinite(position_world_cs.z))
        return false;
    const CcrpStoreModel& profile = get_ccrp_profile();
    const Vec3 releaseVel{vx_world, vy_world, vz_world};
    const double dx = targetWorld[0] - position_world_cs.x;
    const double dz = targetWorld[2] - position_world_cs.z;
    const double horizSpeed = sqrt((releaseVel.x * releaseVel.x) + (releaseVel.z * releaseVel.z));
    const double horizRange = sqrt((dx * dx) + (dz * dz));
    const double dropMeters = position_world_cs.y - targetWorld[1];

    if (horizSpeed < std::max(kReleaseCueMinSpeedMPS, profile.minSpeedMPS) ||
        dropMeters < std::max(kReleaseCueMinDropMeters, profile.minDropMeters))
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
    const double airX = releaseVel.x - worldWind.x, airY = releaseVel.y - worldWind.y,
                 airZ = releaseVel.z - worldWind.z;
    const double airSpeed = std::sqrt(airX * airX + airY * airY + airZ * airZ);
    const StoreKind kind = profile.kind;
    double impactAlongTrack = 0.0;
    double errorMeters = 0.0;
    double timeToGo = 0.0;
    if (store_falls_ballistic(kind))
    {
        Vec3 impactPos{};
        double impactTime = 0.0;
        if (!simulate_bomb_impact_point(profile, position_world_cs, releaseVel, worldWind, targetWorld[1], &impactPos,
                                        &impactTime))
        {
            return false;
        }
        F117::ccrp_time_of_flight = impactTime;
        const Vec3 impactDelta{impactPos.x - position_world_cs.x, impactPos.y - position_world_cs.y,
                               impactPos.z - position_world_cs.z};
        impactAlongTrack = (impactDelta.x * vx) + (impactDelta.z * vz);
        errorMeters = alongTrack - impactAlongTrack;
        timeToGo = (errorMeters > 0.0) ? (errorMeters / std::max(horizSpeed, profile.minSpeedMPS)) : 0.0;
    }
    bool inZone = false;
    if (kind == StoreKind::Jdam || kind == StoreKind::Glide)
    {
        // Weapon-authored launch tables, interpolated in target-relative
        // height and true airspeed. No extrapolation outside the table domain.
        const bool tabled =
            kind == StoreKind::Jdam
                ? JdamEnvelope::lookup(profile.envelope, dropMeters, airSpeed, &g_releaseMinRange, &g_releaseMaxRange)
                : GlideEnvelope::lookup(profile.envelope, dropMeters, airSpeed, &g_releaseMinRange, &g_releaseMaxRange);
        if (!tabled)
            return false;
        g_releaseAlignmentValid = crossTrack <= std::max(50.0, alongTrack * 0.577350269);
        inZone = g_releaseAlignmentValid && horizRange >= g_releaseMinRange && horizRange <= g_releaseMaxRange;
        if (kind == StoreKind::Glide)
        {
            // A glide weapon has no ballistic impact: cue the distance still to close
            // to the maximum range, and time the flight at a typical glide speed.
            errorMeters = std::max(0.0, horizRange - g_releaseMaxRange);
            timeToGo = errorMeters / std::max(horizSpeed, profile.minSpeedMPS);
            F117::ccrp_time_of_flight = horizRange / kGlideWeaponSpeedMPS;
        }
    }
    else if (kind == StoreKind::Unguided)
    {
        // True CCRP: the release fires as the predicted impact crosses the target.
        g_releaseAlignmentValid = crossTrack <= kUnguidedMaxCrossTrackMeters;
        inZone = g_releaseAlignmentValid && fabs(errorMeters) <= profile.releaseWindowMeters;
        g_releaseMinRange = std::max(1.0, impactAlongTrack - profile.releaseWindowMeters);
        g_releaseMaxRange = impactAlongTrack + profile.releaseWindowMeters;
    }
    else if (store_is_missile(kind))
    {
        // Powered missile: target inside the seeker cone around the flight path
        // and slant range inside the stock launch limits.
        const double slant = sqrt(horizRange * horizRange + dropMeters * dropMeters);
        const double speed =
            sqrt(releaseVel.x * releaseVel.x + releaseVel.y * releaseVel.y + releaseVel.z * releaseVel.z);
        const double cosine =
            (slant > 1.0 && speed > 1.0)
                ? (dx * releaseVel.x - dropMeters * releaseVel.y + dz * releaseVel.z) / (slant * speed)
                : -1.0;
        g_releaseAlignmentValid = cosine >= cos(profile.seekerHalfAngleRad);
        g_releaseMinRange = profile.minRangeMeters;
        g_releaseMaxRange = profile.maxRangeMeters;
        inZone = g_releaseAlignmentValid && slant >= g_releaseMinRange && slant <= g_releaseMaxRange;
        errorMeters = std::max(0.0, slant - g_releaseMaxRange);
        timeToGo = errorMeters / std::max(horizSpeed, profile.minSpeedMPS);
        F117::ccrp_time_of_flight = slant / kMissileAverageSpeedMPS;
    }
    else
    {
        inZone = guided_release_in_zone(kind, impactAlongTrack, alongTrack, crossTrack, profile.releaseWindowMeters,
                                        &g_releaseAlignmentValid);
        g_releaseMinRange =
            std::max(1.0, impactAlongTrack - std::max(profile.releaseWindowMeters, impactAlongTrack * 0.20));
        g_releaseMaxRange = impactAlongTrack + std::max(profile.releaseWindowMeters, impactAlongTrack * 0.20);
    }
    static bool logged = false;
    static bool lastZone = false;
    static bool lastAligned = false;
    static unsigned diagnosticFrames = 0;
    if (!logged || inZone != lastZone || g_releaseAlignmentValid != lastAligned ||
        (F117::weapon_release_pickle_held && ++diagnosticFrames >= 600))
    {
        CockpitInterop::Log(
            "DELIVERY ZONE: profile=%d along=%.1f cross=%.1f ballistic=%.1f error=%.1f aligned=%d zone=%d "
            "min=%.1f max=%.1f drop=%.1f speed=%.1f",
            get_ccrp_profile_index(), alongTrack, crossTrack, impactAlongTrack, errorMeters, g_releaseAlignmentValid,
            inZone, g_releaseMinRange, g_releaseMaxRange, dropMeters, horizSpeed);
        logged = true;
        lastZone = inZone;
        lastAligned = g_releaseAlignmentValid;
        diagnosticFrames = 0;
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
    static void* request[7] = {};
    static void* result[5] = {};
    static const char* requestNames[] = {"F117_TERRAIN_OX", "F117_TERRAIN_OY", "F117_TERRAIN_OZ",     "F117_TERRAIN_DX",
                                         "F117_TERRAIN_DY", "F117_TERRAIN_DZ", "F117_TERRAIN_REQUEST"};
    static const char* resultNames[] = {"F117_TERRAIN_X", "F117_TERRAIN_Y", "F117_TERRAIN_Z", "F117_TERRAIN_HIT",
                                        "F117_TERRAIN_ACK"};
    bool handles = true;
    for (int i = 0; i < 7; ++i)
    {
        if (!request[i])
            request[i] = F117::cockpitAPI.getParamHandle(requestNames[i]);
        handles = handles && request[i];
    }
    for (int i = 0; i < 5; ++i)
    {
        if (!result[i])
            result[i] = F117::cockpitAPI.getParamHandle(resultNames[i]);
        handles = handles && result[i];
    }
    if (handles)
    {
        unsigned sequence = 0;
        double ray[6] = {};
        bool pending = CockpitInterop::GetIRADSTerrainRequest(&sequence, ray);
        if (pending && F117::cockpitAPI.getParamNumber(result[4]) == sequence)
        {
            double point[3];
            for (int i = 0; i < 3; ++i)
                point[i] = F117::cockpitAPI.getParamNumber(result[i]);
            CockpitInterop::CompleteIRADSTerrainRequest(sequence, F117::cockpitAPI.getParamNumber(result[3]) == 1.0,
                                                        point);
            pending = false;
        }
        if (pending)
            for (int i = 0; i < 6; ++i)
                F117::cockpitAPI.setParamNumber(request[i], ray[i]);
        F117::cockpitAPI.setParamNumber(request[6], pending ? sequence : 0.0);
    }

    double world[3] = {0.0, 0.0, 0.0};
    double polar[3] = {0.0, 0.0, 0.0};
    const bool valid = CockpitInterop::HasIRADSLock() && CockpitInterop::GetIRADSLockWorldPoint(world) &&
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
}  // namespace weapons
