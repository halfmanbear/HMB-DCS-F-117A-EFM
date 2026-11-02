#pragma once

#include <cstddef>
#include <cstdint>

namespace CockpitInterop
{
    struct WorldPoseMatrix
    {
        double m[16];
    };

    struct SensorFrameLayout
    {
        double halfAzimuth;
        double halfElevation;
    };
    static_assert(sizeof(SensorFrameLayout) == 0x10);

    struct SearchItemLayout
    {
        std::uint32_t objectId;
        std::uint32_t _pad04;
        double relativeAzimuth;
        double relativeElevation;
        double relativeRange;
        double localTrackOffsetX;
        double localTrackOffsetY;
        double localTrackOffsetZ;
        SensorFrameLayout candidateFrame;
        std::uint8_t hasLosIntersection;
        std::uint8_t preciseTrackCandidate;
        std::uint8_t _pad4A[6];
    };
    static_assert(sizeof(SearchItemLayout) == 0x50);

    struct AvTrackDataLayout
    {
        std::uint32_t objectId;
        std::uint32_t _pad04;
        double relativeAzimuth;
        double relativeElevation;
        double relativeRange;
        double worldTransform[16];
        double velocity[3];
        std::uint8_t _pad0B8[0xD0 - 0xB8];
        double lastUpdateModelTime;
        std::uint8_t _pad0D8[0xE8 - 0xD8];
        void* movingObjectOrType;
        double approxSize;
        double localPointOffset[3];
    };
    static_assert(offsetof(AvTrackDataLayout, localPointOffset) == 0xF8);
    static_assert(sizeof(AvTrackDataLayout) == 0x110);

    struct PointerVectorView
    {
        void** begin;
        void** end;
        void** cap;
    };
    static_assert(sizeof(PointerVectorView) == 0x18);

    struct CcCockpitContextLayout
    {
        std::uint8_t _pad000[0x10];
        void* movingObjectRaw;  // the aircraft object
        union
        {
            std::uint8_t carrierLinkBaseStorage[0x18];  // carrier link
            struct
            {
                std::uint8_t _pad018[0x10];
                void* carrierLinkHostRaw;  // carrier link host pointer
            };
        };
        std::uint8_t devicesKeeperStorage[0x18];
        std::uint8_t _pad048[0xB0 - 0x48];
        PointerVectorView indicators;
        std::uint8_t _pad0C8[0x130 - 0xC8];
        PointerVectorView childContexts;
    };
    static_assert(offsetof(CcCockpitContextLayout, movingObjectRaw) == 0x10);
    static_assert(offsetof(CcCockpitContextLayout, carrierLinkBaseStorage) == 0x18);
    static_assert(offsetof(CcCockpitContextLayout, carrierLinkHostRaw) == 0x28);
    static_assert(offsetof(CcCockpitContextLayout, devicesKeeperStorage) == 0x30);
    static_assert(offsetof(CcCockpitContextLayout, indicators) == 0xB0);
    static_assert(offsetof(CcCockpitContextLayout, childContexts) == 0x130);

    struct CcIndicatorLayout
    {
        std::uint8_t _pad000[0x38];
        void* controller;
        std::uint8_t _pad040[0x3A0 - 0x40];
        std::uint32_t screenCondition;
        std::uint8_t _pad3A4[0x46C - 0x3A4];
        std::uint8_t indicatorType;
        std::uint8_t _pad46D[0x470 - 0x46D];
        std::uint32_t renderTargetId;
        std::uint8_t _pad474[0x48C - 0x474];
        std::uint32_t cameraCondition;
    };
    static_assert(offsetof(CcIndicatorLayout, controller) == 0x38);
    static_assert(offsetof(CcIndicatorLayout, screenCondition) == 0x3A0);
    static_assert(offsetof(CcIndicatorLayout, indicatorType) == 0x46C);
    static_assert(offsetof(CcIndicatorLayout, renderTargetId) == 0x470);
    static_assert(offsetof(CcIndicatorLayout, cameraCondition) == 0x48C);

    struct AvTVSensorLayout
    {
        void* primaryVftable;
        void* platformRoot;
        std::uint8_t _platformMotors[0xA8 - 0x10]; // avPlatform motors; NEVER avDevice fields
        WorldPoseMatrix platformWorldPose;
        WorldPoseMatrix platformInitialWorldPose;
        WorldPoseMatrix platformInitialLocalPose;
        double platformLastUpdateTime;  // render extrapolation timestamp
        std::uint8_t _pad230[0x260 - 0x230];
        std::uint8_t activeFlag;
        std::uint8_t _pad261[0x268 - 0x261];
        void* searchTimer;
        std::uint8_t _pad270[0x328 - 0x270];
        double cachedTrackedPoint[3];
        std::uint8_t trackMode;
        std::uint8_t _pad341[0x348 - 0x341];
        double baseFov;
        std::uint8_t _pad350[0x358 - 0x350];
        double currentZoom;
        double targetZoom;  // zoom motor target
        std::uint8_t _pad368[0x4C0 - 0x368];
        double aspectRatio;
        SensorFrameLayout currentFrame;
        std::uint8_t _pad4D8[0x610 - 0x4D8];
        std::uint8_t hatLock;
    };
    static_assert(offsetof(AvTVSensorLayout, platformRoot) == 0x08);
    static_assert(offsetof(AvTVSensorLayout, platformWorldPose) == 0xA8);
    static_assert(offsetof(AvTVSensorLayout, platformInitialWorldPose) == 0x128);
    static_assert(offsetof(AvTVSensorLayout, platformInitialLocalPose) == 0x1A8);
    static_assert(offsetof(AvTVSensorLayout, platformLastUpdateTime) == 0x228);
    static_assert(offsetof(AvTVSensorLayout, activeFlag) == 0x260);
    static_assert(offsetof(AvTVSensorLayout, searchTimer) == 0x268);
    static_assert(offsetof(AvTVSensorLayout, cachedTrackedPoint) == 0x328);
    static_assert(offsetof(AvTVSensorLayout, trackMode) == 0x340);
    static_assert(offsetof(AvTVSensorLayout, baseFov) == 0x348);
    static_assert(offsetof(AvTVSensorLayout, currentZoom) == 0x358);
    static_assert(offsetof(AvTVSensorLayout, targetZoom) == 0x360);
    static_assert(offsetof(AvTVSensorLayout, aspectRatio) == 0x4C0);
    static_assert(offsetof(AvTVSensorLayout, currentFrame) == 0x4C8);
    static_assert(offsetof(AvTVSensorLayout, hatLock) == 0x610);

    struct AvPlatformLayout
    {
        void* primaryVftable;
        void* selfOrRoot;
        std::uint8_t _pad010[0xA8 - 0x10];
        WorldPoseMatrix platformWorldPose;
        WorldPoseMatrix platformInitialWorldPose;
        WorldPoseMatrix platformInitialLocalPose;
    };
    static_assert(offsetof(AvPlatformLayout, selfOrRoot) == 0x08);
    static_assert(offsetof(AvPlatformLayout, platformWorldPose) == 0xA8);
    static_assert(offsetof(AvPlatformLayout, platformInitialWorldPose) == 0x128);
    static_assert(offsetof(AvPlatformLayout, platformInitialLocalPose) == 0x1A8);
}
