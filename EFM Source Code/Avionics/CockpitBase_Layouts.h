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
        std::uint32_t objectId;              // +0x00
        std::uint32_t _pad04;                // +0x04
        double relativeAzimuth;              // +0x08
        double relativeElevation;            // +0x10
        double relativeRange;                // +0x18
        double localTrackOffsetX;            // +0x20
        double localTrackOffsetY;            // +0x28
        double localTrackOffsetZ;            // +0x30
        SensorFrameLayout candidateFrame;    // +0x38
        std::uint8_t hasLosIntersection;     // +0x48
        std::uint8_t preciseTrackCandidate;  // +0x49
        std::uint8_t _pad4A[6];              // +0x4A
    };
    static_assert(sizeof(SearchItemLayout) == 0x50);

    struct AvTrackDataLayout
    {
        std::uint32_t objectId;              // +0x00
        std::uint32_t _pad04;                // +0x04
        double relativeAzimuth;              // +0x08
        double relativeElevation;            // +0x10
        double relativeRange;                // +0x18
        double worldTransform[16];           // +0x20 .. +0x9F
        double velocity[3];                  // +0xA0 .. +0xB7
        std::uint8_t _pad0B8[0xD0 - 0xB8];    // +0xB8 .. +0xCF
        double lastUpdateModelTime;          // +0xD0
        std::uint8_t _pad0D8[0xE8 - 0xD8];    // +0xD8 .. +0xE7
        void* movingObjectOrType;            // +0xE8
        double approxSize;                   // +0xF0
        double localPointOffset[3];          // +0xF8 .. +0x10F
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
        void* movingObjectRaw;               // +0x10 — IwHumanPlane/MovingObject* (from init_unit *(this+2))
        union
        {
            std::uint8_t carrierLinkBaseStorage[0x18]; // +0x18 — LinkBase copied by init_unit
            struct
            {
                std::uint8_t _pad018[0x10];
                void* carrierLinkHostRaw;    // +0x28 — LinkBase host pointer (MovingObject+0x100)
            };
        };
        std::uint8_t devicesKeeperStorage[0x18]; // +0x30
        std::uint8_t _pad048[0xB0 - 0x48];
        PointerVectorView indicators;        // +0xB0
        std::uint8_t _pad0C8[0x130 - 0xC8];
        PointerVectorView childContexts;     // +0x130
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
        void* controller;                    // +0x38
        std::uint8_t _pad040[0x3A0 - 0x40];
        std::uint32_t screenCondition;       // +0x3A0
        std::uint8_t _pad3A4[0x46C - 0x3A4];
        std::uint8_t indicatorType;          // +0x46C
        std::uint8_t _pad46D[0x470 - 0x46D];
        std::uint32_t renderTargetId;        // +0x470
        std::uint8_t _pad474[0x48C - 0x474];
        std::uint32_t cameraCondition;       // +0x48C
    };
    static_assert(offsetof(CcIndicatorLayout, controller) == 0x38);
    static_assert(offsetof(CcIndicatorLayout, screenCondition) == 0x3A0);
    static_assert(offsetof(CcIndicatorLayout, indicatorType) == 0x46C);
    static_assert(offsetof(CcIndicatorLayout, renderTargetId) == 0x470);
    static_assert(offsetof(CcIndicatorLayout, cameraCondition) == 0x48C);

    struct AvTVSensorLayout
    {
        void* primaryVftable;                // +0x00
        void* platformRoot;                  // +0x08
        std::uint8_t _pad010[0x29 - 0x10];
        std::uint8_t deviceSlotIndex;        // +0x29
        std::uint8_t _pad02A[0x50 - 0x2A];
        std::uintptr_t luaStateAlias;        // +0x50
        std::uintptr_t subsystemAlias;       // +0x58
        std::uint8_t _pad060[0xA8 - 0x60];
        WorldPoseMatrix platformWorldPose;   // +0xA8
        WorldPoseMatrix platformInitialWorldPose; // +0x128
        WorldPoseMatrix platformInitialLocalPose; // +0x1A8
        std::uint8_t _pad228[0x260 - 0x228];
        std::uint8_t activeFlag;             // +0x260
        std::uint8_t _pad261[0x268 - 0x261];
        void* searchTimer;                   // +0x268
        std::uint8_t _pad270[0x328 - 0x270];
        double cachedTrackedPoint[3];        // +0x328 .. +0x33F
        std::uint8_t trackMode;              // +0x340
        std::uint8_t _pad341[0x348 - 0x341];
        double baseFov;                      // +0x348
        std::uint8_t _pad350[0x358 - 0x350];
        double currentZoom;                  // +0x358
        std::uint8_t _pad360[0x4C0 - 0x360];
        double aspectRatio;                  // +0x4C0
        SensorFrameLayout currentFrame;      // +0x4C8
        std::uint8_t _pad4D8[0x610 - 0x4D8];
        std::uint8_t hatLock;                // +0x610
    };
    static_assert(offsetof(AvTVSensorLayout, platformRoot) == 0x08);
    static_assert(offsetof(AvTVSensorLayout, deviceSlotIndex) == 0x29);
    static_assert(offsetof(AvTVSensorLayout, luaStateAlias) == 0x50);
    static_assert(offsetof(AvTVSensorLayout, subsystemAlias) == 0x58);
    static_assert(offsetof(AvTVSensorLayout, platformWorldPose) == 0xA8);
    static_assert(offsetof(AvTVSensorLayout, platformInitialWorldPose) == 0x128);
    static_assert(offsetof(AvTVSensorLayout, platformInitialLocalPose) == 0x1A8);
    static_assert(offsetof(AvTVSensorLayout, activeFlag) == 0x260);
    static_assert(offsetof(AvTVSensorLayout, searchTimer) == 0x268);
    static_assert(offsetof(AvTVSensorLayout, cachedTrackedPoint) == 0x328);
    static_assert(offsetof(AvTVSensorLayout, trackMode) == 0x340);
    static_assert(offsetof(AvTVSensorLayout, baseFov) == 0x348);
    static_assert(offsetof(AvTVSensorLayout, currentZoom) == 0x358);
    static_assert(offsetof(AvTVSensorLayout, aspectRatio) == 0x4C0);
    static_assert(offsetof(AvTVSensorLayout, currentFrame) == 0x4C8);
    static_assert(offsetof(AvTVSensorLayout, hatLock) == 0x610);

    struct AvPlatformLayout
    {
        void* primaryVftable;                // +0x00
        void* selfOrRoot;                    // +0x08
        std::uint8_t _pad010[0xA8 - 0x10];
        WorldPoseMatrix platformWorldPose;   // +0xA8
        WorldPoseMatrix platformInitialWorldPose; // +0x128
        WorldPoseMatrix platformInitialLocalPose; // +0x1A8
    };
    static_assert(offsetof(AvPlatformLayout, selfOrRoot) == 0x08);
    static_assert(offsetof(AvPlatformLayout, platformWorldPose) == 0xA8);
    static_assert(offsetof(AvPlatformLayout, platformInitialWorldPose) == 0x128);
    static_assert(offsetof(AvPlatformLayout, platformInitialLocalPose) == 0x1A8);
}
