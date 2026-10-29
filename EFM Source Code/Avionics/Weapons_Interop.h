#pragma once
// ============================================================================
// Weapons_Interop.h — IRADS target hand-off to DCS weapons
// ============================================================================
// Store launch goes straight to the aircraft's DCS weapon manager with the
// IRADS designated point as the launch target (JDAM coordinates / LGB initial
// aim point), instead of avSimpleWeaponSystem's default "10 km ahead" point.
// The laser uses the same native woSpot path as avRangefinder (Litening), so
// laser-guided bombs track it and it is network replicated.

namespace WeaponsInterop
{
    static constexpr unsigned int kDefaultLaserCode = 1688;

    // True once the DCS weapon manager is reachable from the cockpit context.
    bool IsLaunchAvailable();

    // Launch one store from the explicitly selected station at the given world point.
    // Never falls through to another station. Returns true if the
    // weapon manager accepted a launch.
    bool LaunchAtPoint(const double worldPoint[3], int station);

    // Create / move / remove the laser spot. Call every frame.
    void UpdateLaser(bool wantOn, const double worldPoint[3], unsigned int code);
    bool IsLaserFiring();

    // ed_fm_release: drop pointers. The spot is parented to the aircraft and
    // dies with it, so it is not killed here.
    void Reset();
}
