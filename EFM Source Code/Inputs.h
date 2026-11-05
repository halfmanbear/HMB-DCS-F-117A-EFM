#pragma once

// Wire IDs are persisted in DCS bindings. Keep them stable when adding commands.
// Custom IDs follow Cockpit/Scripts/command_defs.lua in the aircraft mod.
// These integer constants are also consumed by CockpitBase_Interop.cpp.

// Flight axes.
constexpr int JoystickPitch = 2001;
constexpr int JoystickRoll = 2002;
constexpr int JoystickYaw = 2003;
constexpr int JoystickThrottle = 2004;
constexpr int ThrottleAxisLeft = 2005;
constexpr int ThrottleAxisRight = 2006;

// Discrete stick and pedal inputs.
constexpr int PitchDown = 193;
constexpr int PitchDownStop = 194;
constexpr int PitchUp = 195;
constexpr int PitchUpStop = 196;
constexpr int RollLeft = 197;
constexpr int RollLeftStop = 198;
constexpr int RollRight = 199;
constexpr int RollRightStop = 200;
constexpr int rudderleft = 201;
constexpr int rudderleftend = 202;
constexpr int rudderright = 203;
constexpr int rudderrightend = 204;

// Trim and flight control override.
constexpr int trimLeft = 93;
constexpr int trimRight = 94;
constexpr int trimUp = 95;
constexpr int trimDown = 96;
constexpr int resetTrim = 97;
constexpr int ruddertrimLeft = 98;
constexpr int ruddertrimRight = 99;
constexpr int FBW_override = 121;

// Engine switches and throttle steps.
constexpr int EnginesOn = 309;
constexpr int EnginesOff = 310;
constexpr int LeftEngineOn = 311;
constexpr int RightEngineOn = 312;
constexpr int LeftEngineOff = 313;
constexpr int RightEngineOff = 314;
constexpr int throttleupLeft = 161;
constexpr int throttledownLeft = 162;
constexpr int throttleupRight = 163;
constexpr int throttledownRight = 164;
constexpr int ThrottleIncrease = 1032;
constexpr int ThrottleDecrease = 1033;
constexpr int ThrottleStop = 1034;

// Airframe mechanisms.
constexpr int geartoggle = 68;
constexpr int gearup = 430;
constexpr int geardown = 431;
constexpr int WheelBrakeOn = 74;
constexpr int WheelBrakeOff = 75;
constexpr int tailhook = 69;
constexpr int AirRefuel = 155;
constexpr int dragChute = 76;

// Stock avionics and hold modes.
constexpr int nav = 105;
constexpr int bvr = 106;
constexpr int vs = 107;
constexpr int bore = 108;
constexpr int helm = 109;
constexpr int fi0 = 110;
constexpr int a2g = 111;
constexpr int gun = 113;
constexpr int autopilot_alt = 59;
constexpr int autopilot_horiz = 62;
constexpr int autopilot_alt_roll = 387;
constexpr int autopilot_reset = 408;
constexpr int TVSensorZoomIn = 103;
constexpr int TVSensorZoomOut = 104;

// F-117 bay and release protocol.
constexpr int pickleOn = 350;
constexpr int pickleOff = 351;
constexpr int bombay = 10100;
constexpr int bombayOpen = 10101;
constexpr int bombayClose = 10102;
constexpr int weaponReleaseHoldOn = 10103;
constexpr int weaponReleaseHoldOff = 10104;
constexpr int MasterArmToggle = 10122;
constexpr int luaDamageHit = 10200;

// IRADS protocol (Keys, hotas_commands and f117_commands).
constexpr int IRADSZoomIn = 10008;
constexpr int IRADSZoomOut = 10009;
constexpr int IRADSSlewLeft = 10110;
constexpr int IRADSSlewRight = 10111;
constexpr int IRADSSlewUp = 10112;
constexpr int IRADSSlewDown = 10113;
constexpr int IRADSSlewStop = 10114;
constexpr int IRADSDesignationClear = 10117;
constexpr int IRADSLaserArmToggle = 10118;
constexpr int IRADSSlewHorizontalAxis = 10119;
constexpr int IRADSSlewVerticalAxis = 10120;
constexpr int IRADSZoomAxis = 10121;

// F-117 automatic controls and steering.
constexpr int AutothrottleEngage = 10125;
constexpr int AutothrottleDisconnect = 10126;
constexpr int PaarsToggle = 10127;
constexpr int NwsAuthorityOn = 10128;
constexpr int NwsAuthorityOff = 10129;

// Alternate name retained for callers using the common throttle axis.
constexpr int throttleAxis = JoystickThrottle;
