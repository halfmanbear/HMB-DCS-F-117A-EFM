#pragma once

// Wire IDs are persisted in DCS bindings. Keep them stable when adding commands.
// Custom IDs follow Cockpit/Scripts/command_defs.lua in the aircraft mod.
// These integer constants are also consumed by CockpitBase_Interop.cpp.

// Flight axes.
constexpr int joystick_pitch = 2001;
constexpr int joystick_roll = 2002;
constexpr int joystick_yaw = 2003;
constexpr int joystick_throttle = 2004;
constexpr int throttle_axis_left = 2005;
constexpr int throttle_axis_right = 2006;

// Discrete stick and pedal inputs.
constexpr int pitch_down = 193;
constexpr int pitch_down_stop = 194;
constexpr int pitch_up = 195;
constexpr int pitch_up_stop = 196;
constexpr int roll_left = 197;
constexpr int roll_left_stop = 198;
constexpr int roll_right = 199;
constexpr int roll_right_stop = 200;
constexpr int rudderleft = 201;
constexpr int rudderleftend = 202;
constexpr int rudderright = 203;
constexpr int rudderrightend = 204;

// Trim and flight control override.
constexpr int trim_left = 93;
constexpr int trim_right = 94;
constexpr int trim_up = 95;
constexpr int trim_down = 96;
constexpr int reset_trim = 97;
constexpr int ruddertrim_left = 98;
constexpr int ruddertrim_right = 99;
constexpr int fbw_override = 121;

// Engine switches and throttle steps.
constexpr int engines_on = 309;
constexpr int engines_off = 310;
constexpr int left_engine_on = 311;
constexpr int right_engine_on = 312;
constexpr int left_engine_off = 313;
constexpr int right_engine_off = 314;
constexpr int throttleup_left = 161;
constexpr int throttledown_left = 162;
constexpr int throttleup_right = 163;
constexpr int throttledown_right = 164;
constexpr int throttle_increase = 1032;
constexpr int throttle_decrease = 1033;
constexpr int throttle_stop = 1034;

// Airframe mechanisms.
constexpr int geartoggle = 68;
constexpr int gearup = 430;
constexpr int geardown = 431;
constexpr int wheel_brake_on = 74;
constexpr int wheel_brake_off = 75;
constexpr int tailhook = 69;
constexpr int air_refuel = 155;
constexpr int drag_chute = 76;

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
constexpr int tv_sensor_zoom_in = 103;
constexpr int tv_sensor_zoom_out = 104;

// F-117 bay and release protocol.
constexpr int pickle_on = 350;
constexpr int pickle_off = 351;
constexpr int bombay = 10100;
constexpr int bombay_open = 10101;
constexpr int bombay_close = 10102;
constexpr int bombay_selected = 10130;  // toggle the bay of the selected store
constexpr int bombay_left = 10131;      // toggle the left bay (station 1)
constexpr int bombay_right = 10132;     // toggle the right bay (station 2)
constexpr int weapon_release_hold_on = 10103;
constexpr int weapon_release_hold_off = 10104;
constexpr int master_arm_toggle = 10122;
constexpr int lua_damage_hit = 10200;

// IRADS protocol (Keys, hotas_commands and f117_commands).
constexpr int irads_zoom_in = 10008;
constexpr int irads_zoom_out = 10009;
constexpr int irads_slew_left = 10110;
constexpr int irads_slew_right = 10111;
constexpr int irads_slew_up = 10112;
constexpr int irads_slew_down = 10113;
constexpr int irads_slew_stop = 10114;
constexpr int irads_designation_clear = 10117;
constexpr int irads_laser_arm_toggle = 10118;
constexpr int irads_slew_horizontal_axis = 10119;
constexpr int irads_slew_vertical_axis = 10120;
constexpr int irads_zoom_axis = 10121;

// F-117 automatic controls and steering.
constexpr int autothrottle_engage = 10125;
constexpr int autothrottle_disconnect = 10126;
constexpr int paars_toggle = 10127;
constexpr int nws_authority_on = 10128;
constexpr int nws_authority_off = 10129;

// Alternate name retained for callers using the common throttle axis.
constexpr int throttle_axis = joystick_throttle;
