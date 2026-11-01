"""Exercise the production EFM mass, ground-effect, PAARS and lifecycle functions."""
from pathlib import Path
import re
import subprocess
import sys

repo = Path(__file__).resolve().parents[1]
src = repo / 'EFM Source Code'
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = (src / 'F_117_NightHawk.cpp').read_text(encoding='utf-8-sig')


def extract(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


code = r"""
#define NOMINMAX
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include "ACTUATORS_HEADER"
#include "PARAM_HEADER"
#include "AUTOTHROTTLE_HEADER"
#include "PAARS_HEADER"
using std::max;
struct V {double x=0,y=0,z=0;};
V wind_world_cs;
double vx_world=0,vy_world=0,vz_world=0;
namespace F117 {
double fuel_consumption_since_last_time=0,internal_fuel=100,thrust_N=0;
param_stuff param_class;
double surface_height_m=0,altitude_m=0,wingSpan_M=13.21;
bool surface_height_valid=false,altitude_valid=false;
double ambientTemperature_DegK=0,ambientDensity_KgPerM3=0,ambientPressure_PA=0;
double pitch_angle=0,roll_angle=0,rollRate_RPS=0,pitchRate_RPS=0,gearDown=0,GearCommand=0;
bool weight_on_wheels=false,wow_from_draw_args=false,airRefuelDoorOpen=false,suspension_wow[3]={};
double radiansToDegrees=180/3.141592653589793,totalVelocity_MPS=200;
bool simInitialized=false;int alt_hold=0,horiz_hold=0,altroll_hold=0;
double DeltaTime=0,pedInput=0,throttleInput=0,elevator_DEG=0,aileron_DEG=0,rudder_DEG=0;
double elevator_DEG_commanded=0,aileron_DEG_commanded=0,rudder_DEG_commanded=0;
float elev_pos=0,rudder_pos=0;
double elevator_PCT=0,aileron_PCT=0,rudder_PCT=0,throttle_state=0,rolling_friction=0,WheelBrakeCommand=0;
double pitchTrim=0,rollTrim=0,yawTrim=0,tailhook_command=0,tailhook_pos=0,dragchute_command=0,dragchute=0;
double misc_cmd=0,misc_state=0,misc_cmdH=0,misc_stateH=0;
namespace ENGINE {double N2=0;}
namespace FLIGHTCONTROLS {
    double latStickInput=0,longStickInput=0,longStickForce=0;int resets=0;
    void reset_runtime_state(){++resets;}
    void set_recovery_nz_command(bool,double){}
}
}
using namespace F117;
Autothrottle::Controller g_autothrottle;
Paars::Controller g_paars;
bool g_paarsEngagedAutothrottle=false;double g_paarsRollRateDegS=0;
void release_paars(){g_paars.disengage();}
double publishedPullout=0;
void publish_named_param(void*&,const char* name,double value){if(!std::strcmp(name,"F117_ARS_MIN_ALT"))publishedPullout=value;}
void reset_damage_and_events(){}void reset_weapon_release_sequence(){}void reset_laser(){}
namespace CockpitInterop {void ResetIRADSSession(){}}
"""
for marker, path in [('ACTUATORS_HEADER', src/'Actuators/Actuators.h'),
                     ('PARAM_HEADER', src/'param_functions.h'),
                     ('AUTOTHROTTLE_HEADER', src/'Avionics/Autothrottle.h'),
                     ('PAARS_HEADER', src/'Avionics/Paars.h')]:
    code = code.replace(marker, path.as_posix())
for name in ['kWheelBrakeOffFriction','kGearCommandDown','kGearCommandUp',
             'kGroundStartIdleN2','kAirStartThrottle','kAirStartN2']:
    declaration = re.search(r'constexpr\s+\w+\s+'+name+r'\s*=\s*[^;]+;',source)
    assert declaration, name
    code += declaration.group()+'\n'
for name in ['void update_fuel_system(double dt)', 'bool ed_fm_change_mass(',
             'void ed_fm_unlimited_fuel(bool value)', 'double induced_drag_ground_effect()',
             'void ed_fm_set_surface(', 'void ed_fm_set_atmosphere(',
             '    void update_paars()', '    void reset_session_controls()',
             '    void apply_ground_start_state(bool enginesRunning)',
             '    void apply_air_start_state()', 'void ed_fm_release()']:
    code += '\n'+extract(name)+'\n'
code += r"""
void check_mass(double expected) {
    double mass=99,x=99,y=99,z=99,ix=99,iy=99,iz=99;
    const bool changed=ed_fm_change_mass(mass,x,y,z,ix,iy,iz);
    assert(changed==(expected>0));assert(std::fabs(mass-expected)<1e-10);
    assert(ix==0&&iy==0&&iz==0&&z==0);
    if(changed)assert(x==-1&&y==1);
}
void dirty_session() {
    alt_hold=altroll_hold=horiz_hold=1;airRefuelDoorOpen=true;
    pitchTrim=rollTrim=yawTrim=3;surface_height_valid=altitude_valid=true;
    ACTUATORS::dragchute_state=1;ACTUATORS::gear_state=.4;ACTUATORS::misc_posH=1;
    thrust_N=80000;elev_pos=rudder_pos=1;
    FLIGHTCONTROLS::longStickInput=1;fuel_consumption_since_last_time=3;
    g_autothrottle.engage(300,50,true,true);g_paars.engage(-30,0,false,false);
}
void check_clean_session() {
    assert(alt_hold==0&&altroll_hold==0&&horiz_hold==0&&!airRefuelDoorOpen);
    assert(pitchTrim==0&&rollTrim==0&&yawTrim==0);
    assert(ACTUATORS::dragchute_state==0&&ACTUATORS::misc_posH==0);
    assert(!ACTUATORS::simInitialized&&!simInitialized&&FLIGHTCONTROLS::longStickInput==0);
    assert(!g_autothrottle.engaged()&&!g_paars.engaged());
    assert(!surface_height_valid&&!altitude_valid&&fuel_consumption_since_last_time==0);
    assert(thrust_N==0&&elev_pos==0&&rudder_pos==0);
}
int main() {
    // Fuel is accumulated across multiple ticks, clamped to available quantity,
    // and collected once even when DCS's inertia does not match an FM constant.
    thrust_N=9.81*3600/1.24;internal_fuel=100;
    assert(param_class.fuelvalue);
    update_fuel_system(.25);update_fuel_system(.75);
    assert(std::fabs(internal_fuel-99)<1e-10);check_mass(1);check_mass(0);
    ed_fm_unlimited_fuel(true);update_fuel_system(10);assert(internal_fuel==99);check_mass(0);
    ed_fm_unlimited_fuel(false);internal_fuel=.1;update_fuel_system(2);
    assert(internal_fuel==0);check_mass(.1);update_fuel_system(2);check_mass(0);
    internal_fuel=10;update_fuel_system(-1);update_fuel_system(NAN);check_mass(0);assert(internal_fuel==10);

    // Unknown/invalid observations use free air; the same AGL produces the same
    // effect over different elevations, with object surfaces handled explicitly.
    assert(induced_drag_ground_effect()==1);
    ed_fm_set_surface(0,0,0,0,1,0);assert(induced_drag_ground_effect()==1);
    ed_fm_set_atmosphere(1,288,340,1.2,101325,0,0,0);
    const double sea=induced_drag_ground_effect();assert(sea>0&&sea<1);
    ed_fm_set_surface(1000,1000,0,0,1,0);
    ed_fm_set_atmosphere(1001,288,340,1.2,101325,0,0,0);
    assert(std::fabs(induced_drag_ground_effect()-sea)<1e-12);
    ed_fm_set_surface(0,50,0,0,1,0);altitude_m=51;
    assert(std::fabs(induced_drag_ground_effect()-sea)<1e-12);
    altitude_m=49;assert(induced_drag_ground_effect()==0);
    ed_fm_set_surface(0,0,0,0,1,0);altitude_m=3048;assert(induced_drag_ground_effect()>.999999);
    ed_fm_set_surface(NAN,0,0,0,1,0);assert(induced_drag_ground_effect()==1);

    // Pitched descents have zero body-axis vertical speed. The published
    // estimate must derive its flight-path angle from world vertical velocity.
    altitude_m=3048;pitch_angle=-30;roll_angle=0;totalVelocity_MPS=200;
    vx_world=200*std::cos(30*Paars::kDegToRad);vy_world=-100;vz_world=0;
    assert(g_paars.engage(-30,0,false,false));update_paars();
    const double expected=Paars::displayed_pullout_altitude_ft(
        Paars::minimum_pullout_altitude_ft(altitude_m*3.28084,200,-30,0));
    assert(publishedPullout==expected&&publishedPullout<10000);
    roll_angle=60;update_paars();
    assert(publishedPullout==Paars::displayed_pullout_altitude_ft(
        Paars::minimum_pullout_altitude_ft(altitude_m*3.28084,200,-30,60)));
    vy_world=0;roll_angle=0;update_paars();assert(publishedPullout==10000);

    // Reused modules are safe on release and on every start path, independently
    // of whether a preceding release callback ran.
    dirty_session();ed_fm_release();check_clean_session();
    dirty_session();apply_ground_start_state(false);check_clean_session();
    assert(gearDown==1&&ACTUATORS::gear_state==1&&weight_on_wheels&&!engineswitch&&ENGINE::N2==0);
    dirty_session();apply_ground_start_state(true);check_clean_session();
    assert(gearDown==1&&ACTUATORS::gear_state==1&&engineswitch&&ENGINE::N2==kGroundStartIdleN2);
    dirty_session();apply_air_start_state();check_clean_session();
    assert(gearDown==0&&ACTUATORS::gear_state==0&&!weight_on_wheels&&engineswitch&&ENGINE::N2==kAirStartN2);
    assert(FLIGHTCONTROLS::resets==4);
    puts("PASS: production fuel accumulation/mass collection/unlimited/exhaustion, surface clearance, PAARS world descent and all session reset paths");
}
"""
# Startup helpers set this actual production flag; other engine behavior is not mocked into these checks.
code = code.replace('bool simInitialized=false;', 'bool engineswitch=false;bool simInitialized=false;')
p = out/'simulation_state_tests.cpp'
p.write_text(code,encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/simulation_state_tests.obj" /Fe"{out}/simulation_state_tests.exe"'
r = subprocess.run(command,shell=True,capture_output=True,text=True)
print(r.stdout,r.stderr);assert r.returncode==0
r = subprocess.run([str(out/'simulation_state_tests.exe')],capture_output=True,text=True)
print(r.stdout,r.stderr);assert r.returncode==0
