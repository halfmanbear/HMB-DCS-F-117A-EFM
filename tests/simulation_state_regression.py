"""Exercise the production EFM mass, ground-effect, PAARS and lifecycle functions."""
from pathlib import Path
import re
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402

repo = Path(__file__).resolve().parents[1]
src = repo / 'EFM Source Code'
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = load_source()


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
#include "ENGINE_START_HEADER"
using std::max;
struct v {double x=0,y=0,z=0;};
v wind_world_cs;
double vx_world=0,vy_world=0,vz_world=0;
namespace F117 {
double fuel_mass_in_dcs=0,internal_fuel=100,thrust_n=0,mach=0;
Param_stuff param_class;
double surface_height_m=0,altitude_m=0,wing_span_m=13.21;
bool surface_height_valid=false,altitude_valid=false;
double ambient_temperature_deg_k=0,ambient_density_kg_per_m3=0,ambient_pressure_pa=0;
double pitch_angle=0,roll_angle=0,roll_rate_rps=0,pitch_rate_rps=0,gear_down=0,gear_command=0;
bool weight_on_wheels=false,wow_from_draw_args=false,air_refuel_door_open=false,suspension_wow[3]={};
double radians_to_degrees=180/3.141592653589793,total_velocity_mps=200;
bool sim_initialized=false;int alt_hold=0,horiz_hold=0,altroll_hold=0;
constexpr double nose_wheel_yaw_low_rad=0.1745;bool nws_authority_held=false;double nose_wheel_yaw_range_rad=0;
double delta_time=0,ped_input=0,throttle_input=0,elevator_deg=0,aileron_deg=0,rudder_deg=0;
double elevator_deg_commanded=0,aileron_deg_commanded=0,rudder_deg_commanded=0;
double throttle_side_input[2]={};
float elev_pos=0,rudder_pos=0;
double elevator_pct=0,aileron_pct=0,rudder_pct=0,throttle_state=0,rolling_friction=0,wheel_brake_command=0;
double pitch_trim=0,roll_trim=0,yaw_trim=0,tailhook_command=0,tailhook_pos=0,dragchute_command=0,dragchute=0;
constexpr int weapon_bay_count=2;float bay_cmd[weapon_bay_count]={},bay_state[weapon_bay_count]={};
namespace Engine {struct Engine_state {double n2=0,thrust_fraction_of_max=0;};}
Engine::Engine_state engines[2];
double engine_thrust_n[2]={},engine_fuel_flow_kg_s[2]={},engine_egt_c[2]={},engine_oil_psi[2]={};
namespace Flight_controls {
    double lat_stick_input=0,long_stick_input=0,long_stick_force=0;int resets=0;
    void reset_runtime_state(){++resets;}
    void set_recovery_nz_command(bool,double){}
}
}
using namespace F117;
bool engines_running(){return true;}
void set_throttle_levers(double p){throttle_side_input[0]=throttle_side_input[1]=throttle_input=p;}
Autothrottle::Controller g_autothrottle;
Paars::Controller g_paars;
Engine_start::Starts g_engine_starts;
bool g_paars_engaged_autothrottle=false;double g_paars_roll_rate_deg_s=0;
void release_paars(){g_paars.disengage();}
double publishedPullout=0;
void publish_named_param(void*&,const char* name,double value){if(!std::strcmp(name,"F117_ARS_MIN_ALT"))publishedPullout=value;}
void reset_damage_and_events(){}namespace Weapons{void reset_weapon_release_sequence(){}void reset_laser(){}}
namespace Cockpit_interop {void reset_irads_session(){}}
"""
for marker, path in [('ACTUATORS_HEADER', src/'Actuators/Actuators.h'),
                     ('PARAM_HEADER', src/'param_functions.h'),
                     ('AUTOTHROTTLE_HEADER', src/'Avionics/Autothrottle.h'),
                     ('PAARS_HEADER', src/'Avionics/Paars.h'),
                     ('ENGINE_START_HEADER', src/'Engine/EngineStart.h')]:
    code = code.replace(marker, path.as_posix())
for name in ['wheel_brake_off_friction','gear_command_down','gear_command_up',
             'ground_start_idle_n2','air_start_throttle','air_start_n2',
             'kg_per_lb','start_fuel_flow_kg_s','idle_fuel_flow_kg_s']:
    declaration = re.search(r'constexpr\s+\w+\s+'+name+r'\s*=\s*[^;]+;',source)
    assert declaration, name
    code += declaration.group()+'\n'
for name in ['void update_fuel_system(double dt)', 'bool ed_fm_change_mass(',
             'void ed_fm_unlimited_fuel(bool enabled)', 'double induced_drag_ground_effect()',
             'void ed_fm_set_surface(', 'void ed_fm_set_atmosphere(',
             'void update_paars()', 'void reset_session_controls()',
             'void apply_ground_start_state(bool engines_running)',
             'void apply_air_start_state()', 'void ed_fm_release(void)',
             'bool gear_retraction_blocked()', 'double clamp_gear_command_for_weight_on_wheels(',
             'void set_gear_command(', 'void update_weight_on_wheels()']:
    code += '\n'+extract(name)+'\n'
code += r"""
void check_mass(double expected) {
    double mass=99,x=99,y=99,z=99,ix=99,iy=99,iz=99;
    const bool changed=ed_fm_change_mass(mass,x,y,z,ix,iy,iz);
    assert(changed==(expected!=0));assert(std::fabs(mass-expected)<1e-10);
    assert(ix==0&&iy==0&&iz==0&&z==0);
    if(changed)assert(x==0&&y==.1);
}
void dirty_session() {
    alt_hold=altroll_hold=horiz_hold=1;air_refuel_door_open=true;
    pitch_trim=roll_trim=yaw_trim=3;surface_height_valid=altitude_valid=true;
    Actuators::drag_chute.position=1;Actuators::gear.position=.4;Actuators::weapon_bay[0].position=Actuators::weapon_bay[1].position=1;bay_cmd[0]=bay_cmd[1]=bay_state[0]=bay_state[1]=1;
    thrust_n=80000;elev_pos=rudder_pos=1;
    Flight_controls::long_stick_input=1;fuel_mass_in_dcs=3;
    g_autothrottle.engage(300,50,true,true);g_paars.engage(-30,0,false,false);
}
void check_clean_session() {
    assert(alt_hold==0&&altroll_hold==0&&horiz_hold==0&&!air_refuel_door_open);
    assert(pitch_trim==0&&roll_trim==0&&yaw_trim==0);
    assert(Actuators::drag_chute.position==0&&Actuators::weapon_bay[0].position==0&&Actuators::weapon_bay[1].position==0&&bay_cmd[0]==0&&bay_cmd[1]==0&&bay_state[0]==0&&bay_state[1]==0);
    assert(!Actuators::initialized&&!sim_initialized&&Flight_controls::long_stick_input==0);
    assert(!g_autothrottle.engaged()&&!g_paars.engaged());
    assert(!surface_height_valid&&!altitude_valid);
    assert(thrust_n==0&&elev_pos==0&&rudder_pos==0);
}
int main() {
    // Fuel is accumulated across multiple ticks, clamped to available quantity,
    // and collected once even when DCS's inertia does not match an FM constant.
    // Both engines running, sharing the thrust.
    g_engine_starts.reset(true);
    engine_thrust_n[0]=engine_thrust_n[1]=9.81*3600/0.9928/2;internal_fuel=100;
    check_mass(100);check_mass(0); // Loaded fuel is added to the DCS mass once.
    assert(param_class.fuelvalue);
    update_fuel_system(.25);update_fuel_system(.75);
    assert(std::fabs(internal_fuel-99)<1e-10);check_mass(-1);check_mass(0); // Burned fuel is removed.
    ed_fm_unlimited_fuel(true);update_fuel_system(10);assert(internal_fuel==99);check_mass(0);
    ed_fm_unlimited_fuel(false);internal_fuel=.1;update_fuel_system(2);
    assert(internal_fuel==0);check_mass(-99);update_fuel_system(2);check_mass(0);
    internal_fuel=10;update_fuel_system(-1);update_fuel_system(NAN);check_mass(10);assert(internal_fuel==10);
    engine_thrust_n[0]=engine_thrust_n[1]=0;update_fuel_system(1);assert(std::fabs(internal_fuel-9.8)<1e-10);check_mass(-.2);
    assert(std::fabs(engine_fuel_flow_kg_s[0]-.1)<1e-12&&std::fabs(engine_fuel_flow_kg_s[1]-.1)<1e-12);
    // During a start the lit engine burns the start flow (600 lb/hr) and an engine not lit burns nothing.
    g_engine_starts.reset(false);g_engine_starts.engine[1].phase=Engine_start::phase_accelerating;
    update_fuel_system(10);assert(std::fabs(internal_fuel-(9.8-600*.45359237/360))<1e-9);
    assert(engine_fuel_flow_kg_s[0]==0&&std::fabs(engine_fuel_flow_kg_s[1]*3600/.45359237-600)<1e-9);
    check_mass(-(9.8-internal_fuel));
    // Unlimited fuel still reports the flow, so the gauges read.
    const double before=internal_fuel;internal_fuel=10;check_mass(10-before);ed_fm_unlimited_fuel(true);g_engine_starts.reset(true);update_fuel_system(1);
    assert(internal_fuel==10&&engine_fuel_flow_kg_s[0]==.1);ed_fm_unlimited_fuel(false);

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
    altitude_m=3048;pitch_angle=-30;roll_angle=0;total_velocity_mps=200;
    vx_world=200*std::cos(30*Paars::deg_to_rad);vy_world=-100;vz_world=0;
    assert(g_paars.engage(-30,0,false,false));update_paars();
    const double expected=Paars::displayed_pullout_altitude_ft(
        Paars::minimum_pullout_altitude_ft(altitude_m*3.28084,200,-30,0));
    assert(publishedPullout==expected&&publishedPullout<10000);
    roll_angle=60;update_paars();
    assert(publishedPullout==Paars::displayed_pullout_altitude_ft(
        Paars::minimum_pullout_altitude_ft(altitude_m*3.28084,200,-30,60)));
    vy_world=0;roll_angle=0;update_paars();assert(publishedPullout==10000);

    // Weight on wheels is wheel contact only. Climbing out with the gear down, lift is below weight and the
    // vertical acceleration is small, and the gear must still come up.
    for(bool& wow:suspension_wow)wow=false;wow_from_draw_args=false;
    Actuators::gear.position=1;gear_down=1;gear_command=gear_command_down;
    update_weight_on_wheels();assert(!weight_on_wheels&&!gear_retraction_blocked());
    set_gear_command(gear_command_up);update_weight_on_wheels();assert(gear_command==gear_command_up);
    // Any wheel on the ground holds the handle down, from the suspension or the strut draw args.
    for(int wheel=0;wheel<3;++wheel){
        for(bool& wow:suspension_wow)wow=false;suspension_wow[wheel]=true;gear_command=gear_command_up;
        update_weight_on_wheels();assert(weight_on_wheels&&gear_command==gear_command_down);
        set_gear_command(gear_command_up);assert(gear_command==gear_command_down);
    }
    for(bool& wow:suspension_wow)wow=false;wow_from_draw_args=true;gear_command=gear_command_up;
    update_weight_on_wheels();assert(weight_on_wheels&&gear_command==gear_command_down);
    wow_from_draw_args=false;update_weight_on_wheels();assert(!weight_on_wheels);

    // Reused modules are safe on release and on every start path, independently
    // of whether a preceding release callback ran.
    dirty_session();ed_fm_release();check_clean_session();assert(fuel_mass_in_dcs==0);
    // DCS collects the loaded fuel before it calls the start functions, so they must not forget it.
    dirty_session();apply_ground_start_state(false);check_clean_session();assert(fuel_mass_in_dcs==3);
    assert(gear_down==1&&Actuators::gear.position==1&&weight_on_wheels&&!engineswitch&&engines[0].n2==0&&engines[1].n2==0);
    assert(g_engine_starts.engine[0].phase==Engine_start::phase_off&&g_engine_starts.engine[1].phase==Engine_start::phase_off);
    dirty_session();apply_ground_start_state(true);check_clean_session();
    assert(gear_down==1&&Actuators::gear.position==1&&engineswitch&&engines[0].n2==ground_start_idle_n2&&engines[1].n2==ground_start_idle_n2);
    assert(g_engine_starts.engine[0].running()&&g_engine_starts.engine[1].running());
    dirty_session();apply_air_start_state();check_clean_session();
    assert(gear_down==0&&Actuators::gear.position==0&&!weight_on_wheels&&engineswitch&&engines[0].n2==air_start_n2&&engines[1].n2==air_start_n2);
    assert(g_engine_starts.any_running()&&std::isnan(engine_egt_c[0]));
    assert(Flight_controls::resets==4);
    puts("PASS: production fuel accumulation/mass collection/unlimited/exhaustion, weight on wheels from wheel contact only, surface clearance, PAARS world descent and all session reset paths");
}
"""
# Startup helpers set this actual production flag; other engine behavior is not mocked into these checks.
code = code.replace('bool sim_initialized=false;', 'bool engineswitch=false;bool sim_initialized=false;')
p = out/'simulation_state_tests.cpp'
p.write_text(code,encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/simulation_state_tests.obj" /Fe"{out}/simulation_state_tests.exe"'
r = subprocess.run(command,shell=True,capture_output=True,text=True)
print(r.stdout,r.stderr);assert r.returncode==0
r = subprocess.run([str(out/'simulation_state_tests.exe')],capture_output=True,text=True)
print(r.stdout,r.stderr);assert r.returncode==0
