"""Exercise the production ED adapters, command protocol and cockpit wrapper.

Usage: python tests/efm_adapters_regression.py <build-directory>
Requires the Visual Studio 2022 C++ tools.
"""
from pathlib import Path
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


code = r'''
#define NOMINMAX
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include "EfmMath.h"
#include "Inputs.h"
#include "Atmosphere/Atmosphere.h"
#include "param_functions.h"
#include "include/Cockpit/CockpitAPI_Declare.h"
#include "Systems/FuelTanks.h"
#include "Diagnostics/MassLog.h"

void mass_log_state(double,double,double,double,double,double,double){}
void mass_log_piece(int,double,double,double,double,double,double,double){}
void mass_log_event(const char*,double){}
void mass_log_release(double){}
Vec3 common_force, common_moment, center_of_gravity, inertia, wind_world_cs;
namespace F117 {
double weight_n=0, internal_fuel=0, fuel_in_dcs_kg[Fuel_tanks::tank_count]={};
double altitude_m=0, ambient_pressure_pa=0, ambient_density_kg_per_m3=0, ambient_temperature_deg_k=0;
bool altitude_valid=false;
int alt_hold=0,horiz_hold=0,altroll_hold=0;
}
using namespace F117;
struct Throttle { int disconnects=0; void disconnect_button(){++disconnects;} } g_autothrottle;
struct Recovery { int disconnects=0; void disengage(){++disconnects;} } g_paars;
bool g_paars_engaged_autothrottle=true;
'''
for name in ['void add_local_force(', 'void add_local_moment(', 'void ed_fm_add_local_force(',
             'void ed_fm_add_local_moment(', 'void ed_fm_add_global_force(', 'void ed_fm_add_global_moment(',
             'void ed_fm_set_atmosphere(', 'void ed_fm_set_current_mass_state(',
             'bool handle_autopilot_command(', 'bool ed_fm_change_mass(',
             'void ed_fm_set_internal_fuel(', 'double ed_fm_get_internal_fuel(',
             'void ed_fm_set_external_fuel(', 'double ed_fm_get_external_fuel(',
             'void ed_fm_refueling_add_fuel(']:
    code += '\n' + extract(name) + '\n'
code += r'''
static double parameter=0;
static const char* text="alpha";
static void* lookup(const char*) {return &parameter;}
static void writeNumber(void*,double n){parameter=n;}
static bool readNumber(const void*,double& n,bool){n=parameter;return true;}
static void writeString(void*,const char* s){text=s;}
static bool readString(const void*,char* buffer,unsigned capacity){
    const size_t length=std::strlen(text);
    const size_t count=length<capacity?length:capacity;
    std::memcpy(buffer,text,count);
    if(count<capacity)buffer[count]='\0';
    return true;
}
int main() {
    // Command values form a wire contract with saved input bindings.
    static_assert(joystick_pitch==2001 && joystick_roll==2002 && joystick_yaw==2003 && throttle_axis==2004, "axes");
    static_assert(bombay==10100 && weapon_release_hold_on==10103 && weapon_release_hold_off==10104, "delivery");
    static_assert(bombay_selected==10130 && bombay_left==10131 && bombay_right==10132, "individual bays");
    static_assert(irads_zoom_in==10008 && irads_zoom_out==10009 && irads_zoom_axis==10121, "IRADS");
    static_assert(autothrottle_engage==10125 && nws_authority_off==10129, "automatic controls");

    ed_fm_set_current_mass_state(20000,1,2,3,40000,260000,225000);
    assert(center_of_gravity.x==1 && center_of_gravity.y==2 && center_of_gravity.z==3);
    assert(inertia.x==40000 && inertia.y==260000 && inertia.z==225000);
    assert(weight_n==20000*9.98665002864);
    add_local_force({10,20,30});add_local_force({-1,2,-3});
    add_local_moment({4,5,6});add_local_moment({1,-2,3});
    double x,y,z,px,py,pz;
    ed_fm_add_local_force(x,y,z,px,py,pz);
    assert(x==9 && y==22 && z==27 && px==1 && py==2 && pz==3);
    ed_fm_add_local_moment(x,y,z);assert(x==5 && y==3 && z==9);
    ed_fm_add_global_force(x,y,z,px,py,pz);assert(x==0&&y==0&&z==0&&px==0&&py==0&&pz==0);
    x=y=z=99;ed_fm_add_global_moment(x,y,z);assert(x==0&&y==0&&z==0);

    ed_fm_set_atmosphere(1000,280,335,1.1,90000,10,-2,3);
    assert(altitude_valid&&altitude_m==1000&&ambient_temperature_deg_k==280&&ambient_density_kg_per_m3==1.1);
    assert(ambient_pressure_pa==90000&&wind_world_cs.x==10&&wind_world_cs.y==-2&&wind_world_cs.z==3);
    ed_fm_set_atmosphere(std::numeric_limits<double>::quiet_NaN(),280,335,1.1,90000,0,0,0);
    assert(!altitude_valid);
    double air[2];Atmos::atmos(288.15,1.225,200,air);
    assert(std::abs(air[0]-24500)<1e-9&&std::abs(air[1]-200/std::sqrt(1.4*287.058*288.15))<1e-12);

    ed_fm_set_internal_fuel(123);assert(ed_fm_get_internal_fuel()==123);
    ed_fm_set_external_fuel(1,50,1,2,3);assert(ed_fm_get_external_fuel()==0);
    assert(ed_fm_get_internal_fuel()==123);
    // A small load sits in the feed tanks: one piece per tank, at the tank, with its own inertia.
    {
        using namespace Fuel_tanks;
        double total=0;int pieces=0;
        while(ed_fm_change_mass(x,px,py,pz,y,z,parameter)){
            const Tank& t=tanks[left_feed+pieces];
            assert(px==t.x&&py==t.y&&pz==t.z&&std::abs(x-61.5)<1e-9);
            assert(std::abs(y-x*own_roll_inertia_per_kg(t))<1e-9&&std::abs(parameter-x*own_pitch_inertia_per_kg(t))<1e-9);
            total+=x;++pieces;
        }
        assert(pieces==2&&std::abs(total-123)<1e-9&&x==0&&px==0&&y==0);
        ed_fm_refueling_add_fuel(10);assert(ed_fm_get_internal_fuel()==133);
        total=0;pieces=0;while(ed_fm_change_mass(x,px,py,pz,y,z,parameter)){total+=x;++pieces;}
        assert(pieces==2&&std::abs(total-10)<1e-9);
    }

    assert(!handle_autopilot_command(-1));
    assert(handle_autopilot_command(autopilot_alt)&&alt_hold==1&&!horiz_hold&&!altroll_hold);
    assert(handle_autopilot_command(autopilot_alt)&&!alt_hold);
    assert(handle_autopilot_command(autopilot_horiz)&&horiz_hold==1&&!alt_hold);
    assert(handle_autopilot_command(autopilot_alt_roll)&&altroll_hold==1&&!horiz_hold);
    assert(handle_autopilot_command(autopilot_reset)&&!alt_hold&&!horiz_hold&&!altroll_hold);
    assert(g_autothrottle.disconnects==1&&g_paars.disconnects==1&&!g_paars_engaged_autothrottle);

    Param_stuff options;
    assert(options.fuelvalue&&options.invincible_value);
    options.set_unlimited_fuel(true);options.set_immortal(true);
    assert(!options.fuelvalue&&!options.invincible_value);
    options.set_unlimited_fuel(false);options.set_immortal(false);
    assert(options.fuelvalue&&options.invincible_value);

    EDPARAM api;api.ed_param_api={lookup,writeString,writeNumber,readNumber,readString};
    void* handle=api.getParamHandle("test");assert(handle==&parameter);
    api.setParamNumber(handle,42);assert(api.getParamNumber(handle)==42);
    const auto first=api.getParamString(handle,16);assert(first=="alpha");
    api.setParamString(handle,"beta");assert(first=="alpha"&&api.getParamString(handle,16)=="beta");
    assert(api.getParamString(handle,3)=="be"&&api.getParamString(handle,0).empty());
    api.ed_param_api={};
    assert(api.getParamHandle("test")==nullptr&&api.getParamNumber(handle)==0&&api.getParamString(handle,8).empty());
    api.setParamNumber(handle,1);api.setParamString(handle,"unused");
    puts("PASS: ED force/moment, atmosphere, mass/fuel adapters with per-tank pieces, command IDs, exclusive hold toggles, option polarity, cockpit API guards and owned strings");
}
'''
p = out / 'efm_adapters_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = (f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 /I"{src}" /I"{src / "include"}" '
           f'"{p}" /Fo"{out}/efm_adapters_tests.obj" /Fe"{out}/efm_adapters_tests.exe"')
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'efm_adapters_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
