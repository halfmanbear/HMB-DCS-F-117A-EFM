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

Vec3 common_force, common_moment, center_of_gravity, inertia, wind_world_cs;
namespace F117 {
double weight_N=0, internal_fuel=0, fuel_mass_in_dcs=0;
double altitude_m=0, ambientPressure_PA=0, ambientDensity_KgPerM3=0, ambientTemperature_DegK=0;
bool altitude_valid=false;
int alt_hold=0,horiz_hold=0,altroll_hold=0;
}
using namespace F117;
struct Throttle { int disconnects=0; void disconnect_button(){++disconnects;} } g_autothrottle;
struct Recovery { int disconnects=0; void disengage(){++disconnects;} } g_paars;
bool g_paarsEngagedAutothrottle=true;
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
    static_assert(JoystickPitch==2001 && JoystickRoll==2002 && JoystickYaw==2003 && throttleAxis==2004, "axes");
    static_assert(bombay==10100 && weaponReleaseHoldOn==10103 && weaponReleaseHoldOff==10104, "delivery");
    static_assert(bombaySelected==10130 && bombayLeft==10131 && bombayRight==10132, "individual bays");
    static_assert(IRADSZoomIn==10008 && IRADSZoomOut==10009 && IRADSZoomAxis==10121, "IRADS");
    static_assert(AutothrottleEngage==10125 && NwsAuthorityOff==10129, "automatic controls");

    ed_fm_set_current_mass_state(20000,1,2,3,40000,260000,225000);
    assert(center_of_gravity.x==1 && center_of_gravity.y==2 && center_of_gravity.z==3);
    assert(inertia.x==40000 && inertia.y==260000 && inertia.z==225000);
    assert(weight_N==20000*9.98665002864);
    add_local_force({10,20,30});add_local_force({-1,2,-3});
    add_local_moment({4,5,6});add_local_moment({1,-2,3});
    double x,y,z,px,py,pz;
    ed_fm_add_local_force(x,y,z,px,py,pz);
    assert(x==9 && y==22 && z==27 && px==1 && py==2 && pz==3);
    ed_fm_add_local_moment(x,y,z);assert(x==5 && y==3 && z==9);
    ed_fm_add_global_force(x,y,z,px,py,pz);assert(x==0&&y==0&&z==0&&px==0&&py==0&&pz==0);
    x=y=z=99;ed_fm_add_global_moment(x,y,z);assert(x==0&&y==0&&z==0);

    ed_fm_set_atmosphere(1000,280,335,1.1,90000,10,-2,3);
    assert(altitude_valid&&altitude_m==1000&&ambientTemperature_DegK==280&&ambientDensity_KgPerM3==1.1);
    assert(ambientPressure_PA==90000&&wind_world_cs.x==10&&wind_world_cs.y==-2&&wind_world_cs.z==3);
    ed_fm_set_atmosphere(std::numeric_limits<double>::quiet_NaN(),280,335,1.1,90000,0,0,0);
    assert(!altitude_valid);
    double air[2];ATMOS::atmos(288.15,1.225,200,air);
    assert(std::abs(air[0]-24500)<1e-9&&std::abs(air[1]-200/std::sqrt(1.4*287.058*288.15))<1e-12);

    ed_fm_set_internal_fuel(123);assert(ed_fm_get_internal_fuel()==123);
    ed_fm_set_external_fuel(1,50,1,2,3);assert(ed_fm_get_external_fuel()==0);
    assert(ed_fm_get_internal_fuel()==123);
    assert(ed_fm_change_mass(x,px,py,pz,y,z,parameter)&&x==123&&py==0.1);
    assert(!ed_fm_change_mass(x,px,py,pz,y,z,parameter)&&x==0);
    ed_fm_refueling_add_fuel(10);assert(ed_fm_get_internal_fuel()==133);
    assert(ed_fm_change_mass(x,px,py,pz,y,z,parameter)&&x==10);

    assert(!handle_autopilot_command(-1));
    assert(handle_autopilot_command(autopilot_alt)&&alt_hold==1&&!horiz_hold&&!altroll_hold);
    assert(handle_autopilot_command(autopilot_alt)&&!alt_hold);
    assert(handle_autopilot_command(autopilot_horiz)&&horiz_hold==1&&!alt_hold);
    assert(handle_autopilot_command(autopilot_alt_roll)&&altroll_hold==1&&!horiz_hold);
    assert(handle_autopilot_command(autopilot_reset)&&!alt_hold&&!horiz_hold&&!altroll_hold);
    assert(g_autothrottle.disconnects==1&&g_paars.disconnects==1&&!g_paarsEngagedAutothrottle);

    param_stuff options;
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
    puts("PASS: ED force/moment, atmosphere, mass/fuel adapters, command IDs, exclusive hold toggles, option polarity, cockpit API guards and owned strings");
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
