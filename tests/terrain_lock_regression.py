from pathlib import Path
import math, subprocess, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source, unit_body  # noqa: E402
from lupa.lua51 import LuaRuntime

repo=Path(__file__).resolve().parents[1]
mod=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
vm=LuaRuntime(unpack_returned_tuples=True)
vm.execute('''params={};writes={};calls=0;fail=false
get_param_handle=function(n) return {get=function()return params[n] or 0 end,
set=function(_,v) params[n]=v;table.insert(writes,n) end} end
require=function() return {GetHeight=function(x,z) calls=calls+1;if fail then error('terrain failure') end;return 342 end} end
log={INFO=1,write=function(...)end}''')
module=vm.execute((mod/'Cockpit/Scripts/Systems/IRADS_Terrain.lua').read_text(encoding='utf-8'))
trace=module['trace']
def hit(height,ray):
    result=trace(vm.table_from({'GetHeight':height}),vm.table_from(ray))
    return result if isinstance(result,tuple) else (result,None)
ray=[-294000,5000,756000,.6,-.4,-.7]
p,reason=hit(lambda x,z:342,ray)
assert p and abs(p[2]-342)<.001
# Horizontal coordinates must move along the original ray, not a Y-only clamp.
t=(342-ray[1])/ray[4]
assert abs(p[1]-(ray[0]+ray[3]*t))<.002 and abs(p[3]-(ray[2]+ray[5]*t))<.002
for heading in [-1,1]:
    ray2=[100,2000,300,heading,-.2,.1]
    p,_=hit(lambda x,z:100+.01*x,ray2)
    expected=(100+.01*ray2[0]-ray2[1])/(ray2[4]-.01*ray2[3])
    assert p and abs(p[1]-(ray2[0]+ray2[3]*expected))<.002
p,_=hit(lambda x,z:500 if x>=1000 else 0,[0,1000,0,1,-.2,0])
assert p and abs(p[1]-2500)<.002 # first ridge crossing, not later valley floor
assert not hit(lambda x,z:0,[0,1000,0,1,0,0])[0]
assert not hit(lambda x,z:0,[0,-10,0,1,-1,0])[0]
assert not hit(lambda x,z:0,[0,1000,0,float('nan'),-1,0])[0]
assert not hit(lambda x,z:0,[0,1000,0,0,0,0])[0]
params=vm.globals().params
for name,value in zip(['OX','OY','OZ','DX','DY','DZ'],ray):params['F117_TERRAIN_'+name]=value
params['F117_TERRAIN_REQUEST']=7
module['update']();assert params['F117_TERRAIN_HIT']==1 and params['F117_TERRAIN_ACK']==7
writes=vm.globals().writes
assert writes[len(writes)]=='F117_TERRAIN_ACK'
calls=vm.globals().calls;module['update']();assert vm.globals().calls==calls
vm.globals().fail=True;params['F117_TERRAIN_REQUEST']=8;module['update']()
assert params['F117_TERRAIN_ACK']==8 and params['F117_TERRAIN_HIT']==0
print('PASS Lua: real terrain trace, flat/sloped/ridge terrain, both headings, no-hit/invalid input; payload before ACK, duplicate suppression and failed query clears HIT')

lock_unit=unit_body('Avionics/Interop/LockAndTerrain.cpp',keep_namespace=False)
fm=load_source()
from lib.sources import extract
code=r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include "LAYOUT"
namespace Cockpit_interop {
Av_tv_sensor_layout sensor{};void* g_av_tv_sensor_instance=&sensor;
Av_tv_sensor_layout* as_sensor(void* p){return static_cast<Av_tv_sensor_layout*>(p);}
bool s_irads_power=true,s_irads_ray_valid=true,s_irads_terrain_pending=false,s_irads_lock_valid=false;
double s_irads_render_ray[6]={0,1000,0,1,-1,0},s_irads_request_ray[6]{},s_irads_terrain_wait=0;
unsigned s_irads_terrain_sequence=0;
double s_irads_lock_world[3]{},s_last_aircraft_pos[3]{},s_irads_ray_age=1e9;
constexpr double irads_ray_max_age=0.5;
void log_format(const char*,...){}
'''.replace('LAYOUT',(repo/'EFM Source Code/Avionics/CockpitBase_Layouts.h').as_posix())
code+=lock_unit+'\n'
code+=r'''
}
namespace F117 { struct API {
std::map<std::string,double> params;
void* getParamHandle(const char* name){return &params[name];}
double getParamNumber(void* p){return *static_cast<double*>(p);}
void setParamNumber(void* p,double v){*static_cast<double*>(p)=v;}
} cockpit_api; }
bool g_irads_lock_params_initialized=true;
void* g_irads_lock_valid_param=F117::cockpit_api.getParamHandle("VALID");
void* g_irads_lock_world_x_param=F117::cockpit_api.getParamHandle("X");
void* g_irads_lock_world_y_param=F117::cockpit_api.getParamHandle("Y");
void* g_irads_lock_world_z_param=F117::cockpit_api.getParamHandle("Z");
void* g_irads_lock_range_param=F117::cockpit_api.getParamHandle("RANGE");
void* g_irads_lock_azimuth_param=F117::cockpit_api.getParamHandle("AZ");
void* g_irads_lock_elevation_param=F117::cockpit_api.getParamHandle("EL");
void initialize_irads_lock_params(){}
'''
code+=extract(fm,'void publish_irads_lock_params()')
code+=r'''
int main(){using namespace Cockpit_interop;
auto& p=F117::cockpit_api.params;
assert(ground_stabilize_irads()&&!has_irads_lock());publish_irads_lock_params();
unsigned first=s_irads_terrain_sequence;assert(p["F117_TERRAIN_REQUEST"]==first&&p["VALID"]==0);
double point[3]={658,342,0};
assert(!complete_irads_terrain_request(first+1,true,point)&&s_irads_terrain_pending);
p["F117_TERRAIN_X"]=point[0];p["F117_TERRAIN_Y"]=point[1];p["F117_TERRAIN_Z"]=point[2];p["F117_TERRAIN_HIT"]=1;p["F117_TERRAIN_ACK"]=first;
publish_irads_lock_params();assert(has_irads_lock()&&p["VALID"]==1&&p["Y"]==342&&p["F117_TERRAIN_REQUEST"]==0);
assert(sensor.cached_tracked_point[0]==658);
clear_irads_lock();assert(!has_irads_lock());assert(ground_stabilize_irads());
publish_irads_lock_params();assert(p["VALID"]==0); // previous ACK must not commit a new request
unsigned second=s_irads_terrain_sequence;assert(second!=first);
clear_irads_lock();assert(!complete_irads_terrain_request(second,true,point));
assert(ground_stabilize_irads());assert(!complete_irads_terrain_request(s_irads_terrain_sequence,false,point)&&!has_irads_lock());
assert(ground_stabilize_irads());double bad[3]={658,1200,0};assert(!complete_irads_terrain_request(s_irads_terrain_sequence,true,bad));
assert(ground_stabilize_irads());bad[0]=NAN;assert(!complete_irads_terrain_request(s_irads_terrain_sequence,true,bad));
assert(ground_stabilize_irads());s_irads_power=false;assert(!complete_irads_terrain_request(s_irads_terrain_sequence,true,point));
s_irads_power=true;s_irads_ray_valid=false;assert(!ground_stabilize_irads());
puts("PASS C++ actual request/commit/publication: pending invalid, payload/ACK, stale/cancelled responses, no-hit, off-ray, nonfinite, power and no-camera guards");
}
'''
p=out/'terrain_lock_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/terrain_lock_tests.obj" /Fe"{out}/terrain_lock_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'terrain_lock_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
