from pathlib import Path
import subprocess,sys,math
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source, extract  # noqa: E402
from lupa.lua51 import LuaRuntime
repo=Path(__file__).resolve().parents[1];mod=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
vm=LuaRuntime();params={}
# Keep the mock handles inside Lua: repeated colon calls must not depend on
# Lupa's Python bound-method/userdata caching or garbage-collection behavior.
vm.execute("""
parameter_handles={}
function get_param_handle(name)
 if not parameter_handles[name] then
  parameter_handles[name]={value=0,get=function(self)return self.value end,
   set=function(self,value)self.value=value end}
 end
 return parameter_handles[name]
end
""")
class Param:
 def __init__(self,name):self.handle=vm.globals().get_param_handle(name)
 def get(self):return self.handle['value']
 def set(self,value):self.handle['value']=value
 v=property(get,set)

def param(name):
 if name not in params:params[name]=Param(name)
 return params[name]
vm.globals().LockOn_Options=vm.table_from({'script_path':str(mod/'Cockpit/Scripts').replace('\\','/')+'/'})
vm.globals().make_default_activity=lambda _:None
vm.execute('''log={INFO=1,write=function(...)end};stations={
 [0]={count=1,CLSID="{GBU-31V3B}"},[1]={count=1,CLSID="{GBU_32_V_2B}"},[2]={count=1,CLSID="UNKNOWN"},[3]={count=1,CLSID="GBU-12"}}
 GetSelf=function()return{listen_command=function(...)end,get_station_info=function(_,s)return stations[s]end}end''')
vm.execute((mod/'Cockpit/Scripts/Systems/Weapons.lua').read_text(encoding='utf-8'))
vm.globals().post_initialize();assert param('F117_RELEASE_STATION').v==0
# Actual accepted-release sequence drives stepping, even with delayed native inventory.
param('F117_RELEASE_SEQUENCE').set(1);vm.globals()['update']();assert param('F117_RELEASE_STATION').v==1 and param('F117_CCRP_PROFILE').v==2
for _ in range(20):vm.globals()['update']()
assert param('F117_RELEASE_STATION').v==1 # no step from held consent/status/repeated polling
param('F117_RELEASE_SEQUENCE').set(2);vm.globals()['update']();assert param('F117_RELEASE_STATION').v==3 and param('F117_CCRP_PROFILE').v==3
vm.execute('stations[0].count=0;stations[1].count=0;stations[3].count=0')
param('F117_RELEASE_SEQUENCE').set(3);vm.globals()['update']();assert param('F117_STORE_VALID').v==0
param('F117_RELEASE_SEQUENCE').set(0);vm.globals()['update']();assert param('F117_STORE_VALID').v==0
vm.execute('stations[3].count=1');vm.globals()['update']();assert param('F117_RELEASE_STATION').v==3
print('PASS: actual Weapons.lua auto-steps once per accepted release, skips unsupported/empty stations, tolerates inventory lag, handles depletion/reset/rearm')
# Validate the deployed pickle binding without depending on a personal saved profile.
keyboard=(mod/'Input/F-117-NightHawk/keyboard/default.lua').read_text(encoding='utf-8')
assert "combos = {{key = 'Space', reformers = {'RAlt'}}}, down = iCommandPlanePickleOn, up = iCommandPlanePickleOff" in keyboard
print('PASS: deployed generic pickle uses RAlt+Space with separate press/release commands')
source=load_source('Avionics')
code=r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
#include "LAYOUT"
using namespace Cockpit_interop;
Av_tv_sensor_layout sensor{};void* g_av_tv_sensor_instance=&sensor;
Av_tv_sensor_layout* as_sensor(void* p){return static_cast<Av_tv_sensor_layout*>(p);}
Cc_cockpit_context_layout* as_context(void* p){return static_cast<Cc_cockpit_context_layout*>(p);}
bool s_irads_power=true,s_irads_lock_valid=true;
bool s_irads_ray_valid=false,s_irads_terrain_pending=false;double s_irads_render_ray[6]{},s_irads_request_ray[6]{};unsigned s_irads_terrain_sequence=0;double s_irads_terrain_wait=0,s_irads_ray_age=1e9;
bool s_carrier_link_disabled_logged=false;
bool s_irads_homing=false;void service_irads_home(const double*){}
void* s_our_ind=nullptr;
double s_irads_lock_world[3]={10000,0,2000};float s_irads_horizontal_axis=0,s_irads_vertical_axis=0;
void log_message(const char*){}template<class...T>void log_format(const char*,T...){}
struct Aircraft_pose{double x,y,z,qx,qy,qz,qw;};
void seed_injected_sensor_pose(const Aircraft_pose& pose){double x=pose.x,y=pose.y,z=pose.z;
 double m[16]={1,0,0,0,0,1,0,0,0,0,1,0,x,y,z,1};std::memcpy(sensor.platform_world_pose.m,m,sizeof(m));}
void laggingNative(void*,double){sensor.platform_world_pose.m[0]=.99;sensor.platform_world_pose.m[1]=-.01;}
void laggingFrame(void*){sensor.platform_world_pose.m[2]=.01;}
void nothing(void*){}
auto g_av_tv_sensor_update=laggingNative;auto g_av_tv_sensor_update_frame=laggingFrame;
void uncage_sensor(){}
void* resolve_cockpit_base_export(const char*){return nullptr;}
constexpr size_t camera_eye_offset=0x260,camera_forward_offset=0x230;constexpr unsigned render_log_limit=120,render_log_interval=120;constexpr double rad_to_deg=57.295779513;
alignas(16) char camera[0x400]{};
void* originalCamera(void*){return &camera;}auto s_original_get_camera=originalCamera;
double renderedPose[16]{};
void fillCamera(void*,void*){
 std::memcpy(renderedPose,sensor.platform_world_pose.m,sizeof(renderedPose));
 // Native get_position uses time-lastUpdate>0 to rebuild the pose from motors.
 if (1000.0-sensor.platform_last_update_time>0.0) {renderedPose[0]=1;renderedPose[1]=0;renderedPose[2]=0;}
 float forward[3],eye[3];for(int i=0;i<3;++i){forward[i]=static_cast<float>(renderedPose[i]);eye[i]=static_cast<float>(renderedPose[12+i]);}
 std::memcpy(camera+0x230,forward,sizeof(forward));std::memcpy(camera+0x260,eye,sizeof(eye));
}auto s_sensor_fill_camera=fillCamera;
'''.replace('LAYOUT',str(repo/'EFM Source Code/Avionics/CockpitBase_Layouts.h').replace('\\','/'))
code+='\n'.join(extract(source,n) for n in ['bool aim_pose_at_world_point(','void update_irads_sensor_frame(','static void* __fastcall irads_get_camera('])
code+=r'''
void checkAim(const double* p,const double* t){
 double d[3],norm=0;for(int i=0;i<3;++i){d[i]=t[i]-p[12+i];norm+=d[i]*d[i];}norm=std::sqrt(norm);
 for(int i=0;i<3;++i)assert(std::fabs(p[i]-d[i]/norm)<1e-12);
 for(int a=0;a<3;++a)for(int b=0;b<3;++b){double dot=0;for(int i=0;i<3;++i)dot+=p[a*4+i]*p[b*4+i];assert(std::fabs(dot-(a==b?1:0))<1e-10);}
}
int main(){
 sensor.active_flag=1;sensor.current_zoom=80;
 for(int frame=0;frame<3600;++frame){
  double x=frame*3.0,y=3000+100*std::sin(frame*.01),z=300*std::cos(frame*.01);
  update_irads_sensor_frame(1.0/60,Aircraft_pose{x,y,z,0,0,0,1});
  checkAim(sensor.platform_world_pose.m,s_irads_lock_world);
  assert(sensor.current_zoom==80&&sensor.platform_world_pose.m[12]==x&&sensor.platform_world_pose.m[13]==y);
  // Another native update between EFM and rendering cannot reintroduce aim lag.
  laggingNative(&sensor,.01);sensor.platform_last_update_time=900.0;irads_get_camera(nullptr);checkAim(renderedPose,s_irads_lock_world);assert(sensor.platform_last_update_time==900.0);
 }
 double vertical[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};double t[3]={0,-1000,0};
 assert(aim_pose_at_world_point(vertical,t));checkAim(vertical,t);
 double before[16];std::memcpy(before,vertical,sizeof(before));t[0]=std::numeric_limits<double>::quiet_NaN();
 assert(!aim_pose_at_world_point(vertical,t)&&std::memcmp(before,vertical,sizeof(before))==0);
 s_irads_lock_valid=false;update_irads_sensor_frame(.01,Aircraft_pose{0,3000,0,0,0,0,1});
 assert(sensor.platform_world_pose.m[0]==.99); // free slew stays native
 s_irads_terrain_pending=true;s_irads_terrain_wait=.995;update_irads_sensor_frame(.01,Aircraft_pose{0,3000,0,0,0,0,1});assert(!s_irads_terrain_pending);
 puts("PASS: actual update/render lock holds fixed-world LOS over 3600 moving frames despite native lag; orthonormal pose, vertical target, invalid input, zoom/origin and unlocked native slew preserved");
}
'''
p=out/'point_track_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/point_track_tests.obj" /Fe"{out}/point_track_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'point_track_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
