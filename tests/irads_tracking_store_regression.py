from pathlib import Path
import subprocess,sys,math
from lupa.lua51 import LuaRuntime
repo=Path(__file__).resolve().parents[1];mod=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
vm=LuaRuntime();params={}
class Param:
 def __init__(self):self.v=0
 def get(self):return self.v
 def set(self,v):self.v=v
 def __getitem__(self,k):return getattr(self,k)
def param(n):
 if n not in params:params[n]=Param()
 return params[n]
vm.globals().get_param_handle=param
vm.globals().LockOn_Options=vm.table_from({'script_path':str(mod/'Cockpit/Scripts').replace('\\','/')+'/'})
vm.globals().make_default_activity=lambda _:None
vm.execute('''log={INFO=1,write=function(...)end};stations={
 [0]={count=1,CLSID="GBU31_V3B_JDAM"},[1]={count=1,CLSID="GBU32_JDAM"},[2]={count=1,CLSID="UNKNOWN"},[3]={count=1,CLSID="GBU-12"}}
 GetSelf=function()return{listen_command=function(...)end,get_station_info=function(_,s)return stations[s]end}end''')
vm.execute((repo/'Cockpit/Scripts/Systems/Weapons.lua').read_text(encoding='utf-8'))
vm.globals().post_initialize();assert param('F117_RELEASE_STATION').v==0
# Actual accepted-release sequence drives stepping, even with delayed native inventory.
param('F117_RELEASE_SEQUENCE').set(1);vm.globals().update();assert param('F117_RELEASE_STATION').v==1 and param('F117_CCRP_PROFILE').v==2
for _ in range(20):vm.globals().update()
assert param('F117_RELEASE_STATION').v==1 # no step from held consent/status/repeated polling
param('F117_RELEASE_SEQUENCE').set(2);vm.globals().update();assert param('F117_RELEASE_STATION').v==3 and param('F117_CCRP_PROFILE').v==3
vm.execute('stations[0].count=0;stations[1].count=0;stations[3].count=0')
param('F117_RELEASE_SEQUENCE').set(3);vm.globals().update();assert param('F117_STORE_VALID').v==0
param('F117_RELEASE_SEQUENCE').set(0);vm.globals().update();assert param('F117_STORE_VALID').v==0
vm.execute('stations[3].count=1');vm.globals().update();assert param('F117_RELEASE_STATION').v==3
print('PASS: actual Weapons.lua auto-steps once per accepted release, skips unsupported/empty stations, tolerates inventory lag, handles depletion/reset/rearm')
# Confirm saved keyboard no longer overrides release to Space or removes gun Space.
saved=vm.execute((Path(r'E:\Saved Games\DCS\Config\Input\F-117-NightHawk/keyboard/Keyboard.diff.lua')).read_text(encoding='utf-8'))
assert saved.keyDiffs['d10103pnilu10104cdnilvdnilvpnilvunil'] is None
assert saved.keyDiffs['d84pnilu85cdnilvdnilvpnilvunil'] is None
keyboard=(mod/'Input/F-117-NightHawk/keyboard/default.lua').read_text(encoding='utf-8')
assert "combos = {{key = 'Space', reformers = {'RAlt'}}}, down = 350, up = 351" in keyboard
print('PASS: generic pickle 350/351 uses RAlt+Space; saved plain-Space release override and gun suppression removed')
source=(repo/'EFM Source Code/Avionics/CockpitBase_Interop.cpp').read_text(encoding='utf-8-sig')
def extract(name):
 start=source.index(name);brace=source.index('{',start);end=brace+1;depth=1
 while depth:depth+=(source[end]=='{')-(source[end]=='}');end+=1
 return source[start:end]
code=r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
#include "LAYOUT"
using namespace CockpitInterop;
AvTVSensorLayout sensor{};void* g_avTVSensor_instance=&sensor;
AvTVSensorLayout* AsSensor(void* p){return static_cast<AvTVSensorLayout*>(p);}
CcCockpitContextLayout* AsContext(void* p){return static_cast<CcCockpitContextLayout*>(p);}
bool s_irads_power=true,s_irads_lock_valid=true,s_irads_designating=false;
bool s_irads_ray_valid=false,s_irads_terrain_pending=false;double s_irads_render_ray[6]{},s_irads_request_ray[6]{};unsigned s_irads_terrain_sequence=0;double s_irads_terrain_wait=0;
bool s_carrier_link_disabled_logged=false,s_carrier_set=false;
void** g_contexts_ptr_addr=nullptr;void* s_our_ind=nullptr;
double s_irads_lock_world[3]={10000,0,2000};float s_irads_horizontal_axis=0,s_irads_vertical_axis=0;
void Log_(const char*){}template<class...T>void Log(const char*,T...){}
bool TryCaptureIRADSLock(){return false;}
void SeedInjectedSensorPose(double x,double y,double z,double,double,double,double){
 double m[16]={1,0,0,0,0,1,0,0,0,0,1,0,x,y,z,1};std::memcpy(sensor.platformWorldPose.m,m,sizeof(m));}
void laggingNative(void*,double){sensor.platformWorldPose.m[0]=.99;sensor.platformWorldPose.m[1]=-.01;}
void laggingFrame(void*){sensor.platformWorldPose.m[2]=.01;}
void nothing(void*){}
auto g_avTVSensor_update=laggingNative;auto g_avTVSensor_update_frame=laggingFrame;auto g_avTVSensor_uncage=nothing;
void (*g_avPlatform_set_carrier)(void*,void*)=nullptr;
void* GetModuleHandleA(const char*){return nullptr;}void* GetProcAddress(void*,const char*){return nullptr;}
alignas(16) char camera[0x400]{};
void* originalCamera(void*){return &camera;}auto s_original_get_camera=originalCamera;
double renderedPose[16]{};
void fillCamera(void*,void*){
 std::memcpy(renderedPose,sensor.platformWorldPose.m,sizeof(renderedPose));
 // Native get_position uses time-lastUpdate>0 to rebuild the pose from motors.
 if (1000.0-sensor.platformLastUpdateTime>0.0) {renderedPose[0]=1;renderedPose[1]=0;renderedPose[2]=0;}
 float forward[3],eye[3];for(int i=0;i<3;++i){forward[i]=static_cast<float>(renderedPose[i]);eye[i]=static_cast<float>(renderedPose[12+i]);}
 std::memcpy(camera+0x230,forward,sizeof(forward));std::memcpy(camera+0x260,eye,sizeof(eye));
}auto s_sensor_fill_camera=fillCamera;
'''.replace('LAYOUT',str(repo/'EFM Source Code/Avionics/CockpitBase_Layouts.h').replace('\\','/'))
code+='\n'.join(extract(n) for n in ['static bool AimPoseAtWorldPoint(','void UpdateIRADSSensorFrame(','static void* __fastcall IRADSGetCamera('])
code+=r'''
void checkAim(const double* p,const double* t){
 double d[3],norm=0;for(int i=0;i<3;++i){d[i]=t[i]-p[12+i];norm+=d[i]*d[i];}norm=std::sqrt(norm);
 for(int i=0;i<3;++i)assert(std::fabs(p[i]-d[i]/norm)<1e-12);
 for(int a=0;a<3;++a)for(int b=0;b<3;++b){double dot=0;for(int i=0;i<3;++i)dot+=p[a*4+i]*p[b*4+i];assert(std::fabs(dot-(a==b?1:0))<1e-10);}
}
int main(){
 sensor.activeFlag=1;sensor.currentZoom=80;
 for(int frame=0;frame<3600;++frame){
  double x=frame*3.0,y=3000+100*std::sin(frame*.01),z=300*std::cos(frame*.01);
  UpdateIRADSSensorFrame(1.0/60,x,y,z,0,0,0,1);
  checkAim(sensor.platformWorldPose.m,s_irads_lock_world);
  assert(sensor.currentZoom==80&&sensor.platformWorldPose.m[12]==x&&sensor.platformWorldPose.m[13]==y);
  // Another native update between EFM and rendering cannot reintroduce aim lag.
  laggingNative(&sensor,.01);sensor.platformLastUpdateTime=900.0;IRADSGetCamera(nullptr);checkAim(renderedPose,s_irads_lock_world);assert(sensor.platformLastUpdateTime==900.0);
 }
 double vertical[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};double t[3]={0,-1000,0};
 assert(AimPoseAtWorldPoint(vertical,t));checkAim(vertical,t);
 double before[16];std::memcpy(before,vertical,sizeof(before));t[0]=std::numeric_limits<double>::quiet_NaN();
 assert(!AimPoseAtWorldPoint(vertical,t)&&std::memcmp(before,vertical,sizeof(before))==0);
 s_irads_lock_valid=false;UpdateIRADSSensorFrame(.01,0,3000,0,0,0,0,1);
 assert(sensor.platformWorldPose.m[0]==.99); // free slew stays native
 s_irads_terrain_pending=true;s_irads_terrain_wait=.995;UpdateIRADSSensorFrame(.01,0,3000,0,0,0,0,1);assert(!s_irads_terrain_pending);
 puts("PASS: actual update/render lock holds fixed-world LOS over 3600 moving frames despite native lag; orthonormal pose, vertical target, invalid input, zoom/origin and unlocked native slew preserved");
}
'''
p=out/'point_track_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/point_track_tests.obj" /Fe"{out}/point_track_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'point_track_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
