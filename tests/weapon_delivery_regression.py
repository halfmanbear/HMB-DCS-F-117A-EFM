from pathlib import Path
import subprocess,sys,shutil
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402
from lupa.lua51 import LuaRuntime
repo=Path(__file__).resolve().parents[1];src=repo/'EFM Source Code';mod=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=True)
def extract(text,name):
 start=text.index(name);brace=text.index('{',start);end=brace+1;depth=1
 while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
 return text[start:end]
def compile_run(name,code):
 p=out/(name+'.cpp');p.write_text(code,encoding='utf-8')
 vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
 cmd=f'call "{vc}" >nul && cl /nologo /utf-8 /EHsc /std:c++17 /I"{out}" "{p}" /Fo"{out/name}.obj" /Fe"{out/name}.exe"'
 r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout);print(r.stderr);assert r.returncode==0
 r=subprocess.run([str(out/(name+'.exe'))],capture_output=True,text=True);print(r.stdout);print(r.stderr);assert r.returncode==0
# Execute real station/profile script against changing, mixed stores.
vm=LuaRuntime();params={};listeners=[]
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
vm.execute('''log={INFO=1,write=function(...) end}; stations={
 [0]={count=1,CLSID="{GBU-31V3B}"},[1]={count=1,CLSID="{DB769D48-67D7-42ED-A2AA-108835D538F3}"}}
 GetSelf=function() return {listen_command=function(_,c) end,get_station_info=function(_,s) return stations[s] end} end''')
vm.execute((mod/'Cockpit/Scripts/Systems/Weapons.lua').read_text(encoding='utf-8'))
vm.globals().post_initialize();assert param('F117_CCRP_PROFILE').v==1 and param('F117_RELEASE_STATION').v==0
vm.globals().SetCommand(10123,0);assert param('F117_CCRP_PROFILE').v==3 and param('F117_RELEASE_STATION').v==1
vm.execute('stations[1].count=0');vm.globals()['update']();assert param('F117_STORE_VALID').v==0 and param('F117_RELEASE_STATION').v==-1
vm.execute('stations[1]={count=1,CLSID="{GBU_32_V_2B}"}');vm.globals()['update']();assert param('F117_CCRP_PROFILE').v==2
vm.execute('stations[1]={count=1,CLSID="{GBU-31}"}');vm.globals()['update']();assert param('F117_CCRP_PROFILE').v==0
vm.execute('stations[1]={count=1,CLSID="UNKNOWN"}');vm.globals()['update']();assert param('F117_STORE_VALID').v==0
print('PASS: actual Weapons.lua: V3B classification, selected-station agreement, mixed stores, depletion, rearm and unknown-store inhibition')
for rel in ['command_defs.lua','Systems/Weapons.lua','Systems/bomb_bay.lua','MFD/Indicator/MFD_indication_page.lua']:
 deployed=mod/'Cockpit/Scripts'/rel
 vm.execute('assert(loadstring(...))',deployed.read_text(encoding='utf-8'))
assert 'dispatch_action' not in (mod/'Cockpit/Scripts/Systems/bomb_bay.lua').read_text()
print('PASS: deployed Lua parses; no untargeted release fallback')
# Actual production gate/sequence functions with controlled cockpit and native outcomes.
fm=load_source()
names=['bool weapon_bay_valid(int bay)','bool weapon_bay_is_open(int bay)','bool weapon_bay_is_closed(int bay)','void command_weapon_bay_open(int bay)','void command_weapon_bay_close(int bay)','int selected_weapon_bay()','void forget_weapon_bay_ownership(int bay)','void reset_weapon_release_sequence()','void handle_weapon_release_pressed()','void handle_weapon_release_released()','void update_laser(double dt)','void update_weapon_release_sequence(double dt)']
code=r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
#include <algorithm>
#include <string>
using std::max;
namespace F117 {
 bool weapon_release_pickle_held=false,weapon_release_gate=false,weapon_release_gate_fired=false;
 constexpr int kWeaponBayCount=2;
 bool weapon_bay_owned_by_release[kWeaponBayCount]{},weapon_bay_close_pending[kWeaponBayCount]{},laser_armed=false,master_armed=false;
 float bay_cmd[kWeaponBayCount]{},bay_state[kWeaponBayCount]{};
 bool weight_on_wheels=false,wow_from_draw_args=false;
 double weapon_bay_close_delay[kWeaponBayCount]{},weapon_release_gate_hold=0,laser_window=0,weapon_flight_time_remaining=0,ccrp_time_of_flight=30,release_retry_delay=0,release_target[3]{};
 int release_status=0; unsigned release_sequence=0;
 struct API {void* getParamHandle(const char*);double getParamNumber(void*);} cockpitAPI;
}
double profileValid=1,stationValue=0,weaponStationValue=1;
void* F117::API::getParamHandle(const char* n){return n[5]=='S'? &profileValid:n[5]=='W'? &weaponStationValue:&stationValue;}
double F117::API::getParamNumber(void* p){return *static_cast<double*>(p);}
bool lockValid=true,power=true,solution=true,zone=true,available=true,accept=false,laserLive=false;
bool g_releaseAlignmentValid=true; double g_releaseMinRange=1000,g_releaseMaxRange=10000;
int launches=0,profile=0;double handoff[3]{};double publishedFlight=-1;
namespace CockpitInterop {
 bool IsIRADSPowered(){return power;} bool HasIRADSLock(){return lockValid;}
 bool GetIRADSLockWorldPoint(double* p){p[0]=1000;p[1]=0;p[2]=-2000;return lockValid;}
 int GetIRADSZoomLevel(){return 1;} void Log_(const char*){}
 bool GetIRADSLineOfSight(double* d){d[0]=0;d[1]=0;d[2]=1;return power&&lockValid;}
}
namespace WeaponsInterop {
 constexpr unsigned kDefaultLaserCode=1688;
 bool IsLaunchAvailable(){return available;}
 bool LaunchAtPoint(const double* p,int station){assert(station==0);++launches;for(int i=0;i<3;++i)handoff[i]=p[i];return accept;}
 void UpdateLaser(bool on,const double*,unsigned code){assert(code==1688);laserLive=on;}
 bool IsLaserFiring(){return laserLive;}
}
constexpr double kWeaponBayOpenThreshold=.995,kWeaponBayClosedThreshold=.005,kWeaponBayHoldAfterReleaseSeconds=1,kWeaponReleaseGateMinHoldSeconds=.25,kLaserOverrunSeconds=4;
const char *kLaserArmedParamName="arm",*kLaserFiringParamName="fire",*kEfmLaunchActiveParamName="efm";
void publish_named_param(void*&,const char* name,double value){if(std::string(name)=="F117_STORE_FLIGHT_TIME")publishedFlight=value;}
void setBay(bool open){F117::bay_state[0]=open?1.f:0.f;}
void publish_weapon_release_gate_param(){}void publish_release_cue_params(){}
int get_ccrp_profile_index(){return profile;}
'''+fm[fm.index('    enum class StoreKind'):fm.index('    constexpr double kGearCommandDown')]+r'''
const CcrpStoreModel& get_ccrp_profile(){return kCcrpStoreModels[profile];}
bool compute_release_solution(double*,double*,double*,bool* z){*z=zone;return solution;}
'''+ '\n'.join(extract(fm,n) for n in names)+r'''
int main(){
 setBay(true);handle_weapon_release_pressed();update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==1);
 F117::master_armed=true;lockValid=false;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==3);
 lockValid=true;profileValid=0;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==4);
 profileValid=1;profile=3;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==5);
 F117::laser_armed=true;setBay(false);update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==9&&laserLive); // doors still travelling: ready, not a fault, no release yet
 setBay(true);g_releaseAlignmentValid=false;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==14);
 g_releaseAlignmentValid=true;zone=false;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==8);
 zone=true;F117::weight_on_wheels=true;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==12);
 F117::weight_on_wheels=false;available=false;update_weapon_release_sequence(.01);assert(launches==0&&F117::release_status==13);
 available=true;update_weapon_release_sequence(.01);assert(launches==1&&!F117::weapon_release_gate_fired&&!F117::weapon_release_gate&&F117::release_sequence==0);
 update_weapon_release_sequence(.01);assert(launches==1);accept=true;
 update_weapon_release_sequence(.3);assert(launches==2&&F117::weapon_release_gate_fired&&F117::weapon_release_gate&&F117::release_sequence==1);
 assert(handoff[0]==1000&&handoff[2]==-2000&&F117::release_target[2]==-2000);
 assert(F117::weapon_flight_time_remaining==30&&F117::laser_window==34&&publishedFlight==30);
 update_weapon_release_sequence(2);assert(F117::weapon_flight_time_remaining==28&&F117::laser_window==32&&publishedFlight==28);
 handle_weapon_release_pressed();update_weapon_release_sequence(.3);assert(launches==2); // key repeat
 update_weapon_release_sequence(.3);assert(!F117::weapon_release_gate&&launches==2);
 handle_weapon_release_released();handle_weapon_release_pressed();profile=0;F117::laser_armed=false;
 update_weapon_release_sequence(.01);assert(launches==3&&!laserLive); // JDAM does not require laser
 handle_weapon_release_released();handle_weapon_release_pressed();profile=6;
 update_weapon_release_sequence(.01);assert(launches==4&&!laserLive); // Mk-82 does not require laser
 handle_weapon_release_released();handle_weapon_release_pressed();profile=13;
 update_weapon_release_sequence(.01);assert(launches==4&&F117::release_status==5); // AGM-65E needs the laser
 profile=12;update_weapon_release_sequence(.01);assert(launches==5); // AGM-65D does not
 handle_weapon_release_released();handle_weapon_release_pressed();profile=5;
 update_weapon_release_sequence(.01);assert(launches==5&&F117::release_status==5); // GBU-10 needs the laser
 profile=0;
 power=false;update_weapon_release_sequence(.01);assert(!laserLive);
 update_weapon_release_sequence(100);assert(F117::weapon_flight_time_remaining==0&&F117::laser_window==0&&publishedFlight==0);
 // Per-bay control: the pickle opens only the selected store's bay, and each bay closes on its own timer.
 F117::master_armed=false;F117::weapon_release_pickle_held=false;weaponStationValue=1;
 handle_weapon_release_released();reset_weapon_release_sequence();
 for(int b=0;b<2;++b){F117::bay_cmd[b]=0;F117::bay_state[b]=0;}
 assert(selected_weapon_bay()==0);weaponStationValue=2;assert(selected_weapon_bay()==1);weaponStationValue=3;assert(selected_weapon_bay()==-1);weaponStationValue=0;assert(selected_weapon_bay()==-1);
 weaponStationValue=2;stationValue=1;handle_weapon_release_pressed();
 assert(F117::bay_cmd[0]==0&&F117::bay_cmd[1]==1&&F117::weapon_bay_owned_by_release[1]&&!F117::weapon_bay_owned_by_release[0]);
 handle_weapon_release_released(); // right bay never reached open: closes straight away
 assert(F117::bay_cmd[1]==0&&!F117::weapon_bay_close_pending[1]);
 weaponStationValue=1;stationValue=0;handle_weapon_release_pressed();F117::bay_state[0]=1;handle_weapon_release_released();
 assert(F117::bay_cmd[0]==1&&F117::weapon_bay_close_pending[0]&&F117::weapon_bay_close_delay[0]==1);
 weaponStationValue=2;stationValue=1;handle_weapon_release_pressed(); // next store while the first bay waits to close
 assert(F117::bay_cmd[1]==1&&F117::weapon_bay_close_pending[0]);
 update_weapon_release_sequence(1.1);
 assert(F117::bay_cmd[0]==0&&!F117::weapon_bay_close_pending[0]&&F117::bay_cmd[1]==1); // left closes while right pickle is held
 // A pickle on the same bay during its pending close keeps it open and owned.
 reset_weapon_release_sequence();F117::bay_cmd[0]=F117::bay_cmd[1]=0;F117::bay_state[0]=F117::bay_state[1]=0;
 weaponStationValue=1;stationValue=0;handle_weapon_release_pressed();F117::bay_state[0]=1;handle_weapon_release_released();
 handle_weapon_release_pressed();assert(!F117::weapon_bay_close_pending[0]&&F117::bay_cmd[0]==1&&F117::weapon_bay_owned_by_release[0]);
 // Selection moving to the other store while consent is held opens that store's bay too.
 weaponStationValue=2;stationValue=1;update_weapon_release_sequence(.01);
 assert(F117::bay_cmd[1]==1&&F117::weapon_bay_owned_by_release[1]);
 // A bay the pilot opened by hand is not the pickle's to close.
 reset_weapon_release_sequence();F117::bay_cmd[1]=F117::bay_state[1]=1;F117::bay_cmd[0]=F117::bay_state[0]=0;
 handle_weapon_release_pressed();assert(!F117::weapon_bay_owned_by_release[1]);handle_weapon_release_released();
 assert(F117::bay_cmd[1]==1&&!F117::weapon_bay_close_pending[1]);
 // Resetting the sequence (manual command, Master Arm) closes only bays the pickle opened.
 weaponStationValue=1;stationValue=0;handle_weapon_release_pressed();assert(F117::weapon_bay_owned_by_release[0]);
 reset_weapon_release_sequence();assert(F117::bay_cmd[0]==0&&F117::bay_cmd[1]==1&&!F117::weapon_release_pickle_held);
 puts("PASS: production release gate: safe/power/ground/target/store/laser/bay/zone/backend inhibits, retry, one release per consent, JDAM target snapshot and laser need for all 14 stores");
}
'''
compile_run('delivery_gate_tests',code)
# Test real native interop against mock interfaces; no DCS library is loaded or executed.
(out/'stdafx.h').write_text('#pragma once\n#include <Windows.h>\n',encoding='utf-8')
weaponpath=str(src/'Avionics/Weapons_Interop.cpp').replace('\\','/')
code=r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include "WEAPON_CPP"
namespace CockpitInterop {void** g_contexts_ptr_addr=nullptr;void Log_(const char*){}}
static int launchCount=0,selected=-1,nativeResult=1,spotKills=0,spotMoves=0;
static float passedPoint[3]{};static unsigned char fakeSpot[0x400]{};
static void* managerSlots[64]{};static void** manager=managerSlots;
void* getManager(void*){return &manager;}
int count(void*,int s){return s==0?1:0;}void selectStation(void*,int s){selected=s;}
int current(void*){return 0;}int stations(void*){return 2;}
int launch(void*,int station,int sub,unsigned id,const WeaponsInterop::Vec3f* point,bool armed){
 assert(station==0&&sub==-1&&id==0&&armed);++launchCount;passedPoint[0]=point->x;passedPoint[1]=point->y;passedPoint[2]=point->z;return nativeResult;}
void* create(unsigned owner,const WeaponsInterop::Vec3d*,const WeaponsInterop::Vec3d* p,unsigned code){assert(owner==42&&code==1688&&p->x==1000);return fakeSpot;}
void move(void*,const WeaponsInterop::Vec3d*){++spotMoves;}void kill(void* p){assert(p==fakeSpot+8);++spotKills;}
int main(){
 void* movingSlots[64]{};movingSlots[0x120/8]=reinterpret_cast<void*>(getManager);void** moving=movingSlots;
 managerSlots[0xa0/8]=reinterpret_cast<void*>(selectStation);managerSlots[0xd0/8]=reinterpret_cast<void*>(count);
 managerSlots[0x168/8]=reinterpret_cast<void*>(current);managerSlots[0x190/8]=reinterpret_cast<void*>(launch);managerSlots[0x1b8/8]=reinterpret_cast<void*>(stations);
 CockpitInterop::CcCockpitContextLayout context{};context.movingObjectRaw=&moving;void* ctx=&context;CockpitInterop::g_contexts_ptr_addr=&ctx;
 double p[3]={1000,50,-2000};assert(WeaponsInterop::LaunchAtPoint(p,0));assert(launchCount==1&&selected==0&&passedPoint[2]==-2000);
 assert(!WeaponsInterop::LaunchAtPoint(p,1)&&launchCount==1); // never fall through to loaded station 0
 nativeResult=0;assert(!WeaponsInterop::LaunchAtPoint(p,0));
 p[0]=NAN;assert(!WeaponsInterop::LaunchAtPoint(p,0));p[0]=1000;
 unsigned char owner[0x200]{};*reinterpret_cast<unsigned*>(owner+0x1c)=42;context.carrierLinkHostRaw=owner+0x100;
 WeaponsInterop::s_resolved=true;WeaponsInterop::s_createLaserSpot=create;WeaponsInterop::s_setPoint=move;WeaponsInterop::s_registerToKill=kill;
 WeaponsInterop::s_spotSuicideVtbl=managerSlots;*reinterpret_cast<const void**>(fakeSpot+8)=managerSlots;
 WeaponsInterop::UpdateLaser(true,p,1688);assert(WeaponsInterop::IsLaserFiring());
 WeaponsInterop::UpdateLaser(true,p,1688);assert(spotMoves==1);
 WeaponsInterop::UpdateLaser(false,p,1688);assert(!WeaponsInterop::IsLaserFiring()&&spotKills==1);
 WeaponsInterop::UpdateLaser(true,p,1688);WeaponsInterop::Reset();assert(spotKills==2&&!WeaponsInterop::IsLaserFiring());
 puts("PASS: actual Weapons_Interop.cpp: explicit station, coordinates, native nonzero launch acceptance, finite input and laser creation/move/removal/reset");
}
'''.replace('WEAPON_CPP',weaponpath)
compile_run('weapons_native_contract_tests',code)

# Terrain designation now requires a matching asynchronous terrain result.
subprocess.run([sys.executable,str(repo/'tests/terrain_lock_regression.py'),str(mod),str(out/'terrain')],check=True)
