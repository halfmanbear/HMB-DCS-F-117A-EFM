from pathlib import Path
import re, subprocess, sys
from lupa.lua51 import LuaRuntime
repo=Path(__file__).resolve().parents[1]
src=repo/'EFM Source Code'
mod=Path(sys.argv[1]); out=Path(sys.argv[2]); out.mkdir(exist_ok=True)
inputs=dict((name,int(value)) for name,value in re.findall(r'^\s*(\w+)\s*=\s*(\d+)\s*,', (src/'Inputs.h').read_text(encoding="utf-8-sig"),re.M))
lua=LuaRuntime(); lua.execute((repo/'Cockpit/Scripts/command_defs.lua').read_text(encoding="utf-8-sig"))
k=lua.globals().Keys; f=lua.globals().f117_commands; h=lua.globals().hotas_commands
for name,want in [('PanelLight',10001),('BcnLgt',10007),('IRADSZoomIn',10008),('IRADSZoomOut',10009),('MFDOverlayToggle',10010)]: assert k[name]==want,(name,k[name])
for lua_name,cpp_name in [('WEAPON_RELEASE_ON','weaponReleaseHoldOn'),('WEAPON_RELEASE_OFF','weaponReleaseHoldOff'),('IRADS_DESIGNATION_CLEAR','IRADSDesignationClear'),('IRADS_LASER_ARM_TOGGLE','IRADSLaserArmToggle'),('IRADS_SLEW_HORIZONTAL','IRADSSlewHorizontalAxis'),('IRADS_SLEW_VERTICAL','IRADSSlewVerticalAxis'),('IRADS_ZOOM_AXIS','IRADSZoomAxis')]: assert f[lua_name]==inputs[cpp_name]
for suffix,cpp_name in [('UP','IRADSSlewUp'),('DOWN','IRADSSlewDown'),('LEFT','IRADSSlewLeft'),('RIGHT','IRADSSlewRight'),('DEPRESS','IRADSSlewStop')]: assert h['THROTTLE_DESIGNATOR_CONTROLLER_'+suffix]==inputs[cpp_name]
assert k.IRADSZoomIn==inputs['IRADSZoomIn'] and k.IRADSZoomOut==inputs['IRADSZoomOut']
profile_signatures = {}
for device in ['keyboard','joystick']:
    vm=LuaRuntime(); vm.globals().folder=str(mod/f'Input/F-117-NightHawk/{device}').replace('\\','/')+'/'
    vm.execute("""
    _ = function(s) return s end
    external_profile = function(_) return {keyCommands={},axisCommands={}} end
    defaultDeviceAssignmentFor = function(_) return {} end
    join = function(a,b) for _,v in ipairs(b) do table.insert(a,v) end end
    setmetatable(_G,{__index=function(t,k) if string.sub(k,1,8)=='iCommand' then rawset(t,k,k); return k end end})
    """)
    result=vm.execute((mod/f'Input/F-117-NightHawk/{device}/default.lua').read_text(encoding="utf-8-sig"))
    rows=list(result.keyCommands.values()); custom=[r for r in rows if isinstance(r.down,(int,float))]
    seen=set()
    for r in custom:
        signature=(r.down,r.up)
        assert signature not in seen,(device,'duplicate custom binding',signature)
        seen.add(signature)
    def row(command): return next(r for r in rows if r.down==command)
    profile_signatures[device] = {command:(row(command).down,row(command).up,row(command).value_down,row(command).value_up) for command in [10008,10009,10110,10111,10112,10113,10114,87]}
    assert row(87).name=='IRADS On/Off'
    assert row(10010).name=='DLIR MFD Overlay Toggle'
    assert row(350).up==351 and row(350).name=='Weapon Release (Pickle)'
    assert not any(r.down==10103 for r in rows)
    assert row(10118).name=='IRADS Laser Arm/Safe'
    for command in range(10110,10115):
        r=row(command); assert r.up==command and r.value_down==1 and r.value_up==0
    assert not any(r.down in ['iCommandPlaneLaserRangerOnOff','iCommandPlaneZoomIn','iCommandPlaneZoomOut'] for r in rows)
    if device=='joystick':
        axes=list(result.axisCommands.values()); assert {r.action for r in axes}=={3001,3002,3003}; assert all(r.cockpit_device_id==7 for r in axes)
        assert not any(r.name=='Target Box Size' for r in axes)
    print('PASS: Lua 5.1',device,'profile: IDs, release edges, unique custom controls, overlay, laser and axes')
assert profile_signatures['keyboard']==profile_signatures['joystick'], 'DCS combines device profiles by full signature: mismatch creates duplicate rows'
print('PASS: keyboard/joystick full signatures match, including FOV and power')
# Run the complete deployed Displays.lua with mocked cockpit services. No input
# redispatch is permitted for EFM-owned motion/zoom; overlay remains functional.
vm=LuaRuntime(); params={}; events=[]
class Param:
    def __init__(self): self.value=0
    def get(self): return self.value
    def set(self,value): self.value=value
    def __getitem__(self,key): return getattr(self,key)
def param(name):
    if name not in params: params[name]=Param()
    return params[name]
vm.globals().get_param_handle=param
vm.globals().LockOn_Options=vm.table_from({'script_path':str(mod/'Cockpit/Scripts').replace('\\','/')+'/'})
vm.globals().make_default_activity=lambda dt: None
vm.execute('GetSelf=function() return {listen_command=function(...) end} end')
vm.globals().dispatch_action=lambda *args: events.append(args)
vm.execute((mod/'Cockpit/Scripts/Systems/Displays.lua').read_text(encoding="utf-8-sig"))
param('mainpower').set(1)
for command in [10008,10009,10110,10111,10112,10113,10114]: vm.globals().SetCommand(command,1)
assert events==[],events
# Exercise each profile action through the actual deployed SetCommand to the EFM.
for action,efm in [(3001,10119),(3002,10120),(3003,10121)]:
    for value in [-1,-.3,0,.3,1]:
        events.clear(); vm.globals().SetCommand(action,value)
        assert events==[(None,efm,value)], (action,value,events)
print('PASS: all device-targeted axis actions forward signed/center values exactly once to custom EFM commands')
events.clear()

param('MFD_OVERLAY').set(1); vm.globals().SetCommand(10010,0); assert param('MFD_OVERLAY').get()==0
vm.globals().SetCommand(10010,0); assert param('MFD_OVERLAY').get()==1
assert (repo/'Cockpit/Scripts/command_defs.lua').read_bytes()==(mod/'Cockpit/Scripts/command_defs.lua').read_bytes()
assert (repo/'Cockpit/Scripts/Systems/Displays.lua').read_bytes()==(mod/'Cockpit/Scripts/Systems/Displays.lua').read_bytes()
print('PASS: deployed cockpit source matches; overlay toggles; no Lua zoom/slew redispatch')

indicator=Path('Cockpit/Scripts/MFD/Indicator/MFD_indication_page.lua')
assert (repo/indicator).read_bytes()==(mod/indicator).read_bytes()
page=(mod/indicator).read_text(encoding='utf-8')
assert '"F117_LASER_ARMED"' in page and '{"MLAS_TOGGLE"}' not in page
syntax=LuaRuntime()
syntax.execute('assert(loadstring(...))',page)
assert 'dispatch_action(nil,392)' not in (mod/'Cockpit/Scripts/Systems/Displays.lua').read_text(encoding='utf-8')
print('PASS: MFD Lua 5.1 syntax; deployed source matches; LAS uses EFM arm state')

# Compile the actual production command handler with sensor-call mocks.
text=(src/'Avionics/CockpitBase_Interop.cpp').read_text(encoding="utf-8-sig")
def function(name):
    start=text.index('    '+name); brace=text.index('{',start); depth=1; end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}'); end+=1
    return text[start:end]
handler=function('bool HandleIRADSCommand(')
reset=function('void ResetIRADSSession(')
camera=function('static bool AimPoseAtWorldPoint(')+'\n'+function('static void* __fastcall IRADSGetCamera(')
resolver=function('static FARPROC ResolveCockpitBaseExport(')
assert 'GetProcAddress(g_hDLL' not in text, 'Native exports must not be resolved from the EFM module'
for export in ['set_platform@avPlatform', 'get_camera@ccIndicator', 'fill_camera@avTVSensor']:
    assert re.search(r'ResolveCockpitBaseExport\(\s*"\?'+re.escape(export), text), export

preamble=r"""
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdint>
#include <cstring>
using HMODULE=void*;
using FARPROC=void(*)();
static bool cockpitLoaded=true;
static int moduleLookups=0, exportLookups=0;
static int nativeModule;
void nativeSentinel(){}
HMODULE GetModuleHandleA(const char* name){assert(std::strcmp(name,"CockpitBase.dll")==0); ++moduleLookups; return cockpitLoaded? &nativeModule:nullptr;}
FARPROC GetProcAddress(HMODULE module,const char* name){assert(module==&nativeModule); assert(std::strcmp(name,"native-test-export")==0); ++exportLookups; return nativeSentinel;}
#include "LAYOUT_HEADER"
using namespace CockpitInterop;
AvTVSensorLayout* AsSensor(void* p){return static_cast<AvTVSensorLayout*>(p);}
#include "INPUT_HEADER"
alignas(16) static unsigned char sensor[2048]{};
static unsigned char cameraStorage[0x400]{};
void* fakeCamera(void*){return cameraStorage;}
static auto s_original_get_camera=fakeCamera;
static int cameraFills=0;
void fillCamera(void* sensorObject,void* cameraObject){assert(sensorObject==sensor); assert(cameraObject==cameraStorage); ++cameraFills;}
static auto s_sensor_fill_camera=fillCamera;
static void* s_camera_bridge_indicator=nullptr;
static bool s_irads_power=true;
static bool s_irads_lock_valid=false;
bool s_irads_ray_valid=false,s_irads_terrain_pending=false;double s_irads_render_ray[6]{},s_irads_request_ray[6]{};unsigned s_irads_terrain_sequence=0;double s_irads_terrain_wait=0;
static void* g_avTVSensor_instance=sensor;
static float s_irads_horizontal_axis=0, s_irads_vertical_axis=0;
static float s_irads_analog_horizontal=0, s_irads_analog_vertical=0;
static float s_irads_keyboard_horizontal=0, s_irads_keyboard_vertical=0;
static int s_irads_zoom_level=1;
static bool s_irads_designating=false, s_added=true, s_started=true, s_carrier_set=true;
static void* s_our_ind=sensor;
static double s_irads_lock_world[3]{},s_irads_lock_polar[3]{};
static int inCalls=0,outCalls=0,stops=0,designations=0,clears=0;
static double horizontal=0,vertical=0,lastNativeRate=0;
void left(void*,double v){lastNativeRate=v;horizontal=-v/60.0;}
void right(void*,double v){lastNativeRate=v;horizontal=v/60.0;}
void up(void*,double v){lastNativeRate=v;vertical=v/60.0;}
void down(void*,double v){lastNativeRate=v;vertical=-v/60.0;}
void stop(void*){++stops;horizontal=vertical=0;}
void zoomIn(void*){++inCalls;}
void zoomOut(void*){++outCalls;}
void (*g_avTVSensor_slew_left)(void*,double)=left;
void (*g_avTVSensor_slew_right)(void*,double)=right;
void (*g_avTVSensor_slew_up)(void*,double)=up;
void (*g_avTVSensor_slew_down)(void*,double)=down;
void (*g_avTVSensor_slew_stop)(void*)=stop;
void (*g_avTVSensor_zoom_in)(void*)=zoomIn;
void (*g_avTVSensor_zoom_out)(void*)=zoomOut;
bool (*g_avTVSensor_move_horizontal_abs)(void*,float,float,double)=nullptr;
bool (*g_avTVSensor_move_vertical_abs)(void*,float,float,double)=nullptr;
void Log(const char*,...){}
void Log_(const char*){}
bool GroundStabilizeIRADS(){++designations;return true;}
bool TryCaptureIRADSLock(){return true;}
void ClearIRADSLock(){++clears;s_irads_lock_valid=false;s_irads_terrain_pending=false;}
void SetExternalDesignation(bool){}
"""
main=r"""
int main(){
 assert(ResolveCockpitBaseExport("native-test-export")==nativeSentinel);
 cockpitLoaded=false; assert(ResolveCockpitBaseExport("native-test-export")==nullptr);
 assert(moduleLookups==2&&exportLookups==1); cockpitLoaded=true;
 puts("PASS: production native resolver uses CockpitBase.dll; missing module avoids export lookup");
 auto* optical=AsSensor(g_avTVSensor_instance); optical->baseFov=0.5235987756;
 int priorClears=clears;s_irads_terrain_pending=true;HandleIRADSCommand(IRADSSlewLeft,1);assert(!s_irads_terrain_pending&&clears==priorClears+1);HandleIRADSCommand(IRADSSlewLeft,0);clears=priorClears;
 assert(HandleIRADSCommand(IRADSZoomIn,0)); assert(s_irads_zoom_level==2&&optical->currentZoom==10&&optical->targetZoom==10);
 HandleIRADSCommand(IRADSZoomIn,1); HandleIRADSCommand(IRADSZoomIn,1);
 assert(s_irads_zoom_level==3&&optical->currentZoom==80);
 HandleIRADSCommand(IRADSZoomOut,0); assert(s_irads_zoom_level==2&&optical->currentZoom==10);
 HandleIRADSCommand(IRADSZoomAxis,-1); assert(s_irads_zoom_level==1&&optical->currentZoom==1.4);
 HandleIRADSCommand(IRADSZoomAxis,-1); assert(optical->targetZoom==1.4);
 HandleIRADSCommand(IRADSZoomAxis,0); assert(s_irads_zoom_level==2&&optical->currentZoom==10);
 HandleIRADSCommand(IRADSZoomAxis,.35f); assert(s_irads_zoom_level==2);
 HandleIRADSCommand(IRADSZoomAxis,.8f); assert(s_irads_zoom_level==3&&optical->currentZoom==80);
 HandleIRADSCommand(IRADSZoomAxis,.35f); assert(s_irads_zoom_level==3);
 HandleIRADSCommand(IRADSZoomAxis,0); assert(optical->currentZoom==10);
 // Execute the production rendering callback: it must bridge to native fill_camera.
 void* rendered=IRADSGetCamera(nullptr); assert(rendered==cameraStorage&&cameraFills==1);
 HandleIRADSCommand(IRADSZoomAxis,1); IRADSGetCamera(nullptr);
 assert(cameraFills==2&&optical->currentZoom==80);
 // Rate follows displayed FOV, including zoom changes while the stick is held.
 HandleIRADSCommand(IRADSZoomAxis,-1);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,.5f); assert(lastNativeRate==30);
 HandleIRADSCommand(IRADSZoomAxis,0); assert(std::abs(lastNativeRate-4.2)<1e-6);
 HandleIRADSCommand(IRADSZoomAxis,1); assert(std::abs(lastNativeRate-.525)<1e-6);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0);
 HandleIRADSCommand(IRADSZoomAxis,-1);
 // Legacy axes remain accepted while the profiles use the standard EFM axis range.
 HandleIRADSCommand(20110,.2f); assert(std::abs(horizontal-.2)<1e-6);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,.2f); assert(std::abs(horizontal-.2)<1e-6);
 HandleIRADSCommand(IRADSSlewVerticalAxis,.3f); assert(std::abs(vertical-.3)<1e-6);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0); assert(horizontal==0&&std::abs(vertical-.3)<1e-6);
 HandleIRADSCommand(IRADSSlewVerticalAxis,.02f); assert(vertical==0);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,-5); assert(horizontal==-1);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0);
 // Native rate is FOV-scaled with proportional axis control; keyboard takes priority.
 HandleIRADSCommand(IRADSSlewHorizontalAxis,.5f); assert(horizontal==.5&&lastNativeRate==30);
 HandleIRADSCommand(IRADSSlewLeft,1); assert(horizontal==-1&&lastNativeRate==60);
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0); assert(horizontal==-1); // center must not cancel key
 HandleIRADSCommand(IRADSSlewHorizontalAxis,.5f); assert(horizontal==-1);
 HandleIRADSCommand(IRADSSlewLeft,0); assert(horizontal==.5); // release returns analog control
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0);
 HandleIRADSCommand(IRADSSlewVerticalAxis,-.5f); assert(vertical==-.5&&lastNativeRate==30);
 HandleIRADSCommand(IRADSSlewUp,1); assert(vertical==1&&lastNativeRate==60);
 HandleIRADSCommand(IRADSSlewVerticalAxis,0); assert(vertical==1);
 HandleIRADSCommand(IRADSSlewUp,0); assert(vertical==0);
 HandleIRADSCommand(2110,.4f); assert(std::abs(horizontal-.4)<1e-6); // prior direct axis alias
 HandleIRADSCommand(IRADSSlewHorizontalAxis,0);
 HandleIRADSCommand(IRADSSlewLeft,1); assert(horizontal==-1);
 HandleIRADSCommand(IRADSSlewUp,1); assert(vertical==1);
 HandleIRADSCommand(IRADSSlewLeft,0); assert(horizontal==0&&vertical==1);
 HandleIRADSCommand(IRADSSlewStop,1); assert(horizontal==0&&vertical==0&&designations==1);
 HandleIRADSCommand(IRADSSlewStop,0); assert(designations==1);
 HandleIRADSCommand(IRADSDesignationClear,0); assert(clears==2);
 assert(!HandleIRADSCommand(IRADSSlewHorizontalAxis,std::numeric_limits<float>::quiet_NaN()));
 assert(!HandleIRADSCommand(IRADSZoomAxis,std::numeric_limits<float>::infinity()));
 assert(!HandleIRADSCommand(10007,1)); // beacon must never operate IRADS
 assert(!HandleIRADSCommand(10010,1)); // overlay is Lua-only
 HandleIRADSCommand(87,0); assert(!s_irads_power);
 assert(!HandleIRADSCommand(IRADSSlewLeft,1));
 HandleIRADSCommand(87,0); assert(s_irads_power&&horizontal==0&&vertical==0);
 s_irads_ray_valid=true;s_irads_terrain_pending=true;
 ResetIRADSSession(); assert(!s_irads_ray_valid&&!s_irads_terrain_pending); assert(s_irads_zoom_level==1&&s_irads_horizontal_axis==0&&s_irads_vertical_axis==0);
 assert(s_irads_analog_horizontal==0&&s_irads_analog_vertical==0);
 assert(s_irads_keyboard_horizontal==0&&s_irads_keyboard_vertical==0);
 assert(!HandleIRADSCommand(IRADSZoomIn,0));
 puts("PASS: production C++: optical current/target zoom, native render-camera delegation, analog detents/hysteresis, FOV-scaled proportional slew and held-input zoom transitions, centered-axis/keyboard arbitration, deadzone, stop/resume, designation, power, finite input and reset");
}
"""
preamble=preamble.replace('LAYOUT_HEADER',str(src/'Avionics/CockpitBase_Layouts.h').replace('\\','/'))
preamble=preamble.replace('INPUT_HEADER',str(src/'Inputs.h').replace('\\','/'))
harness=out/'irads_input_tests.cpp'; harness.write_text(preamble+resolver+'\n'+handler+'\n'+camera+'\n'+reset+'\n'+main,encoding='utf-8')
vcvars=Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
command=f'call "{vcvars}" >nul && cl /nologo /utf-8 /EHsc /std:c++17 /W4 "{harness}" /Fo"{out / "irads_input_tests.obj"}" /Fe"{out / "irads_input_tests.exe"}"'
build=subprocess.run(command,shell=True,text=True,capture_output=True)
print(build.stdout); print(build.stderr); assert build.returncode==0
run=subprocess.run([str(out/'irads_input_tests.exe')],text=True,capture_output=True)
print(run.stdout); print(run.stderr); assert run.returncode==0
