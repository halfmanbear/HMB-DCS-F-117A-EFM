from pathlib import Path
import re, subprocess, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source, extract, unit_body  # noqa: E402
from lupa.lua51 import LuaRuntime
repo=Path(__file__).resolve().parents[1]
src=repo/'EFM Source Code'
mod=Path(sys.argv[1]); out=Path(sys.argv[2]); out.mkdir(exist_ok=True)
inputs=dict((name,int(value)) for name,value in re.findall(r'^\s*constexpr int (\w+)\s*=\s*(\d+)\s*;', (src/'Inputs.h').read_text(encoding="utf-8-sig"),re.M))
lua=LuaRuntime(); lua.execute((mod/'Cockpit/Scripts/command_defs.lua').read_text(encoding="utf-8-sig"))
k=lua.globals().Keys; f=lua.globals().f117_commands; h=lua.globals().hotas_commands; fov=lua.globals().irads_fov_commands
for name,want in [('IRADSZoomIn',10008),('IRADSZoomOut',10009)]: assert k[name]==want,(name,k[name])
for lua_name,cpp_name in [('WEAPON_RELEASE_ON','weapon_release_hold_on'),('WEAPON_RELEASE_OFF','weapon_release_hold_off'),('IRADS_SLEW_HORIZONTAL','irads_slew_horizontal_axis'),('IRADS_SLEW_VERTICAL','irads_slew_vertical_axis'),('IRADS_ZOOM_AXIS','irads_zoom_axis')]: assert f[lua_name]==inputs[cpp_name]
for suffix,cpp_name in [('UP','irads_slew_up'),('DOWN','irads_slew_down'),('LEFT','irads_slew_left'),('RIGHT','irads_slew_right'),('DEPRESS','irads_slew_stop')]: assert h['THROTTLE_DESIGNATOR_CONTROLLER_'+suffix]==inputs[cpp_name]
assert k.IRADSZoomIn==inputs['irads_zoom_in'] and k.IRADSZoomOut==inputs['irads_zoom_out']
profile_signatures = {}
for device in ['keyboard','joystick']:
    vm=LuaRuntime(); vm.globals().folder=str(mod/f'Input/F-117-NightHawk/{device}').replace('\\','/')+'/'
    vm.execute("""
    _ = function(s) return s end
    external_profile = function(_) return {keyCommands={},axisCommands={}} end
    defaultDeviceAssignmentFor = function(_) return {} end
    join = function(a,b) for _,v in ipairs(b) do table.insert(a,v) end end
    -- Stock DCS command numbers the profiles use by name.
    iCommandPlaneEOSOnOff, iCommandPlanePickleOn, iCommandPlanePickleOff = 87, 350, 351
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
    profile_signatures[device] = {command:(row(command).down,row(command).up,row(command).value_down,row(command).value_up) for command in ['iCommandPlaneZoomOut','iCommandPlaneZoomIn','iCommandSelecterUp','iCommandSelecterDown','iCommandSelecterLeft','iCommandSelecterRight','iCommandPlane_HOTAS_TDC_depress_on',87]}
    assert row(87).name=='IRADS On/Off'
    # The FOV buttons are the stock zoom commands, sent to the Displays device, which turns them into the EFM's zoom axis endpoints.
    assert row('iCommandPlaneZoomOut').name=='IRADS FOV - Wide' and row('iCommandPlaneZoomIn').name=='IRADS FOV - Narrow'
    assert row('iCommandPlaneZoomOut').cockpit_device_id==row('iCommandPlaneZoomIn').cockpit_device_id==7
    assert not any(r.down in [10008,10009] for r in rows)
    assert not any(r.name=='DLIR MFD Overlay Toggle' for r in rows)  # symbology follows IRADS power
    assert row(350).up==351 and row(350).name=='Weapon Release (Pickle)'
    assert not any(r.down==10103 for r in rows)
    # Master arm and laser arm are stock commands the EFM handles itself, so they carry no device id.
    for command,name in [('iCommandPlaneAHCPMasterArm','Master Arm'),('iCommandPlaneAHCPMasterSafe','Master Safe'),('iCommandPlaneAHCPLaserArm','IRADS Laser Arm'),('iCommandPlaneAHCPLaserSafe','IRADS Laser Safe')]:
        assert row(command).name==name and row(command).cockpit_device_id is None,command
    assert not any(r.down in [10118,10122,10011,10117] for r in rows)
    assert row('iCommandSensorReset').name=='IRADS Scan Return / Undesignate'
    # The slew keys are stock Selecter commands released by Selecter Stop; the Displays device forwards them to 10110-10113.
    for command in ['iCommandSelecterUp','iCommandSelecterDown','iCommandSelecterLeft','iCommandSelecterRight']:
        r=row(command); assert r.up=='iCommandSelecterStop' and r.value_down==1 and r.value_up==0 and r.cockpit_device_id==7
    r=row('iCommandPlane_HOTAS_TDC_depress_on'); assert r.up=='iCommandPlane_HOTAS_TDC_depress_off' and r.cockpit_device_id==7
    assert not any(r.down in ['iCommandPlaneLaserRangerOnOff'] for r in rows)
    if device=='joystick':
        axes=list(result.axisCommands.values()); assert {r.action for r in axes}=={3001,3002,3003,3009,3010}; assert all(r.cockpit_device_id==7 for r in axes)
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
vm.execute('''
function GetSelf() return {listen_command=function(...)end} end
function get_base_data() return {} end
''')
vm.globals().dispatch_action=lambda *args: events.append(args)
vm.execute((mod/'Cockpit/Scripts/Systems/Displays.lua').read_text(encoding="utf-8-sig"))
param('mainpower').set(1)
for command in [10008,10009,10110,10111,10112,10113,10114]: vm.globals().SetCommand(command,1)
assert events==[],events
# Stock slew and depress commands reach the Displays device and are forwarded to the EFM's own IDs once.
nc=vm.globals().native_commands
for stock,efm,value in [(nc.SELECTER_UP,10112,1.0),(nc.SELECTER_DOWN,10113,1.0),(nc.SELECTER_LEFT,10110,1.0),(nc.SELECTER_RIGHT,10111,1.0),
                        (nc.TDC_DEPRESS_ON,10114,1.0),(nc.TDC_DEPRESS_OFF,10114,0.0)]:
    events.clear(); vm.globals().SetCommand(stock,1); assert events==[(None,efm,value)],(stock,events)
events.clear(); vm.globals().SetCommand(nc.SELECTER_STOP,0); assert events==[(None,i,0.0) for i in (10112,10113,10110,10111)],events
events.clear()
# Exercise each profile action through the actual deployed SetCommand to the EFM.
for action,efm in [(3001,10119),(3002,10120)]:
    for value in [-1,-.3,0,.3,1]:
        events.clear(); vm.globals().SetCommand(action,value)
        assert events==[(None,efm,value)], (action,value,events)
# The FOV axis has two detents with a dead band between them, and sends only the wide and narrow ends to the EFM.
for value,sent in [(-1,-1),(-.3,-1),(0,-1),(.3,1),(0,1),(1,1),(-.3,-1)]:
    events.clear(); vm.globals().SetCommand(3003,value)
    assert events==[(None,10121,sent)], (value,events)
print('PASS: all device-targeted axis actions forward signed/center values exactly once to custom EFM commands')
events.clear()
for command,axis in [(fov.WIDE,-1),(fov.NARROW,1),(nc.ZOOM_OUT,-1),(nc.ZOOM_IN,1)]:
    events.clear(); vm.globals().SetCommand(command,1)
    assert events==[(None,f['IRADS_ZOOM_AXIS'],axis)],(command,events)
    events.clear(); vm.globals().SetCommand(command,0)
    assert events==[],(command,events)
print('PASS: FOV buttons select the wide and narrow ends of the EFM zoom axis once, on press only')
events.clear()

print('PASS: deployed cockpit commands exercised; no Lua zoom/slew redispatch')

indicator=Path('Cockpit/Scripts/MFD/Indicator/MFD_indication_page.lua')
page=(mod/indicator).read_text(encoding='utf-8')
readouts=(mod/'Cockpit/Scripts/Systems/IRADS_Readouts.lua').read_text(encoding='utf-8')
# The page draws laser state from text that IRADS_Readouts.lua builds from the EFM's F117_LASER_ARMED.
assert '"F117_IRADS_LASER_TEXT"' in page and '"F117_LASER_ARMED"' in readouts and '{"MLAS_TOGGLE"}' not in page
syntax=LuaRuntime()
syntax.execute('assert(loadstring(...))',page)
assert 'dispatch_action(nil,392)' not in (mod/'Cockpit/Scripts/Systems/Displays.lua').read_text(encoding='utf-8')
print('PASS: MFD Lua 5.1 syntax; deployed page parses; LAS uses EFM arm state')

# Compile the actual production command handler with sensor-call mocks.
text=load_source('Avionics')
def function(name):
    return extract(text,name)
handler=unit_body('Avionics/Interop/InputHandler.cpp')
reset=function('void reset_irads_session(')
camera=function('bool aim_pose_at_world_point(')+'\n'+function('static void* __fastcall irads_get_camera(')
resolver=function('FARPROC resolve_cockpit_base_export(')
assert 'GetProcAddress(g_h_dll' not in text, 'Native exports must not be resolved from the EFM module'
for export in ['set_platform@avPlatform', 'get_camera@ccIndicator', 'fill_camera@avTVSensor']:
    assert re.search(r'resolve_cockpit_base_export\(\s*"\?'+re.escape(export), text), export

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
using namespace Cockpit_interop;
Av_tv_sensor_layout* as_sensor(void* p){return static_cast<Av_tv_sensor_layout*>(p);}
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
static bool s_irads_homing=false;
bool s_irads_ray_valid=false,s_irads_terrain_pending=false;double s_irads_render_ray[6]{},s_irads_request_ray[6]{};unsigned s_irads_terrain_sequence=0;double s_irads_terrain_wait=0,s_irads_ray_age=1e9;
static void* g_av_tv_sensor_instance=sensor;
static float s_irads_horizontal_axis=0, s_irads_vertical_axis=0;
static float s_irads_analog_horizontal=0, s_irads_analog_vertical=0;
static float s_irads_keyboard_horizontal=0, s_irads_keyboard_vertical=0;
static int s_irads_zoom_level=1;
static bool s_added=true;
static void* s_our_ind=sensor;
static double s_irads_lock_world[3]{};
constexpr size_t camera_eye_offset=0x260,camera_forward_offset=0x230,sensor_caged_byte_offset=938;
constexpr unsigned render_log_limit=120,render_log_interval=120;constexpr double rad_to_deg=57.295779513;
void reset_camera_bridge(){s_camera_bridge_indicator=nullptr;s_our_ind=nullptr;}
static int inCalls=0,outCalls=0,stops=0,designations=0,clears=0;
static double horizontal=0,vertical=0,lastNativeRate=0;
void left(void*,double v){lastNativeRate=v;horizontal=-v/60.0;}
void right(void*,double v){lastNativeRate=v;horizontal=v/60.0;}
void up(void*,double v){lastNativeRate=v;vertical=v/60.0;}
void down(void*,double v){lastNativeRate=v;vertical=-v/60.0;}
void stop(void*){++stops;horizontal=vertical=0;}
void zoomIn(void*){++inCalls;}
void zoomOut(void*){++outCalls;}
void (*g_av_tv_sensor_slew_left)(void*,double)=left;
void (*g_av_tv_sensor_slew_right)(void*,double)=right;
void (*g_av_tv_sensor_slew_up)(void*,double)=up;
void (*g_av_tv_sensor_slew_down)(void*,double)=down;
void (*g_av_tv_sensor_slew_stop)(void*)=stop;
void (*g_av_tv_sensor_zoom_in)(void*)=zoomIn;
void (*g_av_tv_sensor_zoom_out)(void*)=zoomOut;
bool (*g_av_tv_sensor_move_horizontal_abs)(void*,float,float,double)=nullptr;
bool (*g_av_tv_sensor_move_vertical_abs)(void*,float,float,double)=nullptr;
void log_format(const char*,...){}
void log_message(const char*){}
bool ground_stabilize_irads(){++designations;return true;}
void clear_irads_lock(){++clears;s_irads_lock_valid=false;s_irads_terrain_pending=false;}
void (*g_av_tv_sensor_set_external_designation)(void*,char,char)=nullptr;
"""
main=r"""
int main(){
 assert(resolve_cockpit_base_export("native-test-export")==nativeSentinel);
 cockpitLoaded=false; assert(resolve_cockpit_base_export("native-test-export")==nullptr);
 assert(moduleLookups==2&&exportLookups==1); cockpitLoaded=true;
 puts("PASS: production native resolver uses CockpitBase.dll; missing module avoids export lookup");
 auto* optical=as_sensor(g_av_tv_sensor_instance); optical->base_fov=0.5235987756;
 int priorClears=clears;s_irads_terrain_pending=true;handle_irads_command(irads_slew_left,1);assert(!s_irads_terrain_pending&&clears==priorClears+1);handle_irads_command(irads_slew_left,0);clears=priorClears;
 assert(handle_irads_command(irads_zoom_in,0)); assert(s_irads_zoom_level==2&&optical->current_zoom==10&&optical->target_zoom==10);
 handle_irads_command(irads_zoom_in,1); handle_irads_command(irads_zoom_in,1);
 assert(s_irads_zoom_level==3&&optical->current_zoom==80);
 handle_irads_command(irads_zoom_out,0); assert(s_irads_zoom_level==2&&optical->current_zoom==10);
 handle_irads_command(irads_zoom_axis,-1); assert(s_irads_zoom_level==1&&optical->current_zoom==1.4);
 handle_irads_command(irads_zoom_axis,-1); assert(optical->target_zoom==1.4);
 handle_irads_command(irads_zoom_axis,0); assert(s_irads_zoom_level==2&&optical->current_zoom==10);
 handle_irads_command(irads_zoom_axis,.35f); assert(s_irads_zoom_level==2);
 handle_irads_command(irads_zoom_axis,.8f); assert(s_irads_zoom_level==3&&optical->current_zoom==80);
 handle_irads_command(irads_zoom_axis,.35f); assert(s_irads_zoom_level==3);
 handle_irads_command(irads_zoom_axis,0); assert(optical->current_zoom==10);
 // Execute the production rendering callback: it must bridge to native fill_camera.
 void* rendered=irads_get_camera(nullptr); assert(rendered==cameraStorage&&cameraFills==1);
 handle_irads_command(irads_zoom_axis,1); irads_get_camera(nullptr);
 assert(cameraFills==2&&optical->current_zoom==80);
 // Rate follows displayed FOV, including zoom changes while the stick is held.
 handle_irads_command(irads_zoom_axis,-1);
 handle_irads_command(irads_slew_horizontal_axis,.5f); assert(lastNativeRate==30);
 handle_irads_command(irads_zoom_axis,0); assert(std::abs(lastNativeRate-4.2)<1e-6);
 handle_irads_command(irads_zoom_axis,1); assert(std::abs(lastNativeRate-.525)<1e-6);
 handle_irads_command(irads_slew_horizontal_axis,0);
 handle_irads_command(irads_zoom_axis,-1);
 // Legacy axes remain accepted while the profiles use the standard EFM axis range.
 handle_irads_command(20110,.2f); assert(std::abs(horizontal-.2)<1e-6);
 handle_irads_command(irads_slew_horizontal_axis,.2f); assert(std::abs(horizontal-.2)<1e-6);
 handle_irads_command(irads_slew_vertical_axis,.3f); assert(std::abs(vertical-.3)<1e-6);
 handle_irads_command(irads_slew_horizontal_axis,0); assert(horizontal==0&&std::abs(vertical-.3)<1e-6);
 handle_irads_command(irads_slew_vertical_axis,.02f); assert(vertical==0);
 handle_irads_command(irads_slew_horizontal_axis,-5); assert(horizontal==-1);
 handle_irads_command(irads_slew_horizontal_axis,0);
 // Native rate is FOV-scaled with proportional axis control; keyboard takes priority.
 handle_irads_command(irads_slew_horizontal_axis,.5f); assert(horizontal==.5&&lastNativeRate==30);
 handle_irads_command(irads_slew_left,1); assert(horizontal==-1&&lastNativeRate==60);
 handle_irads_command(irads_slew_horizontal_axis,0); assert(horizontal==-1); // center must not cancel key
 handle_irads_command(irads_slew_horizontal_axis,.5f); assert(horizontal==-1);
 handle_irads_command(irads_slew_left,0); assert(horizontal==.5); // release returns analog control
 handle_irads_command(irads_slew_horizontal_axis,0);
 handle_irads_command(irads_slew_vertical_axis,-.5f); assert(vertical==-.5&&lastNativeRate==30);
 handle_irads_command(irads_slew_up,1); assert(vertical==1&&lastNativeRate==60);
 handle_irads_command(irads_slew_vertical_axis,0); assert(vertical==1);
 handle_irads_command(irads_slew_up,0); assert(vertical==0);
 handle_irads_command(2110,.4f); assert(std::abs(horizontal-.4)<1e-6); // prior direct axis alias
 handle_irads_command(irads_slew_horizontal_axis,0);
 handle_irads_command(irads_slew_left,1); assert(horizontal==-1);
 handle_irads_command(irads_slew_up,1); assert(vertical==1);
 handle_irads_command(irads_slew_left,0); assert(horizontal==0&&vertical==1);
 handle_irads_command(irads_slew_stop,1); assert(horizontal==0&&vertical==0&&designations==1);
 handle_irads_command(irads_slew_stop,0); assert(designations==1);
 // One switch designates and undesignates: with a lock held, the next press removes it instead of re-designating.
 s_irads_lock_valid=true; int beforeClears=clears;
 handle_irads_command(irads_slew_stop,1); assert(clears==beforeClears+1&&!s_irads_lock_valid&&designations==1);
 handle_irads_command(irads_slew_stop,0); assert(designations==1);
 s_irads_terrain_pending=true; beforeClears=clears;
 handle_irads_command(irads_slew_stop,1); assert(clears==beforeClears+1&&!s_irads_terrain_pending&&designations==1);
 handle_irads_command(irads_slew_stop,0);
 // Sensor Reset drops the lock and homes the gimbal: the LOS (column 0 of the sensor pose) is driven back to the mount axes.
 {
  double mount[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  auto setLos=[&](double azDeg,double elDeg){auto* sensorPose=as_sensor(g_av_tv_sensor_instance)->platform_world_pose.m;
    const double az=azDeg/rad_to_deg,el=elDeg/rad_to_deg;
    sensorPose[0]=std::cos(el)*std::cos(az);sensorPose[1]=std::sin(el);sensorPose[2]=std::cos(el)*std::sin(az);};
  s_irads_lock_valid=true; beforeClears=clears; int beforeStops=stops;
  assert(handle_irads_command(sensor_reset,1)); assert(clears==beforeClears+1&&!s_irads_lock_valid&&s_irads_homing&&stops==beforeStops+1);
  setLos(20,10); service_irads_home(mount);   // right of and above home: slew left and down
  assert(s_irads_homing&&horizontal<0&&vertical<0&&std::abs(lastNativeRate)>2.99);
  setLos(-20,-10); stops=0; service_irads_home(mount);  // left of and below home: reverses, so the motors stop and re-issue
  assert(s_irads_homing&&horizontal>0&&vertical>0&&stops==1);
  setLos(0.1,-10); stops=0; service_irads_home(mount); // azimuth arrives while elevation still moves
  assert(s_irads_homing&&stops==1&&vertical>0);
  setLos(0.1,0.1); stops=0; service_irads_home(mount); // both on target: stop and finish
  assert(!s_irads_homing&&stops==1&&horizontal==0&&vertical==0);
  setLos(30,0); service_irads_home(mount); assert(!s_irads_homing&&horizontal==0); // not homing: nothing moves
  handle_irads_command(sensor_reset,1); assert(s_irads_homing);
  handle_irads_command(irads_slew_left,1); assert(!s_irads_homing); handle_irads_command(irads_slew_left,0); // pilot input takes over
  handle_irads_command(sensor_reset,1); handle_irads_command(87,0); assert(!s_irads_homing); handle_irads_command(87,0);
  setLos(0,0);
 }
 assert(!handle_irads_command(irads_slew_horizontal_axis,std::numeric_limits<float>::quiet_NaN()));
 assert(!handle_irads_command(irads_zoom_axis,std::numeric_limits<float>::infinity()));
 assert(!handle_irads_command(10007,1)); // beacon must never operate IRADS
 handle_irads_command(87,0); assert(!s_irads_power);
 assert(!handle_irads_command(irads_slew_left,1));
 handle_irads_command(87,0); assert(s_irads_power&&horizontal==0&&vertical==0);
 s_irads_ray_valid=true;s_irads_terrain_pending=true;
 reset_irads_session(); assert(!s_irads_ray_valid&&!s_irads_terrain_pending); assert(s_irads_zoom_level==1&&s_irads_horizontal_axis==0&&s_irads_vertical_axis==0);
 assert(s_irads_analog_horizontal==0&&s_irads_analog_vertical==0);
 assert(s_irads_keyboard_horizontal==0&&s_irads_keyboard_vertical==0);
 assert(!handle_irads_command(irads_zoom_in,0));
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
