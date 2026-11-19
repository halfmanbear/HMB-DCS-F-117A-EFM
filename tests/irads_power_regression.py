"""Compile the production IRADS selection/readiness functions with parameter stubs."""
from pathlib import Path
import subprocess
import sys
from lib.sources import extract

ROOT = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = (ROOT / 'EFM Source Code/Avionics/Interop/InputHandler.cpp').read_text()
code = r'''
#include <cassert>
#include <cstdio>
static bool s_irads_power=false,s_irads_homing=true;
static bool lock=true,read_ok=true;
static double ready=0;
static int stops=0;
static int sensor;
static void* g_av_tv_sensor_instance=&sensor;
void stop(void* p){assert(p==&sensor);++stops;}
auto g_av_tv_sensor_slew_stop=stop;
void clear_slew_demand(){}
void clear_irads_lock(){lock=false;}
void log_format(const char*,const char*){}
bool read_named_param(void*&,const char*,double& v){v=ready;return read_ok;}
'''
code += extract(source, 'void set_irads_power_selected(')
code += extract(source, 'bool is_irads_powered(')
code += r'''
int main(){
 assert(!is_irads_powered());
 set_irads_power_selected(true);
 assert(!is_irads_powered() && !lock && !s_irads_homing && stops==1);
 ready=1;assert(is_irads_powered());
 ready=0;assert(!is_irads_powered());
 ready=1;assert(is_irads_powered());
 read_ok=false;assert(!is_irads_powered());read_ok=true;
 set_irads_power_selected(false);assert(!is_irads_powered());
 g_av_tv_sensor_instance=nullptr;
 set_irads_power_selected(true);assert(stops==2);
 puts("PASS native IRADS: selection, BIT readiness, power loss, missing parameter, lock reset and pre-injection selection");
}
'''
cpp = out / 'irads_power.cpp'
cpp.write_text(code)
exe = out / 'irads_power.exe'
vcvars = Path(r'C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')
cmd = out / 'compile-irads-power.cmd'
cmd.write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /EHsc /std:c++17 "{cpp}" /Fe:"{exe}" /Fo:"{out / "irads_power.obj"}"\n')
subprocess.run(['cmd.exe', '/c', str(cmd)], check=True, cwd=out)
subprocess.run([str(exe)], check=True)
