from pathlib import Path
import subprocess,sys
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
fm=(repo/'EFM Source Code/F_117_NightHawk.cpp').read_text(encoding='utf-8-sig')
def extract(name):
 start=fm.index(name);brace=fm.index('{',start);end=brace+1;depth=1
 while depth:depth+=(fm[end]=='{')-(fm[end]=='}');end+=1
 return fm[start:end]
constants=fm[fm.index('    struct CcrpStoreModel'):fm.index('    std::string build_saved_games_dcs_path')]
start=fm.index('    constexpr double kGravity_MPS2');end=fm.index('    constexpr double kLuaDamageDose')
code=r'''
#include <cmath>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <limits>
using std::max;
#include "JDAM_HEADER"
struct Vec3 { double x,y,z; };
Vec3 position_world_cs{0,4000,0},wind_world_cs{};
double vx_world=200,vy_world=0,vz_world=0,target[3]{};
int profileIndex=0;
namespace F117 {double ambientDensity_KgPerM3=.819; double ccrp_time_of_flight=0;bool weapon_release_pickle_held=false;}
namespace CockpitInterop {
 bool GetIRADSLockWorldPoint(double* p){for(int i=0;i<3;++i)p[i]=target[i];return true;}
 void Log_(const char*){}
}
int get_ccrp_profile_index(){return profileIndex;}
'''+constants+fm[start:end]+r'''
const CcrpStoreModel& get_ccrp_profile(){return kCcrpStoreModels[profileIndex];}
bool g_releaseAlignmentValid=false;
double g_releaseMinRange=0,g_releaseMaxRange=0;
'''+ '\n'.join(extract(n) for n in ['Vec3 get_world_wind_vector()','bool simulate_bomb_impact_point(','bool guided_release_in_zone(','bool compute_release_solution('])+r'''
bool solve(bool expectedSolution,bool expectedZone) {
 double error=0,ttg=0,range=0;bool zone=false;
 bool valid=compute_release_solution(&error,&ttg,&range,&zone);
 assert(valid==expectedSolution);if(valid)assert(zone==expectedZone);return zone;
}
int main(){
 for(profileIndex=0;profileIndex<5;++profileIndex) {
  for(int heading=0;heading<2;++heading) {
   vx_world=heading?0:200;vz_world=heading?200:0;
   Vec3 impact{};double time=0;
   assert(simulate_bomb_impact_point(get_ccrp_profile(),position_world_cs,{vx_world,0,vz_world},wind_world_cs,0,&impact,&time));
   double ballistic=heading?impact.z:impact.x;
   auto aim=[&](double along,double cross){target[0]=heading?cross:along;target[1]=0;target[2]=heading?along:cross;};
   aim(ballistic,0);solve(true,true);
   // Previously inhibited: ordinary guidance correction, away from pinpoint CCRP.
   aim(ballistic*1.10,ballistic*.10);solve(true,true);
   aim(ballistic*2,0);solve(true,false); // beyond reach
   aim(ballistic*.3,0);solve(true,false); // too close
   aim(ballistic,ballistic);solve(true,false);assert(!g_releaseAlignmentValid);
   aim(-ballistic,0);solve(false,false); // behind aircraft
   aim(ballistic,0);target[0]=std::numeric_limits<double>::quiet_NaN();solve(false,false);
   bool entered=false;for(double r=ballistic*2;r>ballistic*.2;r-=100) {aim(r,0);double e,t,d;bool z;assert(compute_release_solution(&e,&t,&d,&z));entered|=z;}
   assert(entered); // continuous approach cannot skip a tiny release window
   aim(ballistic,0);double savedSpeed=vx_world;double savedZ=vz_world;
   vx_world=10;vz_world=0;solve(false,false);vx_world=savedSpeed;vz_world=savedZ;
   target[1]=3995;solve(false,false);
  }
 }
 // Exact authored nodes, interpolation and no out-of-domain extrapolation.
 for(int p=0;p<3;++p)for(int h=0;h<21;++h)for(int v=0;v<10;++v){
  double mn,mx;bool ok=JdamEnvelope::lookup(p,JdamEnvelope::heights[h],JdamEnvelope::speeds[v],&mn,&mx);
  assert(ok);assert(mn==JdamEnvelope::minimum[p][h][v]);assert(mx==JdamEnvelope::maximum[p][h][v]);
 }
 double mn,mx;assert(!JdamEnvelope::lookup(0,399,200,&mn,&mx));assert(!JdamEnvelope::lookup(0,4000,99,&mn,&mx));
 assert(!JdamEnvelope::lookup(0,15001,200,&mn,&mx));assert(!JdamEnvelope::lookup(0,4000,551,&mn,&mx));
 assert(JdamEnvelope::lookup(1,4500,275,&mn,&mx));assert(std::fabs(mx-(12400+18050+12800+19700)/4.0)<1e-9);
 // v8's 135% ballistic ceiling rejected this ordinary authored-table solution.
 profileIndex=1;vx_world=300;vz_world=0;target[0]=16000;target[1]=0;target[2]=0;solve(true,true);
 assert(g_releaseMaxRange==18050&&g_releaseMinRange==2750);
 target[0]=18051;solve(true,false);target[0]=2749;solve(true,false);
 // Relative height and true airspeed rather than absolute altitude/groundspeed.
 position_world_cs.y=4500;target[1]=500;target[0]=16000;solve(true,true);
 wind_world_cs.x=100;solve(true,false);assert(g_releaseMaxRange==7250);wind_world_cs.x=0;
 bool aligned=false;
 assert(!guided_release_in_zone(5,5000,5000,0,75,&aligned));
 assert(!guided_release_in_zone(0,5000,5000,-1,75,&aligned));
 assert(!guided_release_in_zone(0,5000,5000,0,75,nullptr));
 puts("PASS: actual ballistic integration + release solution for all five stores, both world headings; authored JDAM nodes/interpolation/domain, earlier release, relative height/wind, guided correction, continuous approach, too far/close/off-axis/behind/invalid/low speed/drop inhibits");
}
'''
code=code.replace('JDAM_HEADER',str(repo/'EFM Source Code/Avionics/JdamEnvelope.h').replace('\\','/'))
p=out/'zone_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/zone_tests.obj" /Fe"{out}/zone_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'zone_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
