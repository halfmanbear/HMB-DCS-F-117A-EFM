from pathlib import Path
import subprocess,sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
fm=load_source()
ccrp_h=load_source('Weapons/Ccrp.h')
def extract(name):
 start=fm.index(name);brace=fm.index('{',start);end=brace+1;depth=1
 while depth:depth+=(fm[end]=='{')-(fm[end]=='}');end+=1
 return fm[start:end]
constants=ccrp_h[ccrp_h.index('enum class Store_kind'):ccrp_h.index('constexpr double gravity_mps2')]
start=ccrp_h.index('constexpr double gravity_mps2');end=ccrp_h.index('constexpr double weapon_bay_open_threshold')
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
int profile_index=0;
namespace F117 {double ambient_density_kg_per_m3=.819; double ccrp_time_of_flight=0;bool weapon_release_pickle_held=false;}
namespace Cockpit_interop {
 bool get_irads_lock_world_point(double* p){for(int i=0;i<3;++i)p[i]=target[i];return true;}
 void log_message(const char*){} void log_format(const char*,...){}
}
int get_ccrp_profile_index(){return profile_index;}
'''+constants+ccrp_h[start:end]+r'''
const Ccrp_store_model& get_ccrp_profile(){return ccrp_store_models[profile_index];}
bool g_release_alignment_valid=false;
double g_release_min_range=0,g_release_max_range=0;
'''+ '\n'.join(extract(n) for n in ['Vec3 get_world_wind_vector()','bool simulate_bomb_impact_point(','bool guided_release_in_zone(','bool compute_release_solution('])+r'''
bool solve(bool expectedSolution,bool expectedZone) {
 double error=0,ttg=0,range=0;bool zone=false;
 bool valid=compute_release_solution(&error,&ttg,&range,&zone);
 assert(valid==expectedSolution);if(valid)assert(zone==expectedZone);return zone;
}
int main(){
 for(profile_index=0;profile_index<ccrp_store_count;++profile_index) {
  const Store_kind kind=ccrp_store_models[profile_index].kind;
  if(kind!=Store_kind::jdam&&kind!=Store_kind::lgb&&kind!=Store_kind::wcmd) continue;
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
   aim(ballistic,ballistic);solve(true,false);assert(!g_release_alignment_valid);
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
  double mn,mx;bool ok=Jdam_envelope::lookup(p,Jdam_envelope::heights[h],Jdam_envelope::speeds[v],&mn,&mx);
  assert(ok);assert(mn==Jdam_envelope::minimum[p][h][v]);assert(mx==Jdam_envelope::maximum[p][h][v]);
 }
 double mn,mx;assert(!Jdam_envelope::lookup(0,399,200,&mn,&mx));assert(!Jdam_envelope::lookup(0,4000,99,&mn,&mx));
 assert(!Jdam_envelope::lookup(0,15001,200,&mn,&mx));assert(!Jdam_envelope::lookup(0,4000,551,&mn,&mx));
 assert(Jdam_envelope::lookup(1,4500,275,&mn,&mx));assert(std::fabs(mx-(12400+18050+12800+19700)/4.0)<1e-9);
 // v8's 135% ballistic ceiling rejected this ordinary authored-table solution.
 profile_index=1;vx_world=300;vz_world=0;target[0]=16000;target[1]=0;target[2]=0;solve(true,true);
 assert(g_release_max_range==18050&&g_release_min_range==2750);
 target[0]=18051;solve(true,false);target[0]=2749;solve(true,false);
 // Relative height and true airspeed rather than absolute altitude/groundspeed.
 position_world_cs.y=4500;target[1]=500;target[0]=16000;solve(true,true);
 wind_world_cs.x=100;solve(true,false);assert(g_release_max_range==7250);wind_world_cs.x=0;
 bool aligned=false;
 assert(!guided_release_in_zone(Store_kind::unguided,5000,5000,0,75,&aligned));
 assert(!guided_release_in_zone(Store_kind::missile_ir,5000,5000,0,75,&aligned));
 assert(!guided_release_in_zone(Store_kind::lgb,5000,5000,-1,75,&aligned));
 assert(!guided_release_in_zone(Store_kind::lgb,5000,5000,0,75,nullptr));
 assert(guided_release_in_zone(Store_kind::wcmd,5000,5000,0,150,&aligned)&&aligned);
 position_world_cs={0,4000,0};wind_world_cs={0,0,0};vx_world=200;vy_world=0;vz_world=0;
 // unguided: releases only as the predicted impact reaches the target, and only on the line.
 for(profile_index=6;profile_index<=7;++profile_index){
  assert(ccrp_store_models[profile_index].kind==Store_kind::unguided);
  Vec3 impact{};double time=0;
  assert(simulate_bomb_impact_point(get_ccrp_profile(),position_world_cs,{vx_world,0,0},wind_world_cs,0,&impact,&time));
  target[0]=impact.x;target[1]=0;target[2]=0;solve(true,true);
  target[0]=impact.x+ccrp_store_models[profile_index].release_window_meters+5;solve(true,false); // early
  target[0]=impact.x-ccrp_store_models[profile_index].release_window_meters-5;solve(true,false); // late
  target[0]=impact.x;target[2]=unguided_max_cross_track_meters+5;solve(true,false);assert(!g_release_alignment_valid);
  // Drag must shorten the throw against a vacuum drop from the same point.
  const double vacuum=200*std::sqrt(2*4000/gravity_mps2);assert(impact.x<vacuum&&impact.x>vacuum*.8);
  bool entered=false;for(double r=impact.x+600;r>impact.x-600;r-=200/60.0){target[0]=r;target[2]=0;double e,t,d;bool z;assert(compute_release_solution(&e,&t,&d,&z));entered|=z;}
  assert(entered); // a 60 Hz approach at 200 m/s cannot jump the window
 }
 target[2]=0;
 // glide (JSOW): authored table in relative height and true airspeed; the C shares the A table.
 for(profile_index=9;profile_index<=11;++profile_index){
  assert(ccrp_store_models[profile_index].kind==Store_kind::glide);
  vx_world=250;double lo,hi;assert(Glide_envelope::lookup(ccrp_store_models[profile_index].envelope,4000,250,&lo,&hi));
  target[0]=(lo+hi)/2;solve(true,true);assert(g_release_min_range==lo&&g_release_max_range==hi);
  target[0]=hi+10;solve(true,false);target[0]=lo-10;solve(true,false);
 }
 {double lo,hi;assert(Glide_envelope::lookup(0,4000,250,&lo,&hi));assert(lo==11000&&hi==52000);
  assert(!Glide_envelope::lookup(0,300,120,&lo,&hi)); // no release at that speed in the stock table
  assert(!Glide_envelope::lookup(0,299,250,&lo,&hi));assert(!Glide_envelope::lookup(2,4000,250,&lo,&hi));}
 // Missiles: slant range and seeker cone, no ballistic solution.
 vx_world=200;
 for(profile_index=12;profile_index<=13;++profile_index){
  const Ccrp_store_model& m=ccrp_store_models[profile_index];assert(store_is_missile(m.kind));
  position_world_cs={0,1000,0};
  target[0]=5000;target[2]=0;solve(true,true);
  target[0]=std::sqrt(m.max_range_meters*m.max_range_meters-1e6)+50;solve(true,false); // too far
  target[0]=200;solve(true,false);assert(!g_release_alignment_valid); // steeply below: outside the seeker cone
  target[0]=5000;target[2]=5000;solve(true,false);assert(!g_release_alignment_valid); // 45 degrees off the nose
  target[2]=0;
 }
 assert(store_needs_laser(Store_kind::lgb)&&store_needs_laser(Store_kind::missile_laser));
 assert(!store_needs_laser(Store_kind::jdam)&&!store_needs_laser(Store_kind::unguided)&&!store_needs_laser(Store_kind::wcmd)&&!store_needs_laser(Store_kind::glide)&&!store_needs_laser(Store_kind::missile_ir));
 assert(ccrp_store_count==14);
 puts("PASS: all 14 stores: unguided CCRP window, JSOW tables, missile range/cone; ballistic integration + release solution for JDAM/LGB/WCMD, both world headings; authored JDAM nodes/interpolation/domain, earlier release, relative height/wind, guided correction, continuous approach, too far/close/off-axis/behind/invalid/low speed/drop inhibits");
}
'''
code=code.replace('JDAM_HEADER',str(repo/'EFM Source Code/Avionics/JdamEnvelope.h').replace('\\','/'))
p=out/'zone_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/zone_tests.obj" /Fe"{out}/zone_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'zone_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
