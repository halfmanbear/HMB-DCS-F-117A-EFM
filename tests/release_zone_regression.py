from pathlib import Path
import subprocess,sys
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
fm=(repo/'EFM Source Code/F_117_NightHawk.cpp').read_text(encoding='utf-8-sig')
def extract(name):
 start=fm.index(name);brace=fm.index('{',start);end=brace+1;depth=1
 while depth:depth+=(fm[end]=='{')-(fm[end]=='}');end+=1
 return fm[start:end]
constants=fm[fm.index('    enum class StoreKind'):fm.index('    std::string build_saved_games_dcs_path')]
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
 for(profileIndex=0;profileIndex<kCcrpStoreCount;++profileIndex) {
  const StoreKind kind=kCcrpStoreModels[profileIndex].kind;
  if(kind!=StoreKind::Jdam&&kind!=StoreKind::Lgb&&kind!=StoreKind::Wcmd) continue;
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
 assert(!guided_release_in_zone(StoreKind::Unguided,5000,5000,0,75,&aligned));
 assert(!guided_release_in_zone(StoreKind::MissileIr,5000,5000,0,75,&aligned));
 assert(!guided_release_in_zone(StoreKind::Lgb,5000,5000,-1,75,&aligned));
 assert(!guided_release_in_zone(StoreKind::Lgb,5000,5000,0,75,nullptr));
 assert(guided_release_in_zone(StoreKind::Wcmd,5000,5000,0,150,&aligned)&&aligned);
 position_world_cs={0,4000,0};wind_world_cs={0,0,0};vx_world=200;vy_world=0;vz_world=0;
 // Unguided: releases only as the predicted impact reaches the target, and only on the line.
 for(profileIndex=6;profileIndex<=7;++profileIndex){
  assert(kCcrpStoreModels[profileIndex].kind==StoreKind::Unguided);
  Vec3 impact{};double time=0;
  assert(simulate_bomb_impact_point(get_ccrp_profile(),position_world_cs,{vx_world,0,0},wind_world_cs,0,&impact,&time));
  target[0]=impact.x;target[1]=0;target[2]=0;solve(true,true);
  target[0]=impact.x+kCcrpStoreModels[profileIndex].releaseWindowMeters+5;solve(true,false); // early
  target[0]=impact.x-kCcrpStoreModels[profileIndex].releaseWindowMeters-5;solve(true,false); // late
  target[0]=impact.x;target[2]=kUnguidedMaxCrossTrackMeters+5;solve(true,false);assert(!g_releaseAlignmentValid);
  // Drag must shorten the throw against a vacuum drop from the same point.
  const double vacuum=200*std::sqrt(2*4000/kGravity_MPS2);assert(impact.x<vacuum&&impact.x>vacuum*.8);
  bool entered=false;for(double r=impact.x+600;r>impact.x-600;r-=200/60.0){target[0]=r;target[2]=0;double e,t,d;bool z;assert(compute_release_solution(&e,&t,&d,&z));entered|=z;}
  assert(entered); // a 60 Hz approach at 200 m/s cannot jump the window
 }
 target[2]=0;
 // Glide (JSOW): authored table in relative height and true airspeed; the C shares the A table.
 for(profileIndex=9;profileIndex<=11;++profileIndex){
  assert(kCcrpStoreModels[profileIndex].kind==StoreKind::Glide);
  vx_world=250;double lo,hi;assert(GlideEnvelope::lookup(kCcrpStoreModels[profileIndex].envelope,4000,250,&lo,&hi));
  target[0]=(lo+hi)/2;solve(true,true);assert(g_releaseMinRange==lo&&g_releaseMaxRange==hi);
  target[0]=hi+10;solve(true,false);target[0]=lo-10;solve(true,false);
 }
 {double lo,hi;assert(GlideEnvelope::lookup(0,4000,250,&lo,&hi));assert(lo==11000&&hi==52000);
  assert(!GlideEnvelope::lookup(0,300,120,&lo,&hi)); // no release at that speed in the stock table
  assert(!GlideEnvelope::lookup(0,299,250,&lo,&hi));assert(!GlideEnvelope::lookup(2,4000,250,&lo,&hi));}
 // Missiles: slant range and seeker cone, no ballistic solution.
 vx_world=200;
 for(profileIndex=12;profileIndex<=13;++profileIndex){
  const CcrpStoreModel& m=kCcrpStoreModels[profileIndex];assert(store_is_missile(m.kind));
  position_world_cs={0,1000,0};
  target[0]=5000;target[2]=0;solve(true,true);
  target[0]=std::sqrt(m.maxRangeMeters*m.maxRangeMeters-1e6)+50;solve(true,false); // too far
  target[0]=200;solve(true,false);assert(!g_releaseAlignmentValid); // steeply below: outside the seeker cone
  target[0]=5000;target[2]=5000;solve(true,false);assert(!g_releaseAlignmentValid); // 45 degrees off the nose
  target[2]=0;
 }
 assert(store_needs_laser(StoreKind::Lgb)&&store_needs_laser(StoreKind::MissileLaser));
 assert(!store_needs_laser(StoreKind::Jdam)&&!store_needs_laser(StoreKind::Unguided)&&!store_needs_laser(StoreKind::Wcmd)&&!store_needs_laser(StoreKind::Glide)&&!store_needs_laser(StoreKind::MissileIr));
 assert(kCcrpStoreCount==14);
 puts("PASS: all 14 stores: unguided CCRP window, JSOW tables, missile range/cone; ballistic integration + release solution for JDAM/LGB/WCMD, both world headings; authored JDAM nodes/interpolation/domain, earlier release, relative height/wind, guided correction, continuous approach, too far/close/off-axis/behind/invalid/low speed/drop inhibits");
}
'''
code=code.replace('JDAM_HEADER',str(repo/'EFM Source Code/Avionics/JdamEnvelope.h').replace('\\','/'))
p=out/'zone_tests.cpp';p.write_text(code,encoding='utf-8')
vc=r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
cmd=f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 "{p}" /Fo"{out}/zone_tests.obj" /Fe"{out}/zone_tests.exe"'
r=subprocess.run(cmd,shell=True,capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
r=subprocess.run([str(out/'zone_tests.exe')],capture_output=True,text=True);print(r.stdout,r.stderr);assert r.returncode==0
