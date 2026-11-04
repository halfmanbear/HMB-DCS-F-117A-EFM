"""Check the production leading-edge vortex against the airframe and Vermeersch (MIT, 1993).

Usage: python tests/lerx_vortex_regression.py <output folder>
"""
from pathlib import Path
import subprocess
import sys

repo = Path(__file__).resolve().parents[1]
src = repo / 'EFM Source Code'
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = (src / 'F_117_NightHawk.cpp').read_text(encoding='utf-8-sig')
api = (src / 'include/FM/wHumanCustomPhysicsAPI.h').read_text(encoding='utf-8-sig', errors='replace')


def extract(text, signature, opener='{'):
    start = text.index(signature)
    end = text.index(opener, start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + ';'


structs = (extract(api, 'struct LERX_vortex_spline_point') + '\n'
           'const unsigned LERX_vortex_spline_point_size = sizeof(LERX_vortex_spline_point);\n'
           + extract(api, 'struct LERX_vortex\n') + '\n')

code = r"""
#define NOMINMAX
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
namespace F117 {
double DeltaTime=0,alpha_DEG=0,beta_DEG=0,totalVelocity_MPS=0,ambientDensity_KgPerM3=1.225,altitude_m=0;
bool weight_on_wheels=false;
}
""" + structs + extract(source, 'bool ed_fm_LERX_vortex_update(unsigned idx, LERX_vortex& out)')[:-1] + r"""

const float kNoseX=10.02f,kLength=19.43f,kApexX=9.95f,kSemispan=0.418f,kTipX=-6.3f,kTeX=-7.0f;

LERX_vortex fly(unsigned idx,double aoa,double v,double beta=0,double rho=1.225,double alt=0){
    F117::alpha_DEG=aoa;F117::totalVelocity_MPS=v;F117::beta_DEG=beta;
    F117::ambientDensity_KgPerM3=rho;F117::altitude_m=alt;F117::weight_on_wheels=false;
    LERX_vortex o;assert(ed_fm_LERX_vortex_update(idx,o));return o;
}
// x of the point explosion_start metres along the undisturbed spline. Points
// past breakdown are lifted and swirled, but x is never displaced and vel is
// the undisturbed tangent, so measure with those.
float burst_x(const LERX_vortex& o){
    const auto* p=o.spline;float d=0;
    for(unsigned i=1;i<o.spline_points_count;++i){
        float dx=p[i].pos[0]-p[i-1].pos[0];
        float seg=std::fabs(dx)*2.0f/(std::fabs(p[i].vel[0])+std::fabs(p[i-1].vel[0]));
        if(d+seg>=o.explosion_start)return p[i-1].pos[0]+(p[i].pos[0]-p[i-1].pos[0])*(o.explosion_start-d)/seg;
        d+=seg;
    }
    return p[o.spline_points_count-1].pos[0]-(o.explosion_start-d);
}
int main(){
    LERX_vortex o;
    assert(!ed_fm_LERX_vortex_update(2,o));
    F117::weight_on_wheels=true;F117::totalVelocity_MPS=150;F117::alpha_DEG=15;
    assert(ed_fm_LERX_vortex_update(0,o)&&o.spline==nullptr);
    assert(fly(0,2,150).spline==nullptr);                  // no vortex at low AoA
    assert(fly(0,12,150,0,0.41,10500).spline==nullptr);   // too high to condense
    assert(fly(0,12,30).spline==nullptr);                 // too slow

    // Path hugs the leading edge from the apex region to the tip, on the correct side,
    // clear of the skin, with per-point opacity in 0..1 that is zero at the nose.
    for(double aoa:{8.0,12.0,18.0}){
        for(unsigned idx:{0u,1u}){
            o=fly(idx,aoa,160);
            assert(o.version==1&&o.spline_points_count==12&&o.opacity>0&&o.opacity<0.25f);
            const auto* p=o.spline;
            assert(p[0].pos[0]>8.0f&&p[0].opacity<0.01f);
            assert(p[11].pos[0]<kTeX);
            for(unsigned i=0;i<o.spline_points_count;++i){
                const float x=p[i].pos[0],z=p[i].pos[2]*(idx==0?-1.f:1.f);
                const float s=kSemispan*(kApexX-std::max(x,kTipX));
                assert(p[i].pos[2]*(idx==0?1.f:-1.f)<0);   // left is -z, right is +z
                assert(z>0.76f*s&&z<0.92f*s);
                assert(p[i].opacity>=0&&p[i].opacity<=1&&p[i].radius>=0);
                if(x>kTeX)assert(p[i].pos[1]-p[i].radius>-0.26f); // above the leading edge plane
                if(x<5.0f&&x>kTeX)assert(p[i].opacity>0.3f);       // dense over the wing
                const float vm=std::sqrt(p[i].vel[0]*p[i].vel[0]+p[i].vel[1]*p[i].vel[1]+p[i].vel[2]*p[i].vel[2]);
                assert(std::fabs(vm-1)<1e-3&&p[i].vel[0]<0);
            }
        }
    }

    // Breakdown position (Fig. 6.25): X/L from the nose. DeltaTime is 0, so the
    // unsteady wander is at its zero phase for the left vortex.
    const double table[][2]={{20,0.775},{25,0.68},{30,0.555}};
    for(auto& row:table){
        o=fly(0,row[0],120);
        const float expect=kNoseX-(float)row[1]*kLength;
        const float got=burst_x(o);
        std::fprintf(stderr,"AoA %4.1f breakdown x %.2f m (thesis %.2f)\n",row[0],got,expect);
        assert(std::fabs(got-expect)<0.15f);
        assert(got>kTeX);                                   // over the wing at 20 deg and above
    }
    assert(burst_x(fly(0,15,150))<kTeX);                    // just aft of the trailing edge at 15 deg
    assert(burst_x(fly(0,15,150))>kTeX-1.0f);
    assert(burst_x(fly(0,10,150))<-9.0f);                  // far downstream, off the end of the spline

    // Vapour needs suction: a slow approach shows less than a fast, loaded turn.
    assert(fly(0,10,75).opacity<0.6f*fly(0,12,180).opacity);
    // The windward vortex is stronger and breaks down further forward.
    LERX_vortex wl=fly(0,22,150,4),wr=fly(1,22,150,4);
    assert(wr.opacity>wl.opacity&&burst_x(wr)>burst_x(wl));
    std::puts("PASS: LE vortex path, surface clearance, opacity, condensation and breakdown location");
}
"""
p = out / 'lerx_vortex_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++14 /W4 "{p}" /Fo"{out}/lerx_vortex_tests.obj" /Fe"{out}/lerx_vortex_tests.exe"'
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'lerx_vortex_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
