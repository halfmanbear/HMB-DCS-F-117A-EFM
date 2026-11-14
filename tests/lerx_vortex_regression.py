"""Check the production leading-edge vortex against the airframe and Vermeersch (MIT, 1993).

Usage: python tests/lerx_vortex_regression.py <output folder>
"""
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.sources import load_source  # noqa: E402

repo = Path(__file__).resolve().parents[1]
src = repo / 'EFM Source Code'
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = load_source()
lerx_unit = '\n'.join(line for line in load_source('Effects/LerxVortex.cpp').splitlines()
                      if not line.startswith('#include'))
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
double delta_time=0,alpha_deg=0,beta_deg=0,total_velocity_mps=0,ambient_density_kg_per_m3=1.225,altitude_m=0;
double az=0,pitch_angle=0,roll_angle=0;
bool weight_on_wheels=false;
}
""" + structs + lerx_unit + r"""

const float kNoseX=10.02f,kLength=19.43f,kApexX=9.95f,kSemispan=0.418f,kTipX=-6.3f,kTeX=-7.0f;

// nz defaults to a hard pull; wings level, so the body acceleration is (nz - 1) g.
LERX_vortex fly(unsigned idx,double aoa,double v,double beta=0,double rho=1.225,double alt=0,double nz=4){
    F117::az=(nz-1)*9.80665;F117::pitch_angle=0;F117::roll_angle=0;
    F117::alpha_deg=aoa;F117::total_velocity_mps=v;F117::beta_deg=beta;
    F117::ambient_density_kg_per_m3=rho;F117::altitude_m=alt;F117::weight_on_wheels=false;
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
    F117::weight_on_wheels=true;F117::total_velocity_mps=150;F117::alpha_deg=15;
    assert(ed_fm_LERX_vortex_update(0,o)&&o.spline==nullptr);
    assert(fly(0,2,150).spline==nullptr);                  // no vortex at low AoA
    assert(fly(0,12,150,0,0.41,10500).spline==nullptr);   // too high to condense
    assert(fly(0,12,30).spline==nullptr);                 // too slow

    // Path hugs the leading edge from the apex region to the tip, on the correct side,
    // clear of the skin, with per-point opacity in 0..1 that is zero at the nose.
    for(double aoa:{8.0,9.0,10.0}){
        for(unsigned idx:{0u,1u}){
            o=fly(idx,aoa,160);
            assert(o.version==1&&o.spline_points_count==20&&o.opacity>0&&o.opacity<=0.5f);
            const auto* p=o.spline;
            assert(p[0].pos[0]>8.0f&&p[0].opacity<0.01f);
            // Short, straight tail: ends within 1 m behind the tip trailing edge, faded
            // out, with no bend up into the freestream.
            const auto& tail=p[o.spline_points_count-1];
            assert(tail.pos[0]<kTeX&&tail.pos[0]>kTeX-1.0f&&tail.opacity<0.01f);
            assert(std::fabs(tail.vel[1])<1e-3f);
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

    assert(burst_x(fly(0,10,150))<-9.0f);                  // far downstream, off the end of the spline

    // Moderate pull: thin streaks. Hard, fast pull: a wide, billowing, denser tube
    // whose radius is 15-22% of the local semispan, still clear of the skin.
    auto mean_rel_radius=[](const LERX_vortex& o,float x_aft){
        float sum=0;int n=0;
        for(unsigned i=0;i<o.spline_points_count;++i){
            const float x=o.spline[i].pos[0];
            if(x>5.0f||x<x_aft)continue;
            sum+=o.spline[i].radius/(kSemispan*(kApexX-std::max(x,kTipX)));++n;
        }
        return sum/n;
    };
    const LERX_vortex thin=fly(0,8.5,140);
    const float thin_r=mean_rel_radius(thin,kTeX);
    assert(thin_r>0.05f&&thin_r<0.09f);
    // Near the AoA limit the overwing cloud replaces the vortices: they fade
    // from 10 deg and are gone by 11 deg.
    assert(fly(0,10,180).opacity>fly(0,10.5,180).opacity&&fly(0,10.5,180).spline!=nullptr);
    for(double aoa:{11.0,11.5,12.6,14.0,20.0,30.0})
        for(unsigned idx:{0u,1u})
            assert(fly(idx,aoa,180).spline==nullptr);
    // Below about 3 g (a slow, high-AoA pull, no overwing cloud) the vortices stay.
    for(double aoa:{11.5,12.6,14.0})
        assert(fly(0,aoa,130,0,1.225,0,2.5).spline!=nullptr&&fly(0,aoa,130,0,1.225,0,2.5).opacity>fly(0,10,130,0,1.225,0,2.5).opacity*0.9f);
    assert(fly(0,12,130,0,1.225,0,3.0).opacity<fly(0,12,130,0,1.225,0,2.5).opacity);
    assert(fly(0,12,130,0,1.225,0,3.3).spline==nullptr);
    for(double t:{0.0,0.4,1.3,2.9}){
        F117::delta_time=t;
        const LERX_vortex hard=fly(0,9.8,175);
        F117::delta_time=0;
        const float bx=burst_x(hard);
        const float hard_r=mean_rel_radius(hard,std::max(bx,kTeX));
        assert(hard_r>0.15f&&hard_r<0.24f&&hard_r>2.0f*thin_r);
        assert(hard.opacity>1.7f*thin.opacity&&hard.opacity<=0.5f);
        float rmin=10,rmax=0;                                // billowing, not a smooth cone
        for(unsigned i=0;i<hard.spline_points_count;++i){
            const auto& q=hard.spline[i];
            const float x=q.pos[0];
            if(x>5.0f||x<std::max(bx,kTeX))continue;
            const float rel=q.radius/(kSemispan*(kApexX-std::max(x,kTipX)));
            rmin=std::min(rmin,rel);rmax=std::max(rmax,rel);
            assert(q.pos[1]-q.radius>-0.26f);              // underside clear of the skin
            const float z=-q.pos[2],s=kSemispan*(kApexX-std::max(x,kTipX));
            assert(z>0.76f*s&&z<0.92f*s);
            assert(q.opacity>0.3f);
        }
        assert(rmax>1.25f*rmin);
    }

    // Vapour needs suction: a slow approach shows less than a fast, loaded turn.
    assert(fly(0,8,75).opacity<0.6f*fly(0,8,180).opacity);
    // The windward vortex is stronger and breaks down further forward.
    LERX_vortex wl=fly(0,9,150,4),wr=fly(1,9,150,4);
    assert(wr.opacity>wl.opacity&&burst_x(wr)>burst_x(wl));
    std::puts("PASS: LE vortex path, surface clearance, opacity, condensation, thin-to-billowing growth, cloud handover above 3 g and sideslip asymmetry");
}
"""
p = out / 'lerx_vortex_tests.cpp'
p.write_text(code, encoding='utf-8')
vc = r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
command = f'call "{vc}" >nul && cl /nologo /EHsc /std:c++17 /W4 "{p}" /Fo"{out}/lerx_vortex_tests.obj" /Fe"{out}/lerx_vortex_tests.exe"'
r = subprocess.run(command, shell=True, capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
r = subprocess.run([str(out / 'lerx_vortex_tests.exe')], capture_output=True, text=True)
print(r.stdout, r.stderr)
assert r.returncode == 0
