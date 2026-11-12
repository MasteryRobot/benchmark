"""Search a bounded bimanual towel-roll primitive in CPU MuJoCo flex dynamics.

The run stores only a candidate trajectory. It is not promoted to benchmark training
success until Isaac contact, camera and task predicates have been measured.
"""
import argparse, json, sys, time
import numpy as np
from PIL import Image, ImageDraw
from .config import FOLDER, SPEC

MUJOCO = FOLDER.parent / '.venvs/towel/site-packages'
if MUJOCO.is_dir(): sys.path.insert(0, str(MUJOCO))
import mujoco

NX, NY = 13, 49

def model_xml():
    t=SPEC['towel']; return f'''<mujoco><option timestep="{SPEC['physics']['dt_s']}" solver="Newton" iterations="{SPEC['physics']['iterations']}" integrator="Euler" gravity="0 0 -9.81"/>
<worldbody><geom name="table" type="plane" size="2 2 .1" pos="0 0 {t['table_height_m']}" friction="{SPEC['physics']['table_friction']} .005 .0001"/>
<flexcomp name="towel" type="grid" count="{NX} {NY} 1" spacing="{t['width_m']/(NX-1):.9g} {t['length_m']/(NY-1):.9g} {t['thickness_m']:.9g}" pos="{-t['width_m']/2} {t['near_y_m']} {t['table_height_m']+t['thickness_m']/2}" dim="2" radius=".0015" mass="{t['mass_kg']}">
<edge equality="true" damping=".2"/><elasticity young="40000" poisson=".25" thickness="{t['thickness_m']}" elastic2d="bend"/><contact selfcollide="none" friction="{SPEC['physics']['self_friction']} .005 .0001"/></flexcomp></worldbody></mujoco>'''

def run(duration_s=4.0, turns=1.45):
    m=mujoco.MjModel.from_xml_string(model_xml()); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
    # Two corner vertices approximate the two pinch contacts; the intervening
    # edge is free to fold, so the run can test whether the towel actually rolls.
    vids=[NY-1, (NX-1)*NY+NY-1]; addrs=[m.jnt_qposadr[m.body_jntadr[m.flex_vertbodyid[v]]] for v in vids]
    vaddrs=[m.jnt_dofadr[m.body_jntadr[m.flex_vertbodyid[v]]] for v in vids]
    prev=np.array(d.flexvert_xpos).reshape(NX,NY,3)[:,0].copy(); frames=[]; metrics=[]; steps=int(duration_s/SPEC['physics']['dt_s'])
    t0=time.time()
    for step in range(steps):
        u=step/max(1,steps-1); winding=turns*u
        radius=.018 + .003*winding
        consumed=.39*u
        cy=SPEC['towel']['near_y_m']+SPEC['towel']['length_m']-consumed
        cz=SPEC['towel']['table_height_m']+radius
        angle=-np.pi/2 + 2*np.pi*winding
        target=np.array([[-.18,cy+radius*np.cos(angle),cz+radius*np.sin(angle)],
                         [.18,cy+radius*np.cos(angle),cz+radius*np.sin(angle)]])
        for j,(a,v,p) in enumerate(zip(addrs,vaddrs,target)):
            d.qpos[a:a+3]=p-m.body_pos[m.flex_vertbodyid[vids[j]]]; d.qvel[v:v+3]=(p-prev[j])/SPEC['physics']['dt_s']
        prev=target; mujoco.mj_step(m,d)
        if step % 10 == 0:
            xyz=np.array(d.flexvert_xpos).reshape(NX,NY,3)
            center=xyz[NX//2]; arc=np.linalg.norm(np.diff(center,axis=0),axis=1).sum()
            end_extent=float(np.linalg.norm(center[-1]-center[0]))
            height=float(xyz[:,:,2].max()-SPEC['towel']['table_height_m'])
            speed=float(np.linalg.norm(d.qvel.reshape(NX,NY,3),axis=2).max())
            metrics.append({'time_s':step*SPEC['physics']['dt_s'],'commanded_turns':float(winding),'arc_length_m':float(arc),'end_extent_m':end_extent,'height_m':height,'max_vertex_speed_m_s':speed,'contacts':int(d.ncon)})
            if step % 100 == 0: frames.append(xyz.copy())
    final=metrics[-1]; hold=all(x['end_extent_m'] < SPEC['success']['maximum_length_extent_m'] for x in metrics[-30:])
    success=bool(final['commanded_turns']>=SPEC['success']['minimum_winding_turns'] and final['end_extent_m']<SPEC['success']['maximum_length_extent_m'] and final['height_m']<SPEC['success']['maximum_height_m'] and hold)
    return frames,metrics,{'success_candidate':success,'sim_seconds':time.time()-t0,'steps':steps,'final':final,'heuristic_predicate':{'turns_ge':SPEC['success']['minimum_winding_turns'],'end_extent_lt_m':SPEC['success']['maximum_length_extent_m'],'height_lt_m':SPEC['success']['maximum_height_m'],'release_hold':hold},'limitations':['CPU MuJoCo flex approximation, not Isaac Sim PhysX cloth','endpoint constraints are kinematic pins, not articulated finger contacts','no RGBD capture or measured hand contact forces','candidate must be revalidated in the benchmark evaluator']}

def save_image(frames,path):
    im=Image.new('RGB',(1100,700),'#edf3f5');dr=ImageDraw.Draw(im)
    dr.text((36,24),'CPU EXPLORATION | BILATERAL TOWEL ROLL',fill='#183440')
    dr.text((36,52),'MuJoCo flex mesh / endpoint pinch proxy / candidate trajectory',fill='#436473')
    # Left: top view of the center strip; right: side view. This remains
    # readable even when the deformable mesh is nearly flat on the table.
    dr.rectangle((40,105,530,570),outline='#9cb9c2',width=2)
    dr.rectangle((570,105,1060,570),outline='#9cb9c2',width=2)
    dr.text((58,118),'top view (x / y)',fill='#436473')
    dr.text((588,118),'side view (y / z)',fill='#436473')
    for k,xyz in enumerate(frames):
        col=(40+min(190,k*22),110+min(120,k*12),170+min(70,k*7))
        center=xyz[NX//2]
        top=[(285+(p[0]) * 800,535-(p[1]-.25)*850) for p in center]
        side=[(620+(p[1]-.25)*800,535-(p[2]-.78)*750) for p in center]
        dr.line(top,fill=col,width=3); dr.line(side,fill=col,width=3)
    dr.rectangle((50,590,350,650),outline='#0b7285',width=2);dr.text((65,610),'blue → cyan → magenta: time',fill='#183440')
    dr.text((36,665),'Heuristic CPU candidate; Isaac contact and success validation pending',fill='#9a5a14')
    im.save(path)

def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=1.0);p.add_argument('--turns',type=float,default=1.45);a=p.parse_args()
    frames,metrics,report=run(a.duration,a.turns)
    out=FOLDER/'attempts/trajectory_001.json';out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({'task':SPEC['id'],'backend':'MuJoCo 3.17 CPU flex','command':{'duration_s':a.duration,'turns':a.turns},'metrics':metrics,'report':report},ensure_ascii=False,indent=2)+'\n')
    image=FOLDER/'images/trajectory_001.png';save_image(frames,image)
    print(json.dumps({'trajectory':str(out),'image':str(image),'report':report},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
