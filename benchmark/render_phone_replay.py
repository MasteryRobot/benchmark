"""Offline video of measured PhysX body states, explicitly labelled as replay."""
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw
from reports.scene_views_20261031 import render_views as rasterizer

ROOT=Path(__file__).resolve().parent
FFMPEG=ROOT/'.venvs/lerobot-v21/lib/python3.10/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'

def camera(eye,target,width,height):
    eye=np.array(eye); z=eye-np.array(target);z/=np.linalg.norm(z)
    x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    mat=np.eye(4);mat[:3,:3]=np.array([x,y,z]);mat[3,:3]=eye
    return {'world':mat,'focal':40.,'ha':36.,'va':36.*height/width}

def main():
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path)
    p.add_argument('--geometry',type=Path,default=ROOT/'collection_runs/first_success_20261031/replay_geometry.npz')
    p.add_argument('--output',type=Path);p.add_argument('--frame',type=float);p.add_argument('--fps',type=int,default=15)
    a=p.parse_args();report=json.loads((a.episode/'report.json').read_text())
    with np.load(a.episode/'measured_states.npz') as saved:
        states={key:saved[key] for key in saved.files}
    geom=np.load(a.geometry)
    names=list(geom['names']);ref=json.loads(str(geom['reference']))
    groups=[(str(name),geom['tri_'+str(i)],geom['col_'+str(i)]) for i,name in enumerate(names)]
    bodies={('/World/Robot/DualArm/Links/'+n):('links',i) for i,n in enumerate(report['body_names'])}
    bodies.update({n:('objects',i) for i,n in enumerate(report['object_paths'])})
    rasterizer.W=640;rasterizer.H=480
    cams=[camera([.8,1.25,1.6],[-.15,.35,.98],640,480),camera([-.5,.72,1.2],[-.25,.42,.88],640,480)]
    light=np.array([-.35,-.45,.8216])
    def transform_group(tris,col,mat):
        world=tris@mat[:3,:3].T+mat[:3,3]
        n=np.cross(world[:,1]-world[:,0],world[:,2]-world[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12)
        return world,np.clip(col*(.52+.48*np.abs(n@light))[:,None]*255,0,255).astype('uint8')
    static=[transform_group(tri,col,np.array(ref[name])) for name,tri,col in groups if name not in bodies]
    st=np.concatenate([x[0] for x in static]);sc=np.concatenate([x[1] for x in static])
    cache=[rasterizer.draw(st,sc,c) for c in cams]
    def draw(i):
        moving=[]
        for name,tri,col in groups:
            if name not in bodies:continue
            field,idx=bodies[name];pose=states[field][i,idx];mat=np.eye(4)
            mat[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();mat[:3,3]=pose[:3]
            moving.append(transform_group(tri,col,mat))
        tris=np.concatenate([x[0] for x in moving]);cols=np.concatenate([x[1] for x in moving])
        panels=[rasterizer.draw(tris,cols,c,b)[0] for c,b in zip(cams,cache)]
        canvas=Image.fromarray(np.concatenate(panels,axis=1));d=ImageDraw.Draw(canvas)
        d.rectangle([0,0,1280,45],fill=(20,25,35));d.text((12,8),'PHYSX MEASURED-STATE REPLAY | OFFLINE RGB | '+a.episode.name,fill='white')
        forces=states['normal_forces'][i]+states['friction_forces'][i]
        contact=int(np.count_nonzero(np.linalg.norm(forces[0],axis=-1)>=.001))
        d.text((12,26),'t=%.3fs   contact bodies=%d   phone z=%.4fm   success=%s'%(states['time_s'][i],contact,states['objects'][i,0,2],report.get('baseline_predicate_success',False)),fill='white')
        return canvas
    if a.frame is not None:
        i=int(np.argmin(abs(states['time_s']-a.frame)));output=a.output or a.episode/'replay_frame.png';draw(i).save(output);print(str(output));return
    output=a.output or a.episode/'measured_replay.mp4'
    sample_times=np.arange(states['time_s'][0],states['time_s'][-1],1/a.fps)
    indices=np.minimum(np.searchsorted(states['time_s'],sample_times),len(states['time_s'])-1)
    encoder=subprocess.Popen([str(FFMPEG),'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x480','-r',str(a.fps),'-i','-','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    try:
        for k,i in enumerate(indices):
            encoder.stdin.write(np.asarray(draw(i)).tobytes())
            if k%30==0:print('rendered',k,'of',len(indices),flush=True)
    finally:encoder.stdin.close()
    if encoder.wait()!=0:raise RuntimeError('Video encoder failed')
    print(json.dumps({'video':str(output),'frames':len(indices),'fps':a.fps,'source':'measured rigid-body states; offline software rendering'}))

if __name__=='__main__':main()
