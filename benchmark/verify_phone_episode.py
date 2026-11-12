"""Recompute the unchanged task predicate from saved physical measurements."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from task_runtime import TaskEvaluator, load_task, runtime_config

def verify(folder):
    folder=Path(folder);report=json.loads((folder/'report.json').read_text())
    task=load_task(report['task']);cfg=runtime_config(task)
    with np.load(folder/'measured_states.npz',allow_pickle=False) as saved:
        data={key:saved[key] for key in saved.files}
    for key in data:
        if data[key].dtype.kind in 'fi' and not np.isfinite(data[key]).all():
            raise ValueError('Nonfinite '+key)
    n=len(data['time_s']);dt=cfg['physics_dt_s'];settle=cfg['settle_steps']
    if n<=settle:raise ValueError('No controlled episode recorded')
    if not np.allclose(data['time_s'],np.arange(1,n+1)*dt,atol=1e-8):raise ValueError('Discontinuous times')
    def pose(row):
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(row[3:]).as_matrix();T[:3,3]=row[:3];return T
    ev=TaskEvaluator(task);ev.reset(pose(data['objects'][settle-1,0]))
    prefix='l_f_' if task['preferred_hand']=='left' else 'r_f_'
    wrist='Link7_R' if task['preferred_hand']=='left' else 'Link7_L'
    indices=[i for i,name in enumerate(report['contact_filter_paths'][0]) if name.split('/')[-1].startswith(prefix) or name.split('/')[-1]==wrist]
    result=None; success_index=None; max_contacts=0; max_lift=0.
    recorded=[json.loads(line)['evaluation'] for line in (folder/'contacts.jsonl').open()]
    if len(recorded)!=n:raise ValueError('Contact trace/state length mismatch')
    for i in range(settle,n):
        v=data['object_velocities'][i,0]
        contacts=(data['normal_forces'][i,0]+data['friction_forces'][i,0])[indices]
        result=ev.update(pose(data['objects'][i,0]),v[:3],v[3:],contacts,dt)
        reference=recorded[i]
        if reference is None or reference['success']!=result['success'] or reference['criteria']!=result['criteria']:
            raise ValueError('Recorded evaluator differs at frame '+str(i))
        max_contacts=max(max_contacts,result['contact_body_count']);max_lift=max(max_lift,result['lift_m'])
        if result['success']:success_index=i
        if ev.done:
            if i!=n-1:raise ValueError('Frames after terminal state')
            break
    success=bool(result and result['success'])
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [folder/'measured_states.npz',folder/'contacts.jsonl',folder/'scene.usda']}
    out={'task':report['task'],'verification':'PASS' if success else 'FAIL','success':success,'physics_backend':report['backend'],'physics_frames':n,'success_frame':success_index,'max_contact_bodies':max_contacts,'maximum_lift_m':max_lift,'final_evaluation':result,'unchanged_evaluator_sha256':hashlib.sha256(Path('task_runtime.py').read_bytes()).hexdigest(),'artifacts_sha256':hashes,'rgbd_collected':False,'scope':'Baseline physical task success only; native PhysX, no Isaac RTX capture'}
    (folder/'verification.json').write_text(json.dumps(out,indent=2))
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode');a=p.parse_args()
    print(json.dumps(verify(a.episode),indent=2))
