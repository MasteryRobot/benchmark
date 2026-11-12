"""Export a verified native-PhysX episode as aligned 30 Hz state/action data."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from task_runtime import load_task, runtime_config
from verify_phone_episode import verify

def main():
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    verdict=verify(a.episode)
    if not verdict['success']:raise RuntimeError('Only independently verified successes can be exported')
    r=json.loads((a.episode/'report.json').read_text());cfg=runtime_config(load_task(r['task']))
    with np.load(a.episode/'measured_states.npz') as f:d={k:f[k] for k in f.files}
    start=np.arange(cfg['settle_steps'],len(d['time_s'])-1,cfg['control_decimation'])
    end=np.minimum(start+cfg['control_decimation'],len(d['time_s'])-1)
    for s,e in zip(start,end):
        if not np.allclose(d['targets'][s+1:e+1],d['targets'][s+1],atol=1e-7):raise ValueError('Action changed inside a transition')
    a.output.mkdir(parents=True,exist_ok=False)
    terminal=np.zeros(len(start),dtype=bool);terminal[-1]=True
    payload={
        'timestamp':d['time_s'][start]-d['time_s'][start[0]],
        'observation.state':d['joints'][start],
        'action':d['targets'][start+1],
        'next.observation.state':d['joints'][end],
        'next.reward':terminal.astype('float32'),
        'next.done':terminal,
        'next.success':terminal,
        'transition_dt_s':d['time_s'][end]-d['time_s'][start],
        'physics_start_index':start,'physics_end_index':end,
        'joint_names':np.array(r['joint_names']),
        'diagnostic.object_pose_xyzw':d['objects'][start,0],
        'diagnostic.next_object_pose_xyzw':d['objects'][end,0],
        'diagnostic.contact_forces_n':(d['normal_forces']+d['friction_forces'])[start],
    }
    np.savez_compressed(a.output/'episode_000000.npz',**payload)
    check=np.load(a.output/'episode_000000.npz')
    if not np.array_equal(check['action'],payload['action']) or check['observation.state'].shape!=(len(start),54):raise RuntimeError('Dataset readback mismatch')
    info={'task':r['task'],'episodes':1,'success':True,'frames':len(start),'nominal_fps':30,'state_dim':54,'action_representation':'absolute joint position targets in radians','physics_backend':r['backend'],'raw_source':str(a.episode.resolve()),'rgbd_collected':False,'video_source':'Offline rendering of measured rigid-body poses; not Isaac RTX camera capture','verification':verdict,'format':'NPZ aligned transitions, not LeRobot','initial_noop_physics_step_excluded':True,'dataset_sha256':hashlib.sha256((a.output/'episode_000000.npz').read_bytes()).hexdigest()}
    (a.output/'metadata.json').write_text(json.dumps(info,indent=2))
    print(json.dumps({'output':str(a.output),'frames':len(start),'success':True}))

if __name__=='__main__':main()
