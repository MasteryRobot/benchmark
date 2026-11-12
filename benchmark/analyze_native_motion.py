"""Compare backend velocities to measured motion without substituting either."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def analyze(folder):
    report=json.loads((folder/'report.json').read_text())
    data=np.load(folder/'measured_states.npz',allow_pickle=False)
    t=data['time_s']; dt=np.diff(t)
    result={'task':report['task'],'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'Engineering consistency diagnostics only; original task evaluation is unchanged.',
        'velocity_units':'Native linear m/s and angular rad/s, world frame.',
        'reference':'ovphysx/docs/read_write/data_model.md', 'objects':[]}
    for i,path in enumerate(report['object_paths']):
        poses=data['objects'][:,i]; velocity=data['object_velocities'][1:,i]
        rotations=Rotation.from_quat(poses[:,3:])
        delta=(rotations[1:]*rotations[:-1].inv()).as_rotvec()/dt[:,None]
        linear=np.diff(poses[:,:3],axis=0)/dt[:,None]
        windows={}
        for name,mask in [('whole_episode',np.ones(len(dt),dtype=bool)),
                          ('last_two_seconds',t[1:]>t[-1]-2)]:
            windows[name]={}
            for kind,finite,native in [('linear',linear,velocity[:,:3]),('angular',delta,velocity[:,3:])]:
                f=finite[mask];v=native[mask]
                windows[name][kind]={'finite_difference_speed_median':float(np.median(np.linalg.norm(f,axis=1))),
                    'native_speed_median':float(np.median(np.linalg.norm(v,axis=1))),
                    'finite_difference_mean_vector':f.mean(axis=0).tolist(),
                    'native_mean_vector':v.mean(axis=0).tolist(),
                    'rms_vector_discrepancy':float(np.sqrt(np.mean(np.sum((f-v)**2,axis=1))))}
        result['objects'].append({'path':path,'windows':windows})
    mask=t[1:]>t[-1]-2
    fd=np.diff(data['joints'],axis=0)/dt[:,None]
    native=data['joint_velocities'][1:]
    rms=np.sqrt(np.mean((native[mask]-fd[mask])**2,axis=0))
    result['largest_joint_velocity_discrepancies']=[{'joint':report['joint_names'][i],
        'rms_rad_s':float(rms[i]),'native_mean_rad_s':float(native[mask,i].mean()),
        'finite_difference_mean_rad_s':float(fd[mask,i].mean())} for i in np.argsort(rms)[-8:][::-1]]
    (folder/'motion_consistency.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('attempt',type=Path);a=p.parse_args()
    r=analyze(a.attempt)
    print(json.dumps({'task':r['task'],'objects':r['objects']},indent=2))
