"""Verify physical logs and derive palm-relative metrics; never certify tool use."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from robot_model import Robot

ROOT=Path(__file__).resolve().parent


def poses(values):
    result=np.tile(np.eye(4),(len(values),1,1))
    result[:,:3,:3]=Rotation.from_quat(values[:,3:]).as_matrix()
    result[:,:3,3]=values[:,:3]
    return result


def verify(folder, save_features=True):
    report=json.loads((folder/'report.json').read_text())
    if not (folder/'measured_states.npz').exists():
        return {'task':report['task'],'status':'NO_TRAJECTORY','functional_success':False}
    data=np.load(folder/'measured_states.npz',allow_pickle=False)
    if 'objects' not in data:
        return {'task':report['task'],'status':'DEFORMABLE_REQUIRES_SEPARATE_VALIDATOR','functional_success':False}
    n=len(data['time_s']); names=report['joint_names']; contacts=[]
    checks={'gpu_backend':str(report.get('device','')).split(':')[0]=='CUDA','positive_frame_count':n>0,
        'joint_shape':data['joints'].shape==(n,54),
        'target_shape':data['targets'].shape==(n,54),
        'all_numeric_finite':all(np.isfinite(data[k]).all() for k in data.files if data[k].dtype.kind not in ('U','S')),
        'fixed_timestep':bool(np.allclose(np.diff(data['time_s']),report['physics_dt_s'],atol=1e-9)),
        'reported_frame_count_matches':n==report['recorded_physics_frames'],
        'artifact_hashes':all(hashlib.sha256((folder/k).read_bytes()).hexdigest()==v for k,v in report['artifacts_sha256'].items())}
    for line in (folder/'contacts.jsonl').open(): contacts.append(json.loads(line))
    checks['contact_frames']=len(contacts)==n
    checks['contact_time_alignment']=len(contacts)==n and bool(np.allclose([x['time_s'] for x in contacts],data['time_s']))
    checks['contact_fields_finite']=all(len(p)==10 and np.isfinite(p).all() for r in contacts for p in r['contacts'])
    model=Robot(ROOT/'assets/dual_arm/robot.urdf'); zero=model.fk()
    obj=np.stack([poses(data['objects'][:,i]) for i in range(data['objects'].shape[1])],axis=1)
    palms={}; relatives={}; metrics={}
    forces=data['normal_forces']+data['friction_forces']
    for hand in ['left','right']:
        name=hand+'_hand_base';group=model.groups[name]
        fixed=np.linalg.inv(zero[group])@zero[name]
        index=report['body_names'].index(group)
        palms[hand]=poses(data['links'][:,index])@fixed
        relatives[hand]=np.linalg.inv(palms[hand])[:,None]@obj
        delta=np.transpose(relatives[hand][0,:,:3,:3],(0,2,1))[None]@relatives[hand][:,:,:3,:3]
        angles=np.degrees(np.arccos(np.clip((np.trace(delta,axis1=-2,axis2=-1)-1)/2,-1,1)))
        metrics[hand]={'max_relative_rotation_deg':angles.max(axis=0).tolist(),
                      'scope':'All frames, including ungrasped motion; does not establish in-hand regrasp'}
    initial=np.asarray(report.get('settled_object_poses_xyzw',data['objects'][0].tolist()))
    lift=(data['objects'][:,:,2]-initial[None,:,2])
    quality_flags=[]
    max_force=float(np.linalg.norm(forces,axis=-1).max())
    tracking=float(np.max(np.abs(data['targets']-data['joints'])))
    if report.get('status')=='PHYSICS_FAILED':quality_flags.append('physical_run_failed_or_aborted')
    if max_force>100:quality_flags.append('large_hand_contact_force_requires_inspection')
    if tracking>.25:quality_flags.append('large_joint_tracking_error_requires_inspection')
    baseline_replay=None
    spec_path=folder/'task_contract.json'
    if not spec_path.exists(): spec_path=ROOT/'tasks'/report['task']/'task.json'
    if spec_path.exists():
        from task_runtime import TaskEvaluator
        spec=json.loads(spec_path.read_text())
        if 'goal_pose_world' in spec:
            evaluator=TaskEvaluator(spec)
            anchor=poses(initial[:1])[0]
            evaluator.reset(anchor)
            hand=spec['preferred_hand'];model_paths=[]
            def walk(link):
                model_paths.append('/World/Robot/DualArm/Links/'+model.groups[link])
                for joint in model.children.get(link,[]):walk(joint.find('child').get('link'))
            walk(hand+'_hand_base')
            indices=[i for i,path in enumerate(report['contact_filter_paths'][0]) if path in model_paths]
            from task_runtime import runtime_config
            settle_steps=report.get('integration',{}).get('effective_runtime',runtime_config(spec))['settle_steps']
            last=None
            for i in range(settle_steps,n):
                last=evaluator.update(obj[i,0],data['object_velocities'][i,0,:3],
                    data['object_velocities'][i,0,3:],forces[i,0,indices],report['physics_dt_s'])
                if evaluator.done:break
            baseline_replay={'success':bool(last and last['success']), 'result':last,
                             'contract_source':str(spec_path),
                             'historical_contract_snapshot':(folder/'task_contract.json').exists()}
            checks['baseline_flag_matches_replay']=bool(report.get('baseline_predicate_success',False))==baseline_replay['success']
    # These engineering inspection thresholds are not calibrated task criteria.
    # Valid log structure is deliberately independent of plausible physical motion.
    results={'task':report['task'],'checked_utc':datetime.now(timezone.utc).isoformat(),
        'status':'VALID_DIAGNOSTIC' if all(checks.values()) else 'INVALID_DIAGNOSTIC',
        'checks':checks,'frames':n,'simulated_seconds':float(data['time_s'][-1]),
        'contact_point_samples':sum(len(x['contacts']) for x in contacts),
        'max_hand_force_n':max_force,'maximum_joint_tracking_error_rad':tracking,
        'quality_flags':quality_flags,
        'baseline_predicate_replay':baseline_replay,
        'quality_threshold_scope':'Inspection flags only; not calibrated acceptance or success criteria.',
        'maximum_lift_above_settled_m':lift.max(axis=0).tolist(),
        'relative_motion':metrics,'functional_success':False,'training_eligible':False,
        'missing_for_success':['task_specific_functional_contract','measured_use_outcome','five_camera_rgbd'],
        'normal_force_convention':'Signed scalar and world normal retained exactly as emitted by backend; magnitudes require abs(scalar).'}
    if save_features:
        np.savez_compressed(folder/'palm_relative_states.npz',time_s=data['time_s'],
                            left_palm_world=palms['left'],right_palm_world=palms['right'],
                            object_in_left_palm=relatives['left'],object_in_right_palm=relatives['right'])
    (folder/'verification.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    args=p.parse_args();rows=[]
    for path in args.root.rglob('report.json'):
        r=json.loads(path.read_text())
        if r.get('status') in ['STARTING','RUNNING']:continue
        rows.append(verify(path.parent))
    summary={'checked_utc':datetime.now(timezone.utc).isoformat(),'attempts':len(rows),
             'valid_diagnostics':sum(r['status']=='VALID_DIAGNOSTIC' for r in rows),
             'total_physics_frames':sum(r.get('frames',0) for r in rows),
             'functional_successes':0,'results':rows}
    (args.root/'verification_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='results'},ensure_ascii=False))
