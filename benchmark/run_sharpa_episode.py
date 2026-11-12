"""Run one new multi-object scenario in Isaac Sim; blocked runs return code 2.

No actions: repeated reset/contact smoke. Actions: replay user joint commands.
Neither implements an autonomous manipulation policy. All provenance is local
and provisional until release sources have been independently checked.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np
from sharpa.contract import ROOT, load_scenario, rotation
from robot_model import Robot
from build_sharpa_scenes import atomic_write


def serializable(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)


def fingerprint(spec, root=ROOT):
    names = ['tasks/workcell.usdc','assets/dual_arm/robot.urdf','poses.json','cameras.json',
             'robot_model.py','run_sharpa_episode.py','sharpa/contract.py','sharpa/evaluator.py',
             'sharpa/isaac_env.py',f'sharpa/scenes/{spec["id"]}/task.json',
             f'sharpa/scenes/{spec["id"]}/scene.usda']
    visuals=spec.get('visual_assets',{})
    if visuals:
        names.extend(['sharpa/visual_assets.py','assets/realistic/catalog.json'])
        catalog={a['id']:a for a in json.loads((Path(root)/'assets/realistic/catalog.json').read_text())['assets']}
        for visual in visuals.values():
            asset=catalog[visual['asset_id']]
            names.extend([asset['usd'],asset['texture']])
    names=list(dict.fromkeys(names))
    return {name:hashlib.sha256((Path(root)/name).read_bytes()).hexdigest() for name in names}


def read_actions(path, spec):
    if path is None: return []
    doc = json.loads(Path(path).read_text())
    if doc.get('task') != spec['id'] or not isinstance(doc.get('actions'),list) or not doc['actions']:
        raise ValueError('Replay requires a matching task and a nonempty actions list')
    robot = Robot(ROOT/'assets/dual_arm/robot.urdf')
    limits = {j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper')))
              for j in robot.revolute}
    for a in doc['actions']:
        if not isinstance(a,dict) or set(a)-{'joint_targets_rad','control_steps','capture'}:
            raise ValueError('Unknown action fields')
        if not isinstance(a.get('joint_targets_rad',{}),dict):
            raise ValueError('joint_targets_rad must be a dictionary')
        n=a.get('control_steps',1)
        if isinstance(n,bool) or not isinstance(n,int) or n<1:
            raise ValueError('control_steps must be positive integer')
        capture=a.get('capture',False)
        if not isinstance(capture,bool) or (capture and spec['task_type']!='photograph'):
            raise ValueError('Only photography tasks support capture')
        for name,value in a.get('joint_targets_rad',{}).items():
            if (name not in limits or isinstance(value,bool) or not isinstance(value,(float,int))
                or not math.isfinite(value) or not limits[name][0]<=value<=limits[name][1]):
                raise ValueError(f'Invalid joint target: {name}')
    return doc['actions']


def reset_metrics(spec, observation, q0):
    rows=[]
    for b in spec['bodies']:
        if not b['dynamic']: continue
        s=observation['bodies'][b['id']]
        delta=np.asarray(s['position'])-b['position']
        linear=float(np.linalg.norm(s['linear_velocity_m_s']))
        angular=float(np.linalg.norm(s['angular_velocity_rad_s']))
        rows.append(dict(body=b['id'],position=s['position'],orientation_wxyz=s['orientation_wxyz'],
                         displacement_m=float(np.linalg.norm(delta)),linear_speed_m_s=linear,angular_speed_rad_s=angular,
                         stable=bool(np.linalg.norm(delta[:2])<.035 and abs(delta[2])<.035 and linear<.01 and angular<.05)))
    drift=float(np.max(np.abs(observation['joint_positions_rad']-q0)))
    return dict(bodies=rows,robot_joint_drift_rad=drift,
                stable=bool(drift<.02 and all(r['stable'] for r in rows)),
                physics_steps=spec['runtime']['settle_steps'],
                seconds=spec['runtime']['settle_steps']*spec['runtime']['physics_dt_s'])


def repeatability(resets):
    max_position=max_rotation=0.0
    for r in resets[1:]:
        for a,b in zip(resets[0]['bodies'],r['bodies']):
            if a['body']!=b['body']: raise ValueError('Reset body ordering changed')
            max_position=max(max_position,float(np.linalg.norm(np.asarray(a['position'])-b['position'])))
            relative=rotation(a['orientation_wxyz']).T@rotation(b['orientation_wxyz'])
            max_rotation=max(max_rotation,math.degrees(math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1)))))
    return dict(reset_repeatable=bool(max_position<.005 and max_rotation<3),
                reset_position_error_m=max_position,reset_rotation_error_deg=max_rotation)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task');p.add_argument('--actions',type=Path);p.add_argument('--gui',action='store_true')
    p.add_argument('--output',type=Path);p.add_argument('--reset-repeats',type=int,default=2)
    args=p.parse_args();spec=load_scenario(args.task)
    if args.reset_repeats<2:p.error('At least two resets are required')
    actions=read_actions(args.actions,spec)
    output=args.output or ROOT/'sharpa/scenes'/spec['id']/'runtime_validation.json'
    report=dict(task=spec['id'],checked_at_utc=datetime.now(timezone.utc).isoformat(),
                inputs_sha256=fingerprint(spec),runtime_status='NOT_RUN',task_success=False,
                source_status=spec['source_status'],coverage_level=spec['coverage_level'],
                mode='joint_replay' if actions else 'reset_contact_smoke',
                task_evaluation_status='NOT_IMPLEMENTED' if spec['task_type']=='scene_only' else 'LOCAL_APPROXIMATION',
                scope='Measured Isaac reset/contacts and user replay only; not autonomous policy validation.')
    app=env=None;code=0
    try:
        if importlib.util.find_spec('isaacsim') is None:
            report.update(runtime_status='BLOCKED',reason='Isaac Sim is not installed in the selected Python environment.')
            code=2
        else:
            from isaacsim import SimulationApp
            app=SimulationApp({'headless':not args.gui,'limit_cpu_threads':8})
            from sharpa.isaac_env import SharpaScenarioEnv
            env=SharpaScenarioEnv(spec['id'],app,render=args.gui)
            resets=[]
            for _ in range(args.reset_repeats):
                resets.append(reset_metrics(spec,env.reset(),env.q0))
            report.update(resets=resets,sensor_count=len(env.cameras),**repeatability(resets))
            traces=[];result=None
            # Brief control/contact probe; ballistic objects are only launched
            # at first step, not during the five-second settling interval.
            for action in actions or [dict(joint_targets_rad={},control_steps=1)]:
                for i in range(action.get('control_steps',1)):
                    obs,result=env.step(action.get('joint_targets_rad',{}),capture=action.get('capture',False) and i==0)
                    frame=obs.pop('captured_rgb',None)
                    if frame is not None:
                        from PIL import Image
                        capture_path=output.parent/f'capture_{env.capture_count:04d}.png'
                        capture_path.parent.mkdir(parents=True,exist_ok=True)
                        Image.fromarray(frame.astype(np.uint8)).save(capture_path)
                        obs['capture_file']=capture_path.name
                        obs['capture_sha256']=hashlib.sha256(capture_path.read_bytes()).hexdigest()
                    traces.append(dict(observation=obs,evaluation=result))
                    if result['terminated'] or result['truncated']:break
                if result['terminated'] or result['truncated']:break
            report.update(trace=traces,evaluation=result,task_success=result['success'])
            ok=all(r['stable'] for r in resets) and report['reset_repeatable']
            if not actions:ok=ok and not result['success'] and result['reason']=='running'
            report['runtime_status']='PASS' if ok else 'FAIL';code=0 if ok else 1
    except Exception as error:
        import traceback
        traceback.print_exc();report.update(runtime_status='ERROR',error_type=type(error).__name__,reason=str(error));code=1
    finally:
        for obj in (env,app):
            if obj is not None:
                try:obj.close()
                except Exception as error:
                    report.update(runtime_status='ERROR',cleanup_error=type(error).__name__);code=1
        atomic_write(output,json.dumps(report,ensure_ascii=False,indent=2,default=serializable)+'\n')
        print(json.dumps({k:report[k] for k in ('task','runtime_status','task_success')},ensure_ascii=False))
    return code


if __name__=='__main__':sys.exit(main())
