"""Bounded joint-only contact/curl probe. IK feasibility is not grasp success."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .config import FOLDER, SPEC
from .geometry import ArmIK, PALM_DOWN, initial_pose, robot_config


def generate(config):
    robot, _ = robot_config()
    limits = {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower','upper','velocity'))
              for j in robot.revolute}
    q, initial_errors = initial_pose()
    initial = dict(q)
    arms = {side: ArmIK(side) for side in ('left','right')}
    for side, arm in arms.items(): arm.q = np.array([q[n] for n in arm.names])
    fraction = config['max_joint_speed_fraction']
    if not 0 < fraction <= 1: raise ValueError('Invalid speed fraction')
    variant = config['variant']
    z = SPEC['towel']['table_height_m'] + variant['support_height_m'] + variant['thickness_m']/2
    y = variant['near_y_m'] - config['palm_to_nominal_pinch_m'][2]
    palm_z = z + config['palm_to_nominal_pinch_m'][0]
    # Nearby grasp geometry avoids the historical far-edge 30 cm IK residual.
    # Finger contact and collision-free paths still require runtime verification.
    waypoints = [('approach', y, palm_z+.06, 0., 0., 2.0),
                 ('descend', y, palm_z, 0., 0., 2.0),
                 ('pinch', y, palm_z, 0., config['finger_closed_fraction'], 2.0),
                 ('lift', y, palm_z+.025, 0., config['finger_closed_fraction'], 2.0),
                 ('curl', y+.09, palm_z+.031, 65., config['finger_closed_fraction'], 3.0),
                 ('release', y+.09, palm_z+.031, 65., 0., 2.0),
                 ('retreat', y+.09, palm_z+.09, 0., 0., 2.0)]
    samples = [{'time_s': 0., 'phase': 'settle', 'joint_targets_rad': dict(q)},
               {'time_s': 1.5, 'phase': 'settle', 'joint_targets_rad': dict(q)}]
    checks=[]; last_y, last_z, last_angle = .23, .91, 0.
    last_grip=0.
    for phase, target_y, target_z, angle, grip, nominal_duration in waypoints:
        n = math.ceil(nominal_duration*10)
        for k in range(1,n+1):
            u=k/n; u=u*u*(3-2*u)
            r=Rotation.from_euler('x',last_angle+(angle-last_angle)*u,degrees=True).as_matrix()@PALM_DOWN
            new=dict(q); errors={}
            for side,sign in [('left',-1),('right',1)]:
                p=np.array([variant['center_x_m']+sign*config['hand_x_m'],
                            last_y+(target_y-last_y)*u,last_z+(target_z-last_z)*u])
                update, error=arms[side].solve(p,r,trials=2)
                if (error['position_error_m'] > config['ik_position_tolerance_m']
                        or error['rotation_error_rad'] > config['ik_rotation_tolerance_rad']):
                    raise ValueError(f'Unreachable {phase}/{k}/{side}: {error}')
                new.update(update);errors[side]=error
            closure=last_grip+(grip-last_grip)*u
            for name,(lo,hi,_) in limits.items():
                if name.startswith('Joint'): continue
                base=initial[name]
                if name.endswith('1_1'):
                    target=-1.2 if name.startswith('l_') else 1.2
                elif name.endswith('_1'):
                    target=base
                else:
                    target=hi
                new[name]=float(np.clip(base+closure*(target-base),lo,hi))
            dt=max(nominal_duration/n,max(abs(new[name]-q[name])/(limits[name][2]*fraction) for name in q))
            samples.append({'time_s': samples[-1]['time_s']+dt,'phase':phase,'joint_targets_rad':new})
            checks.append(errors);q=new
        last_y,last_z,last_angle,last_grip=target_y,target_z,angle,grip
    samples.append({'time_s':samples[-1]['time_s']+1.,'phase':'observe','joint_targets_rad':dict(q)})
    return {'task':'bimanual_roll_towel','status':'KINEMATIC_PLAN_ONLY','initial_joint_positions_rad':initial,
            'samples':samples,'initial_ik':initial_errors,
            'max_position_error_m':max(e[h]['position_error_m'] for e in checks for h in arms),
            'max_rotation_error_rad':max(e[h]['rotation_error_rad'] for e in checks for h in arms),
            'command_type':'articulation_joint_position_targets_only','physics_success_verified':False,
            'collision_free_path_verified':False,'finger_contact_verified':False}


def validate(plan):
    robot,_=robot_config()
    limits={j.get('name'):tuple(float(j.find('limit').get(k)) for k in ('lower','upper','velocity')) for j in robot.revolute}
    prev=None
    for sample in plan['samples']:
        if set(sample)!= {'time_s','phase','joint_targets_rad'}: raise ValueError('Only joint targets are allowed')
        q=sample['joint_targets_rad'];t=sample['time_s']
        if set(q)!=set(limits) or not math.isfinite(t) or t<0: raise ValueError('Invalid sample')
        for name,value in q.items():
            lo,hi,v=limits[name]
            if not math.isfinite(value) or not lo<=value<=hi: raise ValueError('Joint limit violation')
            if prev:
                dt=t-prev['time_s']
                if dt<=0 or abs(value-prev['joint_targets_rad'][name])/dt > v+1e-8:
                    raise ValueError('Joint speed violation')
        prev=sample
    if len(plan['samples'])<2: raise ValueError('Empty plan')
    return plan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=json.loads((FOLDER/'interaction_config.json').read_text())
    plan=validate(generate(config))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(plan,stream,indent=2)
    print(json.dumps({'samples':len(plan['samples']),'duration_s':plan['samples'][-1]['time_s'],
                      'max_position_error_m':plan['max_position_error_m'],'status':plan['status']}))


if __name__=='__main__':main()
