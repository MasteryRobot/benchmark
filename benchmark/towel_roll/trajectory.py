"""Generate bimanual joint targets around a reduced-order towel roll.

The object path is kept separate from hand IK so policies can replace either
component. This file reports reachable hand targets, then exports actions.
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .config import FOLDER, SPEC
from .geometry import ArmIK, PALM_DOWN, initial_pose

def generate(duration_s=4., fps=30, turns=1.45):
    left, right = ArmIK('left'), ArmIK('right')
    init, _ = initial_pose(); actions=[]; ik=[]
    for k in range(int(duration_s*fps)+1):
        u=k/max(1,int(duration_s*fps)); winding=turns*u; radius=.018+.003*winding
        consumed=.30*u; cy=SPEC['towel']['near_y_m']+SPEC['towel']['length_m']-consumed
        cz=SPEC['towel']['table_height_m']+radius; theta=-np.pi/2+2*np.pi*winding
        rot=PALM_DOWN@Rotation.from_euler('x',theta+np.pi/2).as_matrix()
        targets={}; errs={}
        for arm,side,x in [(left,'left',-.135),(right,'right',.135)]:
            p=np.array([x,cy+radius*np.cos(theta),cz+radius*np.sin(theta)+.14])
            q,e=arm.solve(p,rot,trials=2); targets.update(q); errs[side]=e
        actions.append({'time_s':round(k/fps,4),'joint_targets_rad':targets,'bilateral_grasp':True,'commanded_turns':float(winding),'hand_target_left_m':[-.135,float(cy+radius*np.cos(theta)),float(cz+radius*np.sin(theta)+.14)],'hand_target_right_m':[.135,float(cy+radius*np.cos(theta)),float(cz+radius*np.sin(theta)+.14)]})
        ik.append({'time_s':round(k/fps,4),'errors':errs})
    path=FOLDER/'attempts/trajectory_001_actions.json'
    path.write_text(json.dumps({'task':SPEC['id'],'actions':actions,'ik_checks':ik,'status':'CPU candidate action sequence; validate in Isaac before training'},ensure_ascii=False,indent=2)+'\n')
    return path

if __name__=='__main__': print(generate())
