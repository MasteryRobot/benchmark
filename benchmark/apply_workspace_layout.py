"""Apply the selected whole-base translation and a symmetric, clear parked arm pose."""
from pathlib import Path
import json,numpy as np
from optimize_workspace import Arm,old
root=Path(__file__).resolve().parent;study=json.loads((root/'tasks/reachability.json').read_text());dy=study['selected']['forward_translation_m'];cfg=old['robots']['DualArm'];cfg['base_transform'][1][3]+=dy
arm=Arm('left','R',dy);ok,q=arm.solve(np.array([[-.35,.34,1.08]]),seeds=8);assert ok[0]
cfg['joint_positions'].update({n:float(v) for n,v in zip(arm.names,q[0])})
for i,sgn in enumerate([-1,1,-1,1,-1,1,-1],1):cfg['joint_positions'][f'Joint{i}_L']=sgn*cfg['joint_positions'][f'Joint{i}_R']
old['workspace_forward_translation_m']=dy;old['preset']='table_ready';old['ik']={'method':'bounded DLS IK, mirrored parked posture','hand_targets_m':[[-.35,.24,1.10],[.35,.24,1.10]]}
(root/'poses.json').write_text(json.dumps(old,indent=2));print('Applied forward translation',dy)
