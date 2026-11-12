"""Check pregrasp waypoints at each prop handle, not a solved grasp trajectory."""
from pathlib import Path
import json,numpy as np
from optimize_workspace import Arm,Rdown
ROOT=Path(__file__).resolve().parent;study=json.loads((ROOT/'tasks/reachability.json').read_text());dy=study['selected']['forward_translation_m'];index=json.loads((ROOT/'tasks/index.json').read_text());reports=[]
for task in index:
 side=task['preferred_hand'];arm=Arm(side,'R' if side=='left' else 'L',dy)
 p=np.array(task['grasp_reference_world_m']);p[2]=max(.88,p[2]+.075)
 lift=np.array(task['goal_pose_world'])[:3,3].copy();lift[2]=max(1.06,lift[2])
 ok,q=arm.solve(np.array([p,lift]),seeds=10)
 row={'task':task['id'],'hand':side,'pregrasp_reference_world_m':p.tolist(),'lift_reference_world_m':lift.tolist(),'pregrasp_reachable':bool(ok[0]),'lift_reachable':bool(ok[1]),'joint_names':arm.names,'pregrasp_q_rad':q[0].tolist() if ok[0] else None,'lift_q_rad':q[1].tolist() if ok[1] else None,'scope':'Palm-down arm pregrasp/lift waypoints, not finger contact IK or an executable collision-free trajectory. Object reorientation targets are task specifications.'}
 reports.append(row);print(task['id'],ok,flush=True)
(ROOT/'tasks/task_reach_checks.json').write_text(json.dumps(reports,indent=2))
assert all(r['pregrasp_reachable'] and r['lift_reachable'] for r in reports)
