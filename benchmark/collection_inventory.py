"""Create an auditable all-task collection ledger without inventing successes."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def inventory(output):
    rows = []
    for suite, file in [('baseline', 'tasks/index.json'), ('sharpa', 'sharpa/index.json')]:
        for task in json.loads((ROOT / file).read_text()):
            kind = task.get('task_type', 'pose_lift_contact')
            gaps = ['functional_regrasp_policy', 'randomized_valid_initialization', 'five_camera_rgbd']
            if kind == 'scene_only': gaps += ['task_physics', 'task_outcome_evaluator']
            else: gaps += ['functional_outcome_validation']
            rows.append({'task': task['id'], 'title': task['title'], 'suite': suite,
                'existing_evaluator': kind, 'target_successes': 1,
                'verified_functional_successes': 0, 'status': 'PENDING_PHYSICS', 'gaps': gaps})
    rows.append({'task': 'bimanual_roll_towel', 'title': '双手卷毛巾', 'suite': 'deformable',
        'target_successes': 1, 'verified_functional_successes': 0, 'status': 'PENDING_PHYSICS',
        'gaps': ['stable_cloth_hand_interaction', 'rolling_policy', 'rolling_success_evaluator', 'five_camera_rgbd']})
    measured_reports=[]
    for path in output.rglob('report.json'):
        measured_reports.append((path,json.loads(path.read_text())))
    for row in rows:
        attempts = []
        for path,data in measured_reports:
            if data.get('task') != row['task']: continue
            attempts.append({'report': str(path.relative_to(output)), 'status': data.get('status'),
                'physics_started': data.get('physics_started',False),
                'recorded_physics_frames': data.get('recorded_physics_frames',0),
                'recorded_deformable_frames':data.get('recorded_frames',0),
                'functional_success': data.get('functional_success',False)})
        row['attempts'] = attempts
        if attempts:
            row['status'] = 'DIAGNOSTICS_ONLY' if any(a['recorded_physics_frames'] or a['recorded_deformable_frames'] for a in attempts) else 'BLOCKED_RUNTIME'
        if row['status']=='DIAGNOSTICS_ONLY':
            row['next_required_action']='implement_and_validate_task_policy_and_functional_success'
    report = {'updated_utc': datetime.now(timezone.utc).isoformat(),
        'registered_tasks': len(rows), 'requested_successes_per_task': 1,
        'verified_successes': 0, 'complete': False,
        'scope': 'All registered benchmark tasks. Physical diagnostics are not functional successes.',
        'requirements': {'rgbd_cameras':5,'resolution':[460,260],'fps':30,
            'physics_hz':240,'contact_points':True,'hand_object_relative_pose':True,
            'control':'robot_joint_targets_only'}, 'tasks':rows}
    output.mkdir(parents=True, exist_ok=True)
    (output/'coverage.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'tasks':len(rows),'diagnostic_tasks':sum(r['status']=='DIAGNOSTICS_ONLY' for r in rows),
                      'functional_successes':0,'output':str(output/'coverage.json')}))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    inventory(p.parse_args().output.resolve())
