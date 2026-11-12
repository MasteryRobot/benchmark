"""Run a fresh task reset/contact smoke test or replay joint targets in Isaac."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np
from task_runtime import ROOT, load_task, runtime_config, scene_fingerprint


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def read_actions(path, task):
    if path is None:
        return []
    document = json.loads(path.read_text())
    if document.get('task') != task['id']:
        raise ValueError('Replay task id must match the selected task')
    actions = document['actions']
    if not isinstance(actions, list) or not actions:
        raise ValueError('Replay requires at least one action')
    from robot_model import Robot
    model = Robot(ROOT / 'assets/dual_arm/robot.urdf')
    limits = {j.get('name'): (float(j.find('limit').get('lower')),
                              float(j.find('limit').get('upper'))) for j in model.revolute}
    for action in actions:
        targets = action.get('joint_targets_rad')
        if not isinstance(targets, dict) or not targets:
            raise ValueError('Each replay action requires joint_targets_rad')
        steps = action.get('control_steps', 1)
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError('control_steps must be a positive integer')
        for name, target in targets.items():
            if (name not in limits or isinstance(target, bool)
                    or not isinstance(target, (int, float)) or not math.isfinite(target)
                    or not limits[name][0] <= target <= limits[name][1]):
                raise ValueError(f'Invalid joint target: {name}')
    return actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task')
    parser.add_argument('--actions', type=Path, help='Replay JSON with task and actions fields')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--reset-repeats', type=int, default=2)
    args = parser.parse_args()
    task = load_task(args.task)
    if args.reset_repeats < 1:
        parser.error('--reset-repeats must be positive')
    actions = read_actions(args.actions, task)
    output = args.output or ROOT / 'tasks' / task['id'] / 'runtime_validation.json'
    report = {
        'task': task['id'], 'checked_at_utc': datetime.now(timezone.utc).isoformat(),
        'inputs_sha256': scene_fingerprint(task), 'runtime': runtime_config(task),
        'mode': 'joint_replay' if actions else 'reset_contact_smoke',
        'scope': 'Fresh reset, settling and measured contacts; autonomous manipulation is not implemented.',
        'runtime_status': 'NOT_RUN', 'task_success': False,
    }
    app = env = None
    code = 0
    try:
        if importlib.util.find_spec('isaacsim') is None:
            report['runtime_status'] = 'BLOCKED'
            report['reason'] = 'Isaac Sim is not installed in the selected Python environment.'
            code = 2
        else:
            from isaacsim import SimulationApp
            app = SimulationApp({'headless': not args.gui, 'limit_cpu_threads': 8})
            from isaac_task_env import IsaacTaskEnv
            env = IsaacTaskEnv(task['id'], app, render=args.gui)
            resets = []
            settled = []
            for _ in range(args.reset_repeats):
                observation = env.reset()
                settled.append(observation['object_pose_world'])
                initial = np.asarray(task['initial_pose_world'])
                linear = float(np.linalg.norm(observation['object_linear_velocity_m_s']))
                angular = float(np.linalg.norm(observation['object_angular_velocity_rad_s']))
                drift = float(np.max(np.abs(observation['joint_positions_rad'] - env.q0)))
                delta = observation['object_pose_world'][:3, 3] - initial[:3, 3]
                stable = bool(np.linalg.norm(delta[:2]) < .035 and abs(delta[2]) < .035
                              and linear < .01 and angular < .05 and drift < .02)
                resets.append({'stable': stable, 'physics_steps': env.config['settle_steps'],
                               'seconds': env.config['settle_steps'] * env.config['physics_dt_s'],
                               'linear_speed_m_s': linear, 'angular_speed_rad_s': angular,
                               'robot_joint_drift_rad': drift, 'object_displacement_m': float(np.linalg.norm(delta)),
                               'observation': observation})
            # Reset repeatability checks both translation and orientation.
            from scipy.spatial.transform import Rotation
            reset_position_error = max(float(np.linalg.norm(p[:3, 3] - settled[0][:3, 3])) for p in settled)
            reset_rotation_error = max(float(np.degrees(Rotation.from_matrix(
                settled[0][:3, :3].T @ p[:3, :3]).magnitude())) for p in settled)
            repeatable = reset_position_error < .005 and reset_rotation_error < 3
            report.update(resets=resets, reset_repeatable=repeatable,
                          reset_position_error_m=reset_position_error, reset_rotation_error_deg=reset_rotation_error,
                          sensor_count=len(env.cameras))
            result = None
            trace = []
            for action in actions or [{'joint_targets_rad': {}, 'control_steps': 30}]:
                for _ in range(action.get('control_steps', 1)):
                    observation, result = env.step(action['joint_targets_rad'])
                    trace.append({'observation': observation, 'evaluation': result})
                    if result['terminated'] or result['truncated']:
                        break
                if result['terminated'] or result['truncated']:
                    break
            report.update(trace=trace, evaluation=result, task_success=result['success'])
            smoke_ok = all(r['stable'] for r in resets) and repeatable
            if not actions:
                smoke_ok = smoke_ok and not result['success'] and result['reason'] == 'running'
            report['runtime_status'] = 'PASS' if smoke_ok else 'FAIL'
            code = 0 if smoke_ok else 1
    except Exception as error:
        report['runtime_status'] = 'ERROR'
        report['error_type'] = type(error).__name__
        # Full traceback goes to stderr, never into the portable scene package.
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        if env is not None:
            try:
                env.close()
            except Exception:
                report['cleanup_error'] = True
                report['runtime_status'] = 'ERROR'
                code = 1
        if app is not None:
            try:
                app.close()
            except Exception:
                report['cleanup_error'] = True
                report['runtime_status'] = 'ERROR'
                code = 1
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=serializable) + '\n')
        print(json.dumps({k: report[k] for k in ('task', 'runtime_status', 'task_success')}), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
