"""Task contract and success evaluation; no simulator imports or inferred contacts.

Poses use column-vector homogeneous transforms in metres. Contact samples must
be per-body forces between the chosen hand and object, not total contact force.
"""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import math
import numpy as np

ROOT = Path(__file__).resolve().parent
RUNTIME_DEFAULTS = {
    'physics_dt_s': 1 / 240, 'control_decimation': 8, 'settle_steps': 1200,
    'episode_timeout_s': 20.0, 'success_hold_s': 0.25,
    'max_linear_speed_m_s': 0.05, 'max_angular_speed_rad_s': 0.25,
    'min_contact_force_n': 0.001, 'min_contact_bodies': 2,
    'drop_below_initial_m': 0.08,
}


def pose_matrix(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError('Pose must be a finite 4x4 matrix')
    if not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError('Invalid homogeneous pose bottom row')
    rotation = pose[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError('Pose rotation must belong to SO(3)')
    return pose.copy()


def runtime_config(task):
    result = dict(RUNTIME_DEFAULTS)
    override = task.get('runtime', {})
    if set(override) - set(result):
        raise ValueError('Unknown runtime settings')
    result.update(override)
    for key, value in result.items():
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'{key} must be finite and positive')
    for key in ('control_decimation', 'settle_steps', 'min_contact_bodies'):
        if int(result[key]) != result[key]:
            raise ValueError(f'{key} must be an integer')
        result[key] = int(result[key])
    return result


def validate_task(task):
    name = task['id']
    if not isinstance(name, str) or not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('Task id must be a single directory name')
    if task['preferred_hand'] not in ('left', 'right'):
        raise ValueError('Unknown preferred hand')
    if task.get('units') != 'm' or task['scene'] != 'scene.usda':
        raise ValueError('Expected metre units and local scene.usda')
    initial, goal = (pose_matrix(task[k]) for k in ('initial_pose_world', 'goal_pose_world'))
    for key in ('object_prim', 'goal_prim', 'fixture_prim'):
        if not task[key].startswith('/World/Task/'):
            raise ValueError(f'Invalid {key}')
    for key in ('position_tolerance_m', 'rotation_tolerance_deg', 'minimum_lift_m'):
        value = task['success'][key]
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid success threshold {key}')
    if not isinstance(task['success']['requires_grasp_contact'], bool):
        raise ValueError('requires_grasp_contact must be boolean')
    if goal[2, 3] - initial[2, 3] < task['success']['minimum_lift_m']:
        raise ValueError('Goal cannot meet the specified minimum lift')
    runtime_config(task)
    return task


def load_tasks(root=ROOT):
    root = Path(root)
    tasks = json.loads((root / 'tasks/index.json').read_text())
    seen = set()
    if not tasks:
        raise ValueError('Task index is empty')
    for task in tasks:
        validate_task(task)
        if task['id'] in seen:
            raise ValueError(f'Duplicate task: {task["id"]}')
        seen.add(task['id'])
        local = json.loads((root / 'tasks' / task['id'] / 'task.json').read_text())
        if local != task:
            raise ValueError(f'Index and task metadata differ: {task["id"]}')
    return tasks


def load_task(task_id, root=ROOT):
    for task in load_tasks(root):
        if task['id'] == task_id:
            return task
    raise ValueError(f'Unknown task: {task_id}')


def scene_fingerprint(task, root=ROOT):
    root = Path(root)
    names = ['tasks/workcell.usdc', f'tasks/{task["id"]}/scene.usda',
             f'tasks/{task["id"]}/task.json', 'poses.json', 'cameras.json',
             'assets/dual_arm/robot.urdf', 'task_runtime.py', 'isaac_task_env.py',
             'run_task_episode.py', 'robot_model.py']
    hashes = {}
    for name in names:
        digest = hashlib.sha256()
        with (root / name).open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        hashes[name] = digest.hexdigest()
    return hashes


def runtime_evidence_status(report, task, fingerprint):
    if report.get('task') != task['id'] or report.get('inputs_sha256') != fingerprint:
        return 'STALE'
    status = report.get('runtime_status', 'NOT_RUN')
    if status != 'PASS':
        return status if status in ('BLOCKED', 'FAIL', 'ERROR', 'TIMEOUT', 'NOT_RUN') else 'NOT_RUN'
    resets = report.get('resets', [])
    # A replay or single reset is not evidence of repeatable reset/contact smoke.
    complete = (report.get('mode') == 'reset_contact_smoke'
                and report.get('reset_repeatable') is True and len(resets) >= 2
                and report.get('sensor_count') == 5
                and all(r.get('stable') is True and r.get('physics_steps', 0) >= 1200
                        and r.get('seconds', 0) >= 5 for r in resets)
                and report.get('evaluation', {}).get('reason') == 'running'
                and report.get('task_success') is False)
    return 'PASS' if complete else 'INCOMPLETE'


class TaskEvaluator:
    """Continuous pose/lift/contact/speed checks, drop detection and timeout.

    Full SO(3) error preserves swab roll. Two distinct touching hand bodies
    are a contact proxy, not proof of force closure. Extra runtime thresholds
    are explicit design choices, independent of the original goal tolerances.
    """
    def __init__(self, task):
        self.task = deepcopy(validate_task(task))
        self.config = runtime_config(task)
        self.goal = pose_matrix(task['goal_pose_world'])
        self.reset()

    def reset(self, settled_pose=None):
        self.initial = pose_matrix(
            self.task['initial_pose_world'] if settled_pose is None else settled_pose)
        self.elapsed = self.hold = 0.0
        self.done = False

    def update(self, pose, linear_velocity, angular_velocity, contact_forces, dt):
        if self.done:
            raise RuntimeError('Episode is finished; call reset before stepping again')
        pose = pose_matrix(pose)
        linear, angular = np.asarray(linear_velocity, float), np.asarray(angular_velocity, float)
        contacts = np.asarray(contact_forces, float)
        if (linear.shape != (3,) or angular.shape != (3,)
                or not np.isfinite(linear).all() or not np.isfinite(angular).all()):
            raise ValueError('Velocities must be finite 3-vectors')
        if contacts.ndim != 2 or contacts.shape[1] != 3 or not np.isfinite(contacts).all():
            raise ValueError('Contacts must be a finite (hand bodies, 3) force matrix')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Step duration must be finite and positive')
        cfg, tolerances = self.config, self.task['success']
        position_error = float(np.linalg.norm(pose[:3, 3] - self.goal[:3, 3]))
        cosine = (np.trace(self.goal[:3, :3].T @ pose[:3, :3]) - 1) / 2
        rotation_error = math.degrees(math.acos(float(np.clip(cosine, -1, 1))))
        lift = float(pose[2, 3] - self.initial[2, 3])
        count = int(np.count_nonzero(np.linalg.norm(contacts, axis=1) >= cfg['min_contact_force_n']))
        grasped = count >= cfg['min_contact_bodies']
        criteria = {
            'position': position_error <= tolerances['position_tolerance_m'],
            'rotation': rotation_error <= tolerances['rotation_tolerance_deg'],
            'lift': lift >= tolerances['minimum_lift_m'],
            'grasp_contact': grasped or not tolerances['requires_grasp_contact'],
            'linear_speed': float(np.linalg.norm(linear)) <= cfg['max_linear_speed_m_s'],
            'angular_speed': float(np.linalg.norm(angular)) <= cfg['max_angular_speed_rad_s'],
        }
        self.elapsed += dt
        self.hold = self.hold + dt if all(criteria.values()) else 0.0
        success = self.hold + 1e-10 >= cfg['success_hold_s']
        dropped = lift < -cfg['drop_below_initial_m']
        timeout = self.elapsed + 1e-10 >= cfg['episode_timeout_s']
        terminated = bool(success or dropped)
        truncated = bool(timeout and not terminated)
        self.done = terminated or truncated
        return {
            'success': bool(success), 'terminated': terminated, 'truncated': truncated,
            'reason': 'success' if success else 'dropped' if dropped else 'timeout' if timeout else 'running',
            'position_error_m': position_error, 'rotation_error_deg': rotation_error,
            'lift_m': lift, 'contact_body_count': count, 'grasp_contact': grasped,
            'elapsed_s': self.elapsed, 'hold_s': self.hold, 'criteria': criteria,
            'reward': 1.0 if success else 0.0,
        }
