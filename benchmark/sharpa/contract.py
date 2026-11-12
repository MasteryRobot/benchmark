"""Strict, simulator-independent contracts for multi-object scene recipes."""
import json
import math
from pathlib import Path
import re
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def vector(value, length, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (length,) or not np.isfinite(a).all():
        raise ValueError(f'{label} must be a finite {length}-vector')
    return a


def rotation(q):
    q = vector(q, 4, 'orientation_wxyz')
    if not np.isclose(np.linalg.norm(q), 1, atol=1e-7):
        raise ValueError('Quaternion must be unit length')
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()


def validate(spec):
    if spec.get('schema_version') != 1 or spec.get('units') != 'm' or spec.get('up_axis') != 'Z':
        raise ValueError('Expected schema 1, metres and Z-up')
    if not re.fullmatch(r'sharpa_[a-z0-9_]+', spec['id']):
        raise ValueError('Unsafe scene identifier')
    if spec['task_type'] not in ('place_all', 'photograph', 'return_ball', 'catch', 'scene_only'):
        raise ValueError('Unsupported task type')
    ids = set()
    for obj in spec['bodies']:
        if not re.fullmatch(r'[a-z][a-z0-9_]*', obj['id']) or obj['id'] in ids:
            raise ValueError('Unsafe or duplicate body identifier')
        ids.add(obj['id'])
        vector(obj['position'], 3, 'position'); rotation(obj['orientation_wxyz'])
        if not isinstance(obj['dynamic'], bool):
            raise ValueError('dynamic must be boolean')
        if obj['dynamic'] and (not isinstance(obj['mass_kg'], (float, int))
                               or not math.isfinite(obj['mass_kg']) or obj['mass_kg'] <= 0):
            raise ValueError('Dynamic body mass must be positive')
        if not obj['parts']:
            raise ValueError('Body needs geometry')
        for part in obj['parts']:
            if part['kind'] not in ('cube', 'cylinder', 'sphere'):
                raise ValueError('Unsupported geometry')
            n = {'cube': 3, 'cylinder': 2, 'sphere': 1}[part['kind']]
            if np.any(vector(part['size'], n, 'size') <= 0):
                raise ValueError('Geometry size must be positive')
            vector(part['position'], 3, 'part position')
            color = vector(part['color'], 3, 'color')
            if np.any(color < 0) or np.any(color > 1):
                raise ValueError('Color outside [0,1]')
            if part.get('axis', 'Z') not in ('X', 'Y', 'Z'):
                raise ValueError('Invalid cylinder axis')
            if not math.isfinite(part.get('yaw_deg', 0)):
                raise ValueError('Invalid part orientation')
        if 'launch_velocity_m_s' in obj:
            vector(obj['launch_velocity_m_s'], 3, 'launch velocity')
            if not obj['dynamic']:
                raise ValueError('Only dynamic bodies can be launched')
    if not any(b['dynamic'] for b in spec['bodies']):
        raise ValueError('Scene needs dynamic bodies')
    dynamic = {b['id'] for b in spec['bodies'] if b['dynamic']}
    if spec['task_type'] in ('place_all', 'photograph') and not spec['goals']:
        raise ValueError('Pose tasks require goals')
    for target in spec['goals']:
        if target['body'] not in dynamic:
            raise ValueError('Goal body must exist and be dynamic')
        vector(target['position'], 3, 'goal position'); rotation(target['orientation_wxyz'])
        if target['contact'] not in ('released', 'both_hands', 'either_hand'):
            raise ValueError('Unsupported contact rule')
        for key in ('position_tolerance_m', 'rotation_tolerance_deg'):
            if not math.isfinite(target[key]) or target[key] <= 0:
                raise ValueError('Goal tolerance must be positive')
    if spec['task_type'] == 'return_ball':
        if spec['ball'] not in dynamic or spec['racket'] not in dynamic:
            raise ValueError('Ball/racket body is missing')
    if spec['task_type'] == 'catch' and spec['caught_body'] not in dynamic:
        raise ValueError('Caught body is missing')
    cfg = spec['runtime']
    for key, value in cfg.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid runtime value: {key}')
    for key in ('control_decimation', 'settle_steps', 'min_contact_bodies'):
        if int(cfg[key]) != cfg[key]:
            raise ValueError(f'{key} must be integer')
    return spec


def load_scenarios(root=ROOT):
    root = Path(root)
    rows = json.loads((root / 'sharpa/index.json').read_text())
    if not rows or len({s['id'] for s in rows}) != len(rows):
        raise ValueError('Empty or duplicate scenario registry')
    for spec in rows:
        validate(spec)
        folder = root / 'sharpa/scenes' / spec['id']
        if json.loads((folder / 'task.json').read_text()) != spec:
            raise ValueError(f'Index differs from task.json: {spec["id"]}')
    return rows


def load_scenario(name, root=ROOT):
    for spec in load_scenarios(root):
        if spec['id'] == name:
            return spec
    raise ValueError(f'Unknown scenario: {name}')
