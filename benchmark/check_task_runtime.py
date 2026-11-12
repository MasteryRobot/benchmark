"""CPU asset audit. Text checks never substitute for USD composition or physics."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import re
import sys
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from robot_model import Robot, transform
from task_runtime import ROOT, load_tasks, pose_matrix, scene_fingerprint, runtime_evidence_status


def require(condition, message):
    if not condition:
        raise ValueError(message)


def prim_block(text, name):
    match = re.search(r'\bdef Xform "' + re.escape(name) + r'"(?=\s|\()', text)
    require(match is not None, f'Missing Xform {name}')
    start = text.index('{', match.end())
    depth, quoted, escaped = 0, False, False
    for index in range(start, len(text)):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
        elif char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return text[match.start():index + 1]
    raise ValueError(f'Unclosed Xform {name}')


def authored_pose(block):
    header = re.split(r'\bdef\s', block[block.index('{') + 1:], maxsplit=1)[0]
    position = re.search(r'double3 xformOp:translate = \(([^)]+)\)', header)
    orientation = re.search(r'quatd xformOp:orient = \(([^)]+)\)', header)
    require(position is not None and orientation is not None, 'Missing authored transform')
    p, q = np.fromstring(position[1], sep=','), np.fromstring(orientation[1], sep=',')
    require(p.shape == (3,) and q.shape == (4,), 'Invalid transform components')
    require(np.isclose(np.linalg.norm(q), 1, atol=1e-6), 'Quaternion is not normalized')
    pose = np.eye(4)
    pose[:3, 3] = p
    pose[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return pose_matrix(pose)


def check_ik(task, report, robot, cfg):
    require(report['task'] == task['id'], 'IK task mismatch')
    names = report['joint_names']
    chain = 'R' if task['preferred_hand'] == 'left' else 'L'
    require(names == [f'Joint{k}_{chain}' for k in range(1, 8)], 'Wrong arm joint mapping')
    limits = {j.get('name'): (float(j.find('limit').get('lower')),
                              float(j.find('limit').get('upper'))) for j in robot.revolute}
    desired = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    pregrasp = np.array(task['grasp_reference_world_m'], dtype=float)
    pregrasp[2] = max(.88, pregrasp[2] + .075)
    lift = np.array(task['goal_pose_world'], dtype=float)[:3, 3].copy()
    lift[2] = max(1.06, lift[2])
    rows = {}
    for label, expected in [('pregrasp', pregrasp), ('lift', lift)]:
        require(report[label + '_reachable'], f'Unreachable saved {label}')
        q = np.asarray(report[label + '_q_rad'], dtype=float)
        require(q.shape == (7,) and np.isfinite(q).all(), 'Invalid IK joint vector')
        for name, value in zip(names, q):
            lo, hi = limits[name]
            require(lo <= value <= hi, f'IK exceeds limit: {name}')
        state = dict(cfg['joint_positions'])
        state.update(zip(names, q))
        frames = robot.fk(state, np.asarray(cfg['base_transform']))
        tip = frames[task['preferred_hand'] + '_hand_base'] @ transform([.02, 0, .10])
        target = np.asarray(report[label + '_reference_world_m'])
        require(np.allclose(target, expected, atol=1e-8), 'Stale IK target')
        position_error = float(np.linalg.norm(tip[:3, 3] - target))
        angle_error = float(np.degrees(Rotation.from_matrix(desired @ tip[:3, :3].T).magnitude()))
        require(position_error < .008 and angle_error < 8, 'Saved IK differs from current model')
        rows[label] = {'position_error_m': position_error, 'rotation_error_deg': angle_error}
    return rows


def usd_composition(task, folder):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(folder / 'scene.usda'))
    require(stage is not None, 'Cannot compose USD stage')
    require(UsdGeom.GetStageUpAxis(stage) == 'Z', 'Stage must be Z-up')
    require(np.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1), 'Wrong scene units')
    require(not stage.GetPrimAtPath('/World/Props') and not stage.GetPrimAtPath('/World/Rack'),
            'Shared workcell contains original props')
    require(sum(p.IsA(UsdGeom.Camera) for p in stage.Traverse()) == 5, 'Expected five cameras')
    require(stage.GetPrimAtPath(task['object_prim']).HasAPI(UsdPhysics.RigidBodyAPI), 'Object is not rigid')
    goal = stage.GetPrimAtPath(task['goal_prim'])
    require(bool(goal), 'Missing goal marker')
    require(not any(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI)
                    for p in Usd.PrimRange(goal)), 'Goal marker affects physics')
    cache = UsdGeom.XformCache()
    for path, key in [(task['object_prim'], 'initial_pose_world'), (task['goal_prim'], 'goal_pose_world')]:
        actual = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
        require(np.allclose(actual, task[key], atol=1e-6), 'Composed pose differs from metadata')
    return 'PASS'


def audit(root=ROOT):
    root = Path(root)
    tasks = load_tasks(root)
    require(len(tasks) == 10, 'Expected ten benchmark tasks')
    robot = Robot(root / 'assets/dual_arm/robot.urdf')
    require(len(robot.revolute) == 54, 'Expected 54 revolute joints')
    cfg = json.loads((root / 'poses.json').read_text())['robots']['DualArm']
    for mesh in robot.xml.findall('.//mesh'):
        asset = root / 'assets/dual_arm' / mesh.get('filename')
        require(asset.is_file() and asset.stat().st_size > 84, 'Missing robot mesh')
    for filename in ('dealer_table.usdc', 'tasks/workcell.usdc'):
        with (root / filename).open('rb') as stream:
            require(stream.read(8) == b'PXR-USDC', 'Invalid binary USD signature')
    reach = json.loads((root / 'tasks/task_reach_checks.json').read_text())
    require(len(reach) == len(tasks), 'Missing or duplicate IK records')
    by_id = {row['task']: row for row in reach}
    require(set(by_id) == {t['id'] for t in tasks}, 'IK task ids do not match manifest')
    cameras = json.loads((root / 'cameras.json').read_text())
    require(len(cameras) == 5, 'Expected five camera definitions')
    has_usd = importlib.util.find_spec('pxr') is not None
    rows = []
    for task in tasks:
        row = {'task': task['id'], 'offline_status': 'FAIL', 'usd_composition': 'NOT_RUN',
               'physics_status': 'NOT_RUN', 'autonomous_manipulation': 'NOT_IMPLEMENTED'}
        try:
            folder = root / 'tasks' / task['id']
            text = (folder / 'scene.usda').read_text()
            require(text.startswith('#usda 1.0'), 'Invalid scene entry header')
            require('@../workcell.usdc@' in text, 'Missing shared workcell reference')
            for reference in re.findall(r'@([^@]+)@', text):
                path = (folder / reference).resolve()
                require(not Path(reference).is_absolute() and root.resolve() in path.parents,
                        'Non-portable asset reference')
                require(path.is_file(), 'Unresolved local asset reference')
            obj, goal = prim_block(text, 'Object'), prim_block(text, 'Goal')
            require('PhysicsRigidBodyAPI' in obj and 'PhysicsCollisionAPI' in obj, 'Missing object physics')
            require('PhysicsRigidBodyAPI' not in goal and 'PhysicsCollisionAPI' not in goal, 'Physical goal marker')
            for block, key in [(obj, 'initial_pose_world'), (goal, 'goal_pose_world')]:
                require(np.allclose(authored_pose(block), task[key], atol=1e-6), 'Authored pose mismatch')
            depths = {}
            for name, camera in cameras.items():
                with Image.open(folder / 'images' / (name.lower() + '.png')) as image:
                    width, height = image.size
                    image.verify()
                require(width * camera['resolution'][1] == height * camera['resolution'][0], 'Image aspect ratio mismatch')
                with np.load(folder / 'images' / (name.lower() + '_depth.npz'), allow_pickle=False) as data:
                    require('distance_m' in data, 'Missing geometric distance channel')
                    depth = data['distance_m']
                    require(depth.shape == (height, width), 'RGB/depth shape mismatch')
                    require(not np.isnan(depth).any() and not (depth < 0).any(), 'Invalid depth values')
                    finite = np.isfinite(depth) & (depth > 0)
                    require(finite.any(), 'Empty depth image')
                    depths[name] = {'width': width, 'height': height, 'finite_positive_fraction': float(finite.mean())}
            for name in ('overview', 'detail'):
                with Image.open(folder / 'images' / (name + '.png')) as image:
                    image.verify()
            row['saved_ik_forward_check'] = check_ik(task, by_id[task['id']], robot, cfg)
            row['camera_assets'] = depths
            row['inputs_sha256'] = scene_fingerprint(task, root)
            legacy = json.loads((folder / 'validation.json').read_text())
            require(legacy.get('task') == task['id'], 'Legacy report task mismatch')
            row['legacy_physics_report'] = 'PRESENT_WITHOUT_CURRENT_INPUT_FINGERPRINT'
            live = folder / 'runtime_validation.json'
            if live.exists():
                data = json.loads(live.read_text())
                row['physics_status'] = runtime_evidence_status(data, task, row['inputs_sha256'])
            if has_usd:
                row['usd_composition'] = usd_composition(task, folder)
            row['offline_status'] = 'PASS'
        except Exception as error:
            row['error'] = str(error)
        rows.append(row)
        print(task['id'], row['offline_status'], row.get('error', ''), flush=True)
    return {
        'checked_at_utc': datetime.now(timezone.utc).isoformat(),
        'offline_status': 'PASS' if all(r['offline_status'] == 'PASS' for r in rows) else 'FAIL',
        'tasks': len(rows), 'rows': rows,
        'scope': 'Metadata, local USDA poses/references, images/depth and saved IK forward checks. Live physics and manipulation require separate execution.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'tasks/runtime_checks.json')
    args = parser.parse_args()
    report = audit()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ('offline_status', 'tasks')}))
    return int(report['offline_status'] != 'PASS')


if __name__ == '__main__':
    sys.exit(main())
