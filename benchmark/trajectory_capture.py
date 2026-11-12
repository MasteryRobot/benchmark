"""Synchronous camera capture and measured transition recording, without simulator mocks."""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from task_runtime import ROOT, TaskEvaluator, scene_fingerprint


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    default=jsonable, allow_nan=False) + '\n')
    temporary.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def camera_intrinsics(focal_length, horizontal_aperture, resolution):
    width, height = resolution
    if any(type(x) is not int or x <= 0 or x % 2 for x in resolution):
        raise ValueError('Video resolution must contain positive even integers')
    fx = width * float(focal_length) / float(horizontal_aperture)
    return np.array([[fx, 0., width / 2], [0., fx, height / 2], [0., 0., 1.]])


class CameraRig:
    """Render all five cameras at a frozen physics time; wrist extrinsics are dynamic."""
    def __init__(self, env, config):
        import omni.replicator.core as rep
        from pxr import Usd, UsdGeom
        self.env, self.rep = env, rep
        self.names = list(config['cameras'])
        self.resolution = tuple(config['resolution'])
        self.products, self.rgb, self.depth, self.calibration = {}, {}, {}, {}
        self.save_depth = config['save_depth']
        width, height = self.resolution
        with Usd.EditContext(env.stage, env.stage.GetSessionLayer()):
            # Goal geometry is an inspection overlay, not a physical RGB observation.
            UsdGeom.Imageable(env.stage.GetPrimAtPath(env.task['goal_prim'])).MakeInvisible()
            for name in self.names:
                spec = env.cameras[name]
                camera = UsdGeom.Camera(env.stage.GetPrimAtPath(spec['prim_path']))
                focal = float(camera.GetFocalLengthAttr().Get())
                aperture = float(camera.GetHorizontalApertureAttr().Get())
                camera.GetVerticalApertureAttr().Set(aperture * height / width)
                self.calibration[name] = {
                    'resolution': [width, height],
                    'K': camera_intrinsics(focal, aperture, self.resolution).tolist(),
                    'prim_path': spec['prim_path'], 'parent_body': spec['parent_body'],
                    'depth': 'Optical-axis z in metres, float32; invalid depths stored as NaN with mask',
                    'convention': 'OpenCV: +X right, +Y down, +Z forward',
                    'fov_policy': 'Preserve horizontal aperture; adapt vertical aperture for square pixels',
                }
                product = rep.create.render_product(spec['prim_path'], self.resolution)
                self.products[name] = product
                self.rgb[name] = rep.AnnotatorRegistry.get_annotator('rgb')
                self.rgb[name].attach([product])
                if self.save_depth:
                    self.depth[name] = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
                    self.depth[name].attach([product])

    def capture(self):
        from pxr import Usd, UsdGeom
        before = float(self.env.world.current_time)
        self.rep.orchestrator.step(rt_subframes=1, pause_timeline=False, delta_time=0.0)
        after = float(self.env.world.current_time)
        if abs(after - before) > 1e-9:
            raise RuntimeError('Camera capture advanced physics; refusing misaligned training data')
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        rgb, depth, poses = {}, {}, {}
        width, height = self.resolution
        for name in self.names:
            data = np.asarray(self.rgb[name].get_data())
            if data.shape not in [(height, width, 3), (height, width, 4)] or data.dtype != np.uint8:
                raise RuntimeError(f'Invalid RGB frame: {name} {data.shape} {data.dtype}')
            rgb[name] = data[:, :, :3].copy()
            if not np.any(rgb[name]):
                raise RuntimeError(f'Empty/black camera frame: {name}')
            if self.save_depth:
                z = np.asarray(self.depth[name].get_data(), dtype=np.float32).reshape(height, width)
                valid = np.isfinite(z) & (z > 0)
                if not valid.any():
                    raise RuntimeError(f'No valid measured depth: {name}')
                depth[name] = np.where(valid, z, np.nan)
            prim = self.env.stage.GetPrimAtPath(self.calibration[name]['prim_path'])
            poses[name] = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float).T @ np.diag([1, -1, -1, 1])
        return {'rgb': rgb, 'depth': depth, 'camera_poses': poses, 'sim_time_s': before}

    def close(self):
        for name, product in self.products.items():
            self.rgb[name].detach([product])
            if name in self.depth:
                self.depth[name].detach([product])
            product.destroy()


class RawEpisode:
    """A row means observation at t, commanded action at t, outcome after action."""
    def __init__(self, path, env, config, rig, initial, policy_info):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'frames').mkdir()
        self.env, self.config, self.rig = env, config, rig
        self.count = 0
        self.physics_count = 0
        self.last_result = None
        self.physics_log = (self.path / 'physics.jsonl').open('w')
        self.metadata = {
            'schema_version': 1, 'source': 'isaacsim_measured',
            'started_at_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'RECORDING', 'success': False, 'task': deepcopy(env.task),
            'config': deepcopy(config), 'joint_names': list(env.names),
            'settled_object_pose_world': initial['object_pose_world'],
            'contact_body_paths': initial['contact_body_paths'],
            'calibration': rig.calibration, 'policy': policy_info,
            'inputs_sha256': scene_fingerprint(env.task),
            'collector_sha256': {name: sha256(ROOT / name) for name in
                                 ['trajectory_capture.py', 'collect_representative.py']},
            'alignment': 'observation[t], image[t], action[t], next_observation[t+1], next.reward/done',
        }
        write_json(self.path / 'episode.json', self.metadata)

    def physics_step(self, observation, result, dt):
        row = {key: observation[key] for key in [
            'object_pose_world', 'object_linear_velocity_m_s',
            'object_angular_velocity_rad_s', 'contact_forces_n']}
        row.update(dt_s=dt, evaluation=result)
        self.physics_log.write(json.dumps(row, default=jsonable, allow_nan=False) + '\n')
        self.physics_count += 1

    def append(self, before, action, captured, after, result, dt):
        folder = self.path / 'frames' / f'{self.count:06d}'
        folder.mkdir()
        if list(before['joint_names']) != self.metadata['joint_names']:
            raise ValueError('Joint ordering changed within episode')
        arrays = {
            'observation.state': np.asarray(before['joint_positions_rad'], dtype=np.float32),
            'observation.velocity': np.asarray(before['joint_velocities_rad_s'], dtype=np.float32),
            'action': np.asarray(action, dtype=np.float32),
            'next.observation.state': np.asarray(after['joint_positions_rad'], dtype=np.float32),
            'timestamp': np.float64(self.count / self.config['fps']),
            'sim_timestamp': np.float64(captured['sim_time_s']),
            'transition_dt': np.float64(dt),
            'next.reward': np.array([result['reward']], dtype=np.float32),
            'next.done': np.array([result['terminated'] or result['truncated']], dtype=bool),
            'next.success': np.array([result['success']], dtype=bool),
            'next.truncated': np.array([result['truncated']], dtype=bool),
            'diagnostic.object_pose_world': np.asarray(before['object_pose_world']),
            'diagnostic.next_object_pose_world': np.asarray(after['object_pose_world']),
            'diagnostic.contact_forces_n': np.asarray(before['contact_forces_n']),
            'diagnostic.next_contact_forces_n': np.asarray(after['contact_forces_n']),
        }
        for key in ['observation.state', 'observation.velocity', 'action', 'next.observation.state']:
            if arrays[key].shape != (len(self.env.names),) or not np.isfinite(arrays[key]).all():
                raise ValueError(f'Invalid measured feature: {key}')
        for name in self.config['cameras']:
            Image.fromarray(captured['rgb'][name]).save(folder / f'{name}.png')
            arrays[f'camera_pose.{name}'] = captured['camera_poses'][name]
            if self.config['save_depth']:
                arrays[f'depth.{name}'] = captured['depth'][name]
                arrays[f'depth_valid.{name}'] = np.isfinite(captured['depth'][name])
        np.savez_compressed(folder / 'transition.npz', **arrays)
        write_json(folder / 'evaluation.json', result)
        self.count += 1
        self.last_result = result

    def finish(self, error=None):
        self.physics_log.close()
        success = bool(error is None and self.count and self.last_result['success'])
        self.metadata.update(status='SUCCESS' if success else 'ERROR' if error else 'FAILED',
                             success=success, frames=self.count, physics_steps=self.physics_count,
                             final_evaluation=self.last_result, error=error,
                             finished_at_utc=datetime.now(timezone.utc).isoformat())
        self.metadata['files_sha256'] = {str(p.relative_to(self.path)): sha256(p)
            for p in sorted(self.path.rglob('*')) if p.is_file() and p.name != 'episode.json'}
        write_json(self.path / 'episode.json', self.metadata)
        return self.metadata


def validate_success_episode(path, verify_hashes=True):
    """Re-evaluate every measured physics tick before accepting a training episode."""
    path = Path(path)
    meta = json.loads((path / 'episode.json').read_text())
    if meta.get('source') != 'isaacsim_measured' or meta.get('status') != 'SUCCESS' or meta.get('success') is not True:
        raise ValueError('Only completed measured successful episodes can be exported')
    if not meta.get('files_sha256') or not meta.get('inputs_sha256'):
        raise ValueError('Missing provenance hashes')
    required_files = {'physics.jsonl'}
    if type(meta.get('frames')) is not int or meta['frames'] < 1 or len(meta.get('joint_names', [])) != 54:
        raise ValueError('Invalid episode length or robot joint schema')
    if len(set(meta['joint_names'])) != 54:
        raise ValueError('Duplicate joint names')
    if meta['config']['fps'] != 30 or meta['config']['resolution'] != [460, 260]:
        raise ValueError('Unexpected capture configuration')
    for index in range(meta.get('frames', 0)):
        required_files.update({f'frames/{index:06d}/transition.npz', f'frames/{index:06d}/evaluation.json'})
        required_files.update(f'frames/{index:06d}/{name}.png' for name in meta['config']['cameras'])
    if not required_files.issubset(meta['files_sha256']):
        raise ValueError('Provenance does not cover every training frame')
    if verify_hashes:
        for name, expected in meta['files_sha256'].items():
            file = (path / name).resolve()
            if not file.is_relative_to(path.resolve()) or sha256(file) != expected:
                raise ValueError(f'Raw file integrity failed: {name}')
    evaluator = TaskEvaluator(meta['task'])
    evaluator.reset(meta['settled_object_pose_world'])
    result = None
    count = 0
    for line in (path / 'physics.jsonl').read_text().splitlines():
        row = json.loads(line)
        result = evaluator.update(row['object_pose_world'], row['object_linear_velocity_m_s'],
                                  row['object_angular_velocity_rad_s'], row['contact_forces_n'], row['dt_s'])
        if result['reason'] != row['evaluation']['reason']:
            raise ValueError('Stored and replayed physics evaluation disagree')
        count += 1
    if not result or not result['success'] or count != meta['physics_steps']:
        raise ValueError('Physics trace does not establish continuous success')
    frames = sorted((path / 'frames').iterdir())
    if len(frames) != meta['frames'] or not frames:
        raise ValueError('Missing transition frames')
    expected_time = None
    previous_next_state = None
    total_dt = 0.
    width, height = meta['config']['resolution']
    for index, frame in enumerate(frames):
        if frame.name != f'{index:06d}':
            raise ValueError('Non-contiguous raw frame indices')
        with np.load(frame / 'transition.npz') as data:
            for key in ['observation.state', 'observation.velocity', 'action', 'next.observation.state']:
                if data[key].shape != (len(meta['joint_names']),) or not np.isfinite(data[key]).all():
                    raise ValueError(f'Invalid numeric feature: {key}')
            if previous_next_state is not None and not np.allclose(previous_next_state, data['observation.state'], atol=1e-6):
                raise ValueError('Consecutive transition states are not aligned')
            previous_next_state = data['next.observation.state'].copy()
            if abs(float(data['timestamp']) - index / meta['config']['fps']) > 1e-8:
                raise ValueError('Invalid frame timestamp')
            sim_time = float(data['sim_timestamp'])
            if expected_time is not None and abs(sim_time - expected_time) > 1e-6:
                raise ValueError('Camera/physics timeline discontinuity')
            dt = float(data['transition_dt'])
            if dt <= 0 or (index < len(frames)-1 and abs(dt - 1/meta['config']['fps']) > 1e-6):
                raise ValueError('Unexpected control period')
            expected_time = sim_time + dt
            total_dt += dt
            if bool(data['next.done'][0]) != (index == len(frames)-1):
                raise ValueError('Episode boundary is misaligned')
            if index == len(frames)-1 and not bool(data['next.success'][0]):
                raise ValueError('Final training transition is not successful')
            for name in meta['config']['cameras']:
                pose = data[f'camera_pose.{name}']
                if pose.shape != (4, 4) or not np.isfinite(pose).all():
                    raise ValueError('Missing or invalid per-frame camera extrinsics')
                if meta['config']['save_depth']:
                    depth, mask = data[f'depth.{name}'], data[f'depth_valid.{name}']
                    if depth.shape != (height, width) or mask.shape != depth.shape or mask.dtype != bool:
                        raise ValueError('Invalid depth shape or mask')
                    if not mask.any() or not np.array_equal(mask, np.isfinite(depth)) or (depth[mask] <= 0).any():
                        raise ValueError('Depth validity mask does not match measured depth')
        for name in meta['config']['cameras']:
            with Image.open(frame / f'{name}.png') as image:
                if image.size != (width, height) or image.mode != 'RGB':
                    raise ValueError(f'Invalid image: {frame.name}/{name}')
    if abs(total_dt - result['elapsed_s']) > 1e-6:
        raise ValueError('Rendered transitions do not span the measured physics trace')
    return meta, frames
