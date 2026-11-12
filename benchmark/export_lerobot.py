"""Export measured successes with official writers; no network upload.

Use lerobot==0.3.3 for v2.1, lerobot==0.4.3 for v3.0 in separate environments.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from trajectory_capture import validate_success_episode, write_json

PINNED = {'v2.1': '0.3.3', 'v3.0': '0.4.3'}


def feature_schema(names, cameras, resolution, media='video'):
    width, height = resolution
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (len(names),), 'names': names},
        'action': {'dtype': 'float32', 'shape': (len(names),), 'names': names},
        'next.reward': {'dtype': 'float32', 'shape': (1,), 'names': None},
        'next.done': {'dtype': 'bool', 'shape': (1,), 'names': None},
        'next.success': {'dtype': 'bool', 'shape': (1,), 'names': None},
    }
    for camera in cameras:
        features[f'observation.images.{camera.lower()}'] = {
            'dtype': media, 'shape': (height, width, 3), 'names': ['height', 'width', 'channels']}
    return features


def make_writer(output, version, names, config, media='video'):
    # The host contains an older ONNX protobuf schema imported by torchvision.
    # Use protobuf's supported Python implementation without changing system packages.
    os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
    os.environ.setdefault('HF_HUB_OFFLINE', '1')
    os.environ.setdefault('HF_DATASETS_OFFLINE', '1')
    os.environ.setdefault('HF_HOME', str(Path(__file__).resolve().parent / '.cache/huggingface'))
    os.environ.setdefault('HF_DATASETS_CACHE', str(Path(os.environ['HF_HOME']) / 'datasets'))
    from importlib.metadata import version as installed
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    if CODEBASE_VERSION != version or installed('lerobot') != PINNED[version]:
        raise RuntimeError(f'Use lerobot=={PINNED[version]} for {version}; installed format is {CODEBASE_VERSION}')
    kwargs = dict(repo_id=f'local/dex_representative_{version.replace(".", "_")}',
                  fps=config['fps'], root=Path(output), robot_type='dual_arm_dexterous_54dof',
                  features=feature_schema(names, config['cameras'], config['resolution'], media),
                  use_videos=media == 'video', image_writer_threads=0,
                  video_backend='pyav', batch_encoding_size=1)
    if version == 'v3.0':
        kwargs['vcodec'] = 'h264'
    return LeRobotDataset.create(**kwargs)


def add_frame(dataset, frame, task, version):
    if version == 'v2.1':
        dataset.add_frame(frame, task=task)
    else:
        dataset.add_frame({**frame, 'task': task})


def save_episode(dataset, version):
    if version == 'v2.1':
        dataset.save_episode()
    else:
        dataset.save_episode(parallel_encoding=False)


def finalize(dataset, version):
    if version == 'v3.0':
        dataset.finalize()
    dataset.stop_image_writer()


def verify_dataset(output, version, expected_episodes, expected_frames):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    info = json.loads((Path(output) / 'meta/info.json').read_text())
    if info['codebase_version'] != version:
        raise ValueError('Exported metadata format is incorrect')
    dataset = LeRobotDataset(repo_id=f'local/dex_representative_{version.replace(".", "_")}',
                             root=Path(output), video_backend='pyav')
    if dataset.num_episodes != expected_episodes or len(dataset) != expected_frames:
        raise ValueError('Official loader returned unexpected episode or frame count')
    previous_episode = -1
    for index in range(len(dataset)):
        row = dataset[index]
        episode = int(row['episode_index'])
        if episode != previous_episode:
            if int(row['frame_index']) != 0:
                raise ValueError('Episode does not begin at frame zero')
            previous_episode = episode
        for key in ['observation.state', 'action']:
            if not np.isfinite(row[key].numpy()).all():
                raise ValueError(f'Non-finite readback values: {key}')
        for key, feature in info['features'].items():
            if feature['dtype'] in ['video', 'image']:
                height, width, _ = feature['shape']
                if tuple(row[key].shape) != (3, height, width):
                    raise ValueError(f'Invalid decoded image shape: {key}')
    return {'status': 'PASS', 'format': version, 'episodes': expected_episodes,
            'frames': expected_frames, 'official_loader': PINNED[version], 'all_frames_decoded': True}


def export(raw_root, output, version, media='video'):
    raw_root, output = Path(raw_root), Path(output)
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite dataset: {output}')
    accepted = []
    for path in sorted(raw_root.rglob('episode.json')):
        metadata = json.loads(path.read_text())
        if metadata.get('status') == 'SUCCESS':
            meta, frames = validate_success_episode(path.parent)
            accepted.append((path.parent, meta, frames))
    if not accepted:
        raise ValueError('No measured successful episodes. No training dataset was created.')
    _, first, _ = accepted[0]
    for _, meta, _ in accepted[1:]:
        if (meta['joint_names'] != first['joint_names'] or
            any(meta['config'][k] != first['config'][k] for k in ['fps', 'resolution', 'cameras'])):
            raise ValueError('Cannot mix incompatible recording configurations')
    writer = make_writer(output, version, first['joint_names'], first['config'], media)
    manifest, total_frames = [], 0
    try:
        for episode_index, (path, meta, frames) in enumerate(accepted):
            for folder in frames:
                with np.load(folder / 'transition.npz') as arrays:
                    frame = {key: arrays[key].copy() for key in
                             ['observation.state', 'action', 'next.reward', 'next.done', 'next.success']}
                for camera in meta['config']['cameras']:
                    with Image.open(folder / f'{camera}.png') as image:
                        frame[f'observation.images.{camera.lower()}'] = np.asarray(image).copy()
                add_frame(writer, frame, meta['task']['title'], version)
            save_episode(writer, version)
            sidecar = output / 'extras' / f'episode_{episode_index:06d}'
            shutil.copytree(path, sidecar)
            manifest.append({'episode_index': episode_index, 'task_id': meta['task']['id'],
                             'raw_sidecar': str(sidecar.relative_to(output)), 'frames': len(frames),
                             'success': True, 'policy': meta['policy']})
            total_frames += len(frames)
        finalize(writer, version)
        result = verify_dataset(output, version, len(accepted), total_frames)
        write_json(output / 'meta' / 'collection_manifest.json', {
            'format': version, 'episodes': manifest, 'readback': result,
            'state': '54 measured joint positions in radians; order in feature names',
            'action': '54 absolute commanded joint position targets in radians at observation time',
            'depth': 'Lossless float32 metres and validity masks in extras/*/frames/*/transition.npz',
            'privileged_state': 'Object poses and contacts are diagnostics in extras, not policy input',
        })
        return result
    except Exception as error:
        write_json(output / 'EXPORT_INCOMPLETE.json', {'status': 'ERROR', 'error': str(error)})
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--format', choices=list(PINNED), required=True)
    parser.add_argument('--media', choices=['video', 'image'], default='video')
    args = parser.parse_args()
    print(json.dumps(export(args.raw_root, args.output, args.format, args.media), indent=2))


if __name__ == '__main__':
    main()
