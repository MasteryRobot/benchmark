"""Synthetic format-only check, isolated in a temporary directory; never training data."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from export_lerobot import make_writer, add_frame, save_episode, finalize, verify_dataset
from trajectory_capture import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--format', choices=['v2.1', 'v3.0'], required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    config = {'fps': 30, 'resolution': [460, 260],
              'cameras': ['Head', 'LeftWrist', 'RightWrist', 'LeftSide', 'RightSide']}
    names = [f'test_joint_{j:02d}' for j in range(54)]
    with tempfile.TemporaryDirectory(prefix='synthetic_lerobot_format_test_') as temp:
        root = Path(temp) / 'dataset'
        writer = make_writer(root, args.format, names, config)
        for episode in range(2):
            for i in range(4):
                frame = {'observation.state': np.full(54, episode + i/10, dtype=np.float32),
                         'action': np.full(54, episode + (i+1)/10, dtype=np.float32),
                         'next.reward': np.zeros(1, dtype=np.float32),
                         'next.done': np.array([i == 3], dtype=bool),
                         'next.success': np.zeros(1, dtype=bool)}
                for j, camera in enumerate(config['cameras']):
                    image = np.zeros((260, 460, 3), dtype=np.uint8)
                    image[:, :, 0] = 30 + 20*j
                    image[:, :, 1] = 10 + 40*i
                    image[:, :, 2] = 30 + 60*episode
                    frame[f'observation.images.{camera.lower()}'] = image
                add_frame(writer, frame, f'SYNTHETIC FORMAT TEST {episode}; NOT A ROBOT TRAJECTORY', args.format)
            save_episode(writer, args.format)
        finalize(writer, args.format)
        result = verify_dataset(root, args.format, 2, 8)
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        reader = LeRobotDataset(repo_id='local/format_test', root=root, video_backend='pyav')
        for index in range(8):
            episode, i = divmod(index, 4)
            row = reader[index]
            np.testing.assert_allclose(row['observation.state'].numpy(), episode + i/10)
            np.testing.assert_allclose(row['action'].numpy(), episode + (i+1)/10)
            assert int(row['episode_index']) == episode
            assert int(row['frame_index']) == i
            assert int(row['task_index']) == episode
            assert bool(row['next.done'].item()) == (i == 3)
            for j, camera in enumerate(config['cameras']):
                mean = row[f'observation.images.{camera.lower()}'].numpy().mean(axis=(1, 2)) * 255
                np.testing.assert_allclose(mean, [30+20*j, 10+40*i, 30+60*episode], atol=8)
        result.update(scope='Synthetic format test only; no successful robot episodes',
                      resolution=[460, 260], cameras=5, alignment_and_values_checked=True,
                      temporary_dataset_deleted_after_test=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.report, result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
