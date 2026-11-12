"""Offline contract tests only. No fixture here is a manipulation success."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from trajectory_capture import camera_intrinsics, validate_success_episode
from export_lerobot import export, feature_schema
from task_runtime import ROOT, load_task, runtime_config


class CollectionTests(unittest.TestCase):
    def test_capture_matches_three_tasks_and_control_period(self):
        config = json.loads((ROOT / 'collection_config.json').read_text())
        self.assertEqual(config['resolution'], [460, 260])
        self.assertEqual(len(config['tasks']), 3)
        for task_id in config['tasks']:
            runtime = runtime_config(load_task(task_id))
            self.assertAlmostEqual(runtime['physics_dt_s'] * runtime['control_decimation'], 1/config['fps'])

    def test_square_pixels_and_updated_principal_point(self):
        K = camera_intrinsics(18., 36., [460, 260])
        np.testing.assert_allclose(K, [[230, 0, 230], [0, 230, 130], [0, 0, 1]])
        with self.assertRaises(ValueError):
            camera_intrinsics(18., 36., [460, 259])

    def test_features_keep_rgb_and_joint_order(self):
        names = ['a', 'b']
        features = feature_schema(names, ['Head'], [460, 260])
        self.assertEqual(features['action']['names'], names)
        self.assertEqual(features['observation.images.head']['shape'], (260, 460, 3))
        self.assertNotIn('object_pose_world', features)

    def test_empty_collection_cannot_create_training_dataset(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'export'
            with self.assertRaisesRegex(ValueError, 'No measured successful'):
                export(folder, output, 'v2.1')
            self.assertFalse(output.exists())

    def test_failed_and_synthetic_episodes_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            for source, status in [('isaacsim_measured', 'FAILED'), ('synthetic_test', 'SUCCESS')]:
                (path / 'episode.json').write_text(json.dumps({'source': source, 'status': status, 'success': True}))
                with self.assertRaisesRegex(ValueError, 'Only completed measured'):
                    validate_success_episode(path)

    def test_success_flag_without_provenance_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path / 'episode.json').write_text(json.dumps({
                'source': 'isaacsim_measured', 'status': 'SUCCESS', 'success': True}))
            with self.assertRaisesRegex(ValueError, 'Missing provenance'):
                validate_success_episode(path)


if __name__ == '__main__':
    unittest.main()
