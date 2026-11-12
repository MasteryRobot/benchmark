"""Regression cases for task success, termination and stale evidence handling."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from task_runtime import ROOT, TaskEvaluator, load_tasks, pose_matrix, validate_task, runtime_evidence_status
from run_task_episode import read_actions


class EvaluatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tasks = load_tasks()

    def update(self, evaluator, pose=None, contacts=None, linear=None, angular=None, dt=1/240):
        return evaluator.update(
            evaluator.goal if pose is None else pose,
            [0, 0, 0] if linear is None else linear,
            [0, 0, 0] if angular is None else angular,
            [[.01, 0, 0], [-.01, 0, 0]] if contacts is None else contacts, dt)

    def test_all_ten_goals_require_continuous_hold(self):
        self.assertEqual(len(self.tasks), 10)
        for task in self.tasks:
            with self.subTest(task=task['id']):
                evaluator = TaskEvaluator(task)
                for _ in range(59):
                    self.assertFalse(self.update(evaluator)['success'])
                result = self.update(evaluator)
                self.assertTrue(result['success'])
                self.assertTrue(result['terminated'])
                self.assertFalse(result['truncated'])
                with self.assertRaises(RuntimeError):
                    self.update(evaluator)

    def test_initial_pose_is_never_success(self):
        for task in self.tasks:
            evaluator = TaskEvaluator(task)
            for _ in range(65):
                result = self.update(evaluator, pose=task['initial_pose_world'])
            self.assertFalse(result['success'])
            self.assertFalse(result['criteria']['lift'])

    def test_pose_without_hand_contacts_does_not_succeed(self):
        for contacts in ([], [[1, 0, 0]], [[.0001, 0, 0], [0, .0001, 0]]):
            evaluator = TaskEvaluator(self.tasks[0])
            for _ in range(70):
                result = self.update(evaluator, contacts=np.asarray(contacts).reshape(-1, 3))
            self.assertFalse(result['success'])
            self.assertEqual(result['hold_s'], 0)

    def test_contact_loss_restarts_hold(self):
        evaluator = TaskEvaluator(self.tasks[0])
        for _ in range(59):
            self.update(evaluator)
        self.assertEqual(self.update(evaluator, contacts=np.empty((0, 3)))['hold_s'], 0)
        for _ in range(59):
            self.assertFalse(self.update(evaluator)['success'])
        self.assertTrue(self.update(evaluator)['success'])

    def test_orientation_detects_phone_flip_and_swab_roll(self):
        for task in self.tasks:
            if task['action'] == 'lift':
                continue
            evaluator = TaskEvaluator(task)
            pose = evaluator.goal.copy()
            pose[:3, :3] = np.asarray(task['initial_pose_world'])[:3, :3]
            result = self.update(evaluator, pose=pose)
            self.assertFalse(result['criteria']['rotation'], task['id'])
            self.assertGreater(result['rotation_error_deg'], 12)

    def test_rotation_error_near_pi_is_finite(self):
        evaluator = TaskEvaluator(self.tasks[0])
        pose = evaluator.goal.copy()
        pose[:3, :3] = pose[:3, :3] @ Rotation.from_euler('x', 180, degrees=True).as_matrix()
        result = self.update(evaluator, pose=pose)
        self.assertAlmostEqual(result['rotation_error_deg'], 180)

    def test_speed_and_position_reject_success(self):
        for kwargs, criterion in [({'linear': [.1, 0, 0]}, 'linear_speed'),
                                  ({'angular': [0, 1, 0]}, 'angular_speed')]:
            evaluator = TaskEvaluator(self.tasks[0])
            self.assertFalse(self.update(evaluator, **kwargs)['criteria'][criterion])
        evaluator = TaskEvaluator(self.tasks[0])
        pose = evaluator.goal.copy()
        pose[0, 3] += .016
        self.assertFalse(self.update(evaluator, pose=pose)['criteria']['position'])

    def test_drop_terminates_and_timeout_truncates(self):
        evaluator = TaskEvaluator(self.tasks[0])
        pose = evaluator.initial.copy()
        pose[2, 3] -= .081
        result = self.update(evaluator, pose=pose)
        self.assertEqual(result['reason'], 'dropped')
        self.assertTrue(result['terminated'])
        evaluator.reset()
        for _ in range(600):
            result = self.update(evaluator, pose=evaluator.initial, dt=1/30)
        self.assertEqual(result['reason'], 'timeout')
        self.assertTrue(result['truncated'])
        self.assertFalse(result['terminated'])

    def test_reset_uses_settled_height_and_clears_state(self):
        evaluator = TaskEvaluator(self.tasks[0])
        self.update(evaluator)
        pose = evaluator.initial.copy()
        pose[2, 3] += .15
        evaluator.reset(pose)
        result = self.update(evaluator)
        self.assertFalse(result['criteria']['lift'])
        self.assertEqual(result['hold_s'], 0)
        self.assertAlmostEqual(result['elapsed_s'], 1/240)

    def test_invalid_state_is_rejected(self):
        evaluator = TaskEvaluator(self.tasks[0])
        invalid = evaluator.goal.copy()
        invalid[0, 0] = float('nan')
        with self.assertRaises(ValueError):
            self.update(evaluator, pose=invalid)
        with self.assertRaises(ValueError):
            self.update(evaluator, contacts=[[float('nan'), 0, 0]])
        with self.assertRaises(ValueError):
            self.update(evaluator, dt=0)
        reflection = np.diag([-1, 1, 1, 1])
        with self.assertRaises(ValueError):
            pose_matrix(reflection)
        self.assertEqual(evaluator.elapsed, 0)

    def test_runtime_override_validation(self):
        for override in ({'settle_steps': 1.5}, {'min_contact_bodies': 0},
                         {'episode_timeout_s': float('inf')}, {'typo': 1}):
            task = deepcopy(self.tasks[0])
            task['runtime'].update(override)
            with self.assertRaises(ValueError):
                validate_task(task)

    def test_replay_validates_task_names_limits_and_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'actions.json'
            valid = {'task': self.tasks[0]['id'], 'actions': [
                {'joint_targets_rad': {'Joint1_R': 0}, 'control_steps': 2}]}
            path.write_text(json.dumps(valid))
            self.assertEqual(len(read_actions(path, self.tasks[0])), 1)
            for modify in ('task', 'joint', 'duration', 'limit'):
                value = deepcopy(valid)
                if modify == 'task':
                    value['task'] = 'wrong_task'
                elif modify == 'joint':
                    value['actions'][0]['joint_targets_rad'] = {'not_a_joint': 0}
                elif modify == 'duration':
                    value['actions'][0]['control_steps'] = -1
                else:
                    value['actions'][0]['joint_targets_rad']['Joint1_R'] = 100
                path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    read_actions(path, self.tasks[0])

    def test_manifest_rejects_misaligned_local_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root / 'tasks' / self.tasks[0]['id']
            folder.mkdir(parents=True)
            (root / 'tasks/index.json').write_text(json.dumps([self.tasks[0]]))
            changed = deepcopy(self.tasks[0])
            changed['preferred_hand'] = 'right'
            (folder / 'task.json').write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                load_tasks(root)

    def test_runtime_report_requires_fresh_complete_smoke_evidence(self):
        task, fingerprint = self.tasks[0], {'scene': 'test-hash'}
        report = {'task': task['id'], 'inputs_sha256': fingerprint, 'runtime_status': 'PASS',
                  'mode': 'reset_contact_smoke', 'reset_repeatable': True, 'sensor_count': 5,
                  'task_success': False, 'evaluation': {'reason': 'running'},
                  'resets': [{'stable': True, 'physics_steps': 1200, 'seconds': 5}] * 2}
        self.assertEqual(runtime_evidence_status(report, task, fingerprint), 'PASS')
        self.assertEqual(runtime_evidence_status(report, task, {'scene': 'changed'}), 'STALE')
        changed = deepcopy(report)
        changed['task'] = self.tasks[1]['id']
        self.assertEqual(runtime_evidence_status(changed, task, fingerprint), 'STALE')
        for key, value in [('mode', 'joint_replay'), ('resets', report['resets'][:1]),
                           ('sensor_count', 4), ('reset_repeatable', False)]:
            changed = deepcopy(report)
            changed[key] = value
            self.assertEqual(runtime_evidence_status(changed, task, fingerprint), 'INCOMPLETE')


if __name__ == '__main__':
    unittest.main()
