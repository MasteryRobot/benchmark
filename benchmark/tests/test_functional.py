"""Synthetic counterexamples to false hand manipulation claims."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from functional.evaluator import FunctionalEvaluator

ROOT = Path(__file__).resolve().parents[1]


def contract():
    return json.loads((ROOT / 'functional/example_contract.json').read_text())


def observation(angle=0, bodies=('thumb', 'index'), mass=0):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', angle, degrees=True).as_matrix()
    return {'object_pose_world': pose, 'palm_pose_world': {'left': np.eye(4), 'right': np.eye(4)},
            'finger_positions_rad': {'left': [angle / 180, 0]},
            'contacts': {'left': [{'body': b, 'force_n': .1, 'point_object_m': [0, -.04, 0]}
                                  for b in bodies], 'right': []},
            'lift_above_settled_m': .1, 'external_support_force_n': 0,
            'outcomes': {'delivered_mass_kg': mass}}


def acquire(evaluator):
    for _ in range(15):
        evaluator.update(observation(), .01)


def ready(evaluator, mass=0):
    obs = observation(90, ('thumb', 'middle'), mass)
    for _ in range(25):
        result = evaluator.update(obs, .01)
    return obs, result


class FunctionalTests(unittest.TestCase):
    def test_relative_change_contact_regrasp_then_new_outcome_required(self):
        evaluator = FunctionalEvaluator(contract()); acquire(evaluator)
        obs, result = ready(evaluator)
        self.assertEqual(result['stage'], 'use')
        self.assertFalse(result['predicate_success'])
        obs['outcomes']['delivered_mass_kg'] = .01
        for _ in range(24):
            self.assertFalse(evaluator.update(obs, .01)['predicate_success'])
        self.assertTrue(evaluator.update(obs, .01)['predicate_success'])
        with self.assertRaises(RuntimeError): evaluator.update(obs, .01)

    def test_world_rotation_without_relative_motion_cannot_pass(self):
        spec = contract(); spec['track'] = 'free_wrist'
        evaluator = FunctionalEvaluator(spec); acquire(evaluator)
        obs = observation(90, ('thumb', 'middle'), .1)
        obs['palm_pose_world']['left'] = np.array(obs['object_pose_world'])
        for _ in range(100): result = evaluator.update(obs, .01)
        self.assertFalse(result['predicate_success'])
        self.assertFalse(result['regrasp_evidence'])
        self.assertAlmostEqual(result['relative_rotation_error_deg'], 90)

    def test_world_transform_invariance(self):
        a, b = FunctionalEvaluator(contract()), FunctionalEvaluator(contract())
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', [20, 10, 40], degrees=True).as_matrix()
        transform[:3, 3] = [2, -3, 1]
        samples = [observation()] * 15 + [observation(90, ('thumb', 'middle'))] * 25
        samples += [observation(90, ('thumb', 'middle'), .01)] * 25
        for obs in samples:
            shifted = deepcopy(obs)
            shifted['object_pose_world'] = transform @ obs['object_pose_world']
            shifted['palm_pose_world'] = {h: transform @ p for h, p in obs['palm_pose_world'].items()}
            ra, rb = a.update(obs, .01), b.update(shifted, .01)
            self.assertEqual(ra['predicate_success'], rb['predicate_success'])
            self.assertAlmostEqual(ra['relative_position_error_m'], rb['relative_position_error_m'])

    def test_stale_outcome_cannot_pass(self):
        evaluator = FunctionalEvaluator(contract()); acquire(evaluator)
        obs, _ = ready(evaluator, .01)
        for _ in range(80): result = evaluator.update(obs, .01)
        self.assertFalse(result['predicate_success'])

    def test_wrong_region_or_unchanged_contact_bodies_cannot_pass(self):
        for wrong_region in (False, True):
            evaluator = FunctionalEvaluator(contract()); acquire(evaluator)
            obs = observation(90, ('thumb', 'middle') if wrong_region else ('thumb', 'index'), .1)
            if wrong_region:
                obs['contacts']['left'][0]['point_object_m'] = [0, .04, 0]
            for _ in range(80): result = evaluator.update(obs, .01)
            self.assertFalse(result['predicate_success'])
            self.assertEqual(result['stage'], 'reorient')

    def test_contact_gap_support_and_arm_cheating_are_terminal(self):
        for kind in ('gap', 'support', 'wrist'):
            evaluator = FunctionalEvaluator(contract()); acquire(evaluator)
            obs = observation(90, ('thumb', 'middle'))
            if kind == 'gap': obs['contacts']['left'] = []
            if kind == 'support': obs['external_support_force_n'] = .1
            if kind == 'wrist': obs['palm_pose_world']['left'][0, 3] = .1
            result = evaluator.update(obs, .01)
            self.assertTrue(result['done']); self.assertFalse(result['predicate_success'])

    def test_assisting_hand_preserves_contact_but_not_functional_goal(self):
        evaluator = FunctionalEvaluator(contract()); acquire(evaluator)
        obs = observation()
        obs['contacts']['right'] = obs['contacts']['left']; obs['contacts']['left'] = []
        result = evaluator.update(obs, .01)
        self.assertFalse(result['done']); self.assertFalse(result['predicate_success'])
        self.assertEqual(result['stage'], 'reorient')

    def test_missing_measurements_nan_and_duplicate_body_do_not_pass(self):
        evaluator = FunctionalEvaluator(contract())
        with self.assertRaises(ValueError): evaluator.update(observation(), float('nan'))
        obs = observation(); del obs['contacts']
        with self.assertRaises(KeyError): evaluator.update(obs, .01)
        evaluator.reset()
        obs = observation(bodies=('thumb', 'thumb'))
        for _ in range(40): result = evaluator.update(obs, .01)
        self.assertEqual(result['stage'], 'acquire')


if __name__ == '__main__': unittest.main()
