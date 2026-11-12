"""Geometry and entry-point checks; no substitute for Isaac physical tests."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from towel_roll.config import SPEC
from towel_roll.surface import mesh_for_variant
from towel_roll.prepare_isaac import prepare

ROOT = Path(__file__).resolve().parents[1]


class TowelPreparationTests(unittest.TestCase):
    def test_all_variants_have_correct_extent_area_and_support_height(self):
        variants = json.loads((ROOT / 'towel_roll/isaac_config.json').read_text())['variants']
        for variant in variants:
            points, faces = mesh_for_variant(variant)
            np.testing.assert_allclose(np.ptp(points, axis=0)[:2], [variant['width_m'], variant['length_m']])
            self.assertAlmostEqual(points[:, 2].min(), SPEC['towel']['table_height_m']
                                   + variant['support_height_m'] + variant['thickness_m'] / 2)
            triangles = points[faces]
            normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            self.assertTrue(np.all(normals[:, 2] > 0))
            self.assertAlmostEqual(np.linalg.norm(normals, axis=1).sum() / 2,
                                   variant['width_m'] * variant['length_m'])

    def test_invalid_geometry_rejected(self):
        variant = json.loads((ROOT / 'towel_roll/isaac_config.json').read_text())['variants'][0]
        for spacing in (0, float('nan'), .000001):
            with self.assertRaises(ValueError): mesh_for_variant(variant, spacing)

    def test_missing_driver_or_runtime_never_becomes_a_physics_attempt(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'blocked.json'
            with (patch('towel_roll.prepare_isaac.subprocess.run', return_value=
                       subprocess.CompletedProcess([], 9, '', 'driver unavailable')),
                  patch('towel_roll.prepare_isaac.importlib.util.find_spec', return_value=None)):
                self.assertEqual(prepare(output, prepare_only=True), 2)
            report = json.loads(output.read_text())
            self.assertFalse(report['physics_started']); self.assertFalse(report['task_success'])
            self.assertEqual(report['status'], 'BLOCKED')
            with self.assertRaises(FileExistsError): prepare(output)

    def test_unified_towel_entry_reaches_preflight(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'report.json'
            # Without --prepare-only the unavailable controller is an explicit
            # blocker on every host; this never imports or launches SimulationApp.
            result = subprocess.run([sys.executable, '-B', str(ROOT / 'benchmark_cli.py'),
                                     'run', 'bimanual_roll_towel', '--output', str(output)],
                                    cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertTrue(output.exists(), result.stderr)
            self.assertFalse(json.loads(output.read_text())['physics_started'])


if __name__ == '__main__': unittest.main()
