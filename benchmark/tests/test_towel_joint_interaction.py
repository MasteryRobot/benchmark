"""Offline limits and command-boundary checks, not a physical grasp test."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from towel_roll.joint_plan import generate, validate
from towel_roll.geometry import robot_config
from towel_roll.surface import mesh_for_variant

ROOT=Path(__file__).resolve().parents[1]


class JointInteractionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ROOT/'towel_roll/interaction_config.json').read_text())
        cls.plan=generate(cls.config)

    def test_trajectory_has_reachable_palms_and_bounded_joint_speed(self):
        validate(self.plan)
        self.assertLessEqual(self.plan['max_position_error_m'],self.config['ik_position_tolerance_m'])
        self.assertLessEqual(self.plan['max_rotation_error_rad'],self.config['ik_rotation_tolerance_rad'])
        robot,_=robot_config()
        velocities={j.get('name'):float(j.find('limit').get('velocity')) for j in robot.revolute}
        for a,b in zip(self.plan['samples'],self.plan['samples'][1:]):
            for name,v in velocities.items():
                speed=abs(b['joint_targets_rad'][name]-a['joint_targets_rad'][name])/(b['time_s']-a['time_s'])
                self.assertLessEqual(speed,v*self.config['max_joint_speed_fraction']+1e-8)

    def test_vertex_commands_and_excessive_joint_speed_are_rejected(self):
        bad=deepcopy(self.plan);bad['samples'][2]['cloth_vertices']=[[0,0,0]]
        with self.assertRaisesRegex(ValueError,'Only joint'):validate(bad)
        bad=deepcopy(self.plan);bad['samples'][2]['time_s']=bad['samples'][1]['time_s']+1e-9
        with self.assertRaisesRegex(ValueError,'speed'):validate(bad)

    def test_unreachable_targets_stop_planning(self):
        bad=deepcopy(self.config);bad['variant']['near_y_m']=2.
        with self.assertRaisesRegex(ValueError,'Unreachable'):generate(bad)

    def test_thick_towel_mesh_starts_at_physical_surface_height(self):
        v=self.config['variant'];points,_=mesh_for_variant(v)
        self.assertEqual(v['thickness_m'],.008)
        self.assertAlmostEqual(points[:,2].min(),.795+.03+.004)


if __name__=='__main__':unittest.main()
