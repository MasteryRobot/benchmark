"""Behavioral regression checks; synthetic observations are unit tests only."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sharpa.recipes import scenarios
from sharpa.contract import validate,load_scenarios,ROOT
from sharpa.evaluator import ScenarioEvaluator
from build_sharpa_scenes import layer
from check_sharpa_scenes import geometry_check
from run_sharpa_episode import read_actions,repeatability


def observation(spec,at_goals=False,contact=False):
    out={'bodies':{},'ball_racket_contact_n':[0,0,0], 'capture_frame_id':None,'capture_valid':False}
    for b in spec['bodies']:
        if b['dynamic']:
            out['bodies'][b['id']]=dict(position=list(b['position']),orientation_wxyz=list(b['orientation_wxyz']),
                linear_velocity_m_s=[0,0,0],angular_velocity_rad_s=[0,0,0],
                contacts_n={h:[[.01,0,0],[-.01,0,0]] if contact else [[0,0,0],[0,0,0]] for h in ('left','right')})
    if at_goals:
        for g in spec['goals']:
            out['bodies'][g['body']].update(position=list(g['position']),orientation_wxyz=list(g['orientation_wxyz']))
    return out


class ScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.specs={s['id']:s for s in scenarios()}

    def test_manifest_layer_and_initial_geometry_all_scenes(self):
        loaded=load_scenarios()
        self.assertEqual(len(loaded),72)
        self.assertEqual(len([s for s in loaded if s['task_type']=='scene_only']),47)
        for spec in loaded:
            with self.subTest(task=spec['id']):
                validate(spec)
                path=ROOT/'sharpa/scenes'/spec['id']/'scene.usda'
                self.assertEqual(path.read_text(),layer(spec))
                self.assertEqual(geometry_check(spec)['status'],'PASS')
                self.assertIsNone(spec['official_source'])
                self.assertTrue(spec['source_status'].startswith('UNVERIFIED'))

    def test_every_evaluated_initial_state_is_unsuccessful(self):
        for s in self.specs.values():
            e=ScenarioEvaluator(s);obs=observation(s)
            for _ in range(65):r=e.update(obs,1/240)
            self.assertFalse(r['success'],s['id'])

    def test_released_cards_require_prior_touch_all_goals_and_hold(self):
        s=self.specs['sharpa_card_dealing'];e=ScenarioEvaluator(s)
        obs=observation(s,True)
        for _ in range(70):r=e.update(obs,1/240)
        self.assertFalse(r['success'])
        e.update(observation(s,contact=True),1/240)
        for _ in range(59):self.assertFalse(e.update(obs,1/240)['success'])
        self.assertTrue(e.update(obs,1/240)['success'])
        with self.assertRaises(RuntimeError):e.update(obs,1/240)
        e.reset();obs['bodies']['card_5']['position'][0]+=.1
        e.update(observation(s,contact=True),1/240)
        for _ in range(70):r=e.update(obs,1/240)
        self.assertFalse(r['success'])

    def test_photography_requires_current_frame_at_bimanual_pose(self):
        s=self.specs['sharpa_camera_photography'];e=ScenarioEvaluator(s);obs=observation(s,True,True)
        for _ in range(70):r=e.update(obs,1/240)
        self.assertFalse(r['success'])
        obs.update(capture_frame_id=1,capture_valid=True)
        for _ in range(60):r=e.update(obs,1/240)
        self.assertTrue(r['success'])
        e.reset();bad=observation(s,contact=True);bad.update(capture_frame_id=1,capture_valid=True)
        e.update(bad,1/240)
        for _ in range(70):r=e.update(obs,1/240)
        self.assertFalse(r['success'],'Stale frame must not become valid after moving camera')
        obs['capture_frame_id']=2
        obs['bodies']['camera_body']['contacts_n']['left']=[[0,0,0],[0,0,0]]
        for _ in range(70):r=e.update(obs,1/240)
        self.assertFalse(r['success'])

    def test_tennis_requires_held_racket_contact_then_valid_crossing(self):
        s=self.specs['sharpa_table_tennis'];e=ScenarioEvaluator(s);obs=observation(s,contact=True)
        obs['bodies']['ball']['position']=[0,.40,1.0]
        e.update(obs,1/240)
        obs['bodies']['ball']['position']=[0,.64,1.0];obs['bodies']['ball']['linear_velocity_m_s']=[0,1,0]
        self.assertFalse(e.update(obs,1/240)['success'])
        e.reset();obs['bodies']['ball']['position']=[0,.40,1.0];obs['ball_racket_contact_n']=[0,.1,0]
        self.assertFalse(e.update(obs,1/240)['success'])
        obs['ball_racket_contact_n']=[0,0,0];obs['bodies']['ball']['position']=[0,.64,1.0]
        self.assertTrue(e.update(obs,1/240)['success'])

    def test_catch_needs_contact_low_speed_and_hold(self):
        s=self.specs['sharpa_baton_catching'];e=ScenarioEvaluator(s);obs=observation(s,contact=True)
        obs['bodies']['baton']['linear_velocity_m_s']=[0,0,1]
        for _ in range(65):r=e.update(obs,1/240)
        self.assertFalse(r['success'])
        obs['bodies']['baton']['linear_velocity_m_s']=[0,0,0]
        for _ in range(60):r=e.update(obs,1/240)
        self.assertTrue(r['success'])

    def test_scene_only_never_reports_task_success(self):
        s=next(s for s in self.specs.values() if s['task_type']=='scene_only');e=ScenarioEvaluator(s)
        r=e.update(observation(s,contact=True),s['runtime']['episode_timeout_s'])
        self.assertFalse(r['success']);self.assertTrue(r['truncated'])
        self.assertEqual(r['task_evaluation_status'],'NOT_IMPLEMENTED')

    def test_bad_observations_and_missing_state_fail(self):
        s=self.specs['sharpa_card_dealing'];e=ScenarioEvaluator(s);obs=observation(s)
        obs['bodies']['card_1']['contacts_n']['left']=[[float('nan'),0,0]]
        with self.assertRaises(ValueError):e.update(obs,1/240)
        with self.assertRaises(ValueError):e.update(observation(s),float('nan'))
        bad=deepcopy(s);bad['id']='../../escape'
        with self.assertRaises(ValueError):validate(bad)

    def test_replay_rejects_invalid_targets_and_cross_task_capture(self):
        s=self.specs['sharpa_card_dealing']
        for action in [{'joint_targets_rad':{'bad_joint':0}}, {'control_steps':True}, {'capture':True},
                       {'joint_targets_rad':{'Joint1_L':float('nan')}}, {'teleport_object':[0,0,0]}]:
            with tempfile.TemporaryDirectory() as d:
                path=Path(d)/'actions.json';path.write_text(json.dumps({'task':s['id'],'actions':[action]}))
                with self.assertRaises(ValueError):read_actions(path,s)

    def test_repeatability_checks_all_bodies(self):
        first=dict(bodies=[dict(body='a',position=[0,0,0],orientation_wxyz=[1,0,0,0]),
                           dict(body='b',position=[0,0,0],orientation_wxyz=[1,0,0,0])])
        second=deepcopy(first);second['bodies'][1]['position'][0]=.01
        self.assertFalse(repeatability([first,second])['reset_repeatable'])


if __name__=='__main__':unittest.main()
