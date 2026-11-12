"""Evaluate measured traces in the palm frame, independently of world motion.

This is an offline predicate implementation, not a simulator or a grasp policy.
Contacts must be object/hand pairs with positions in the object's local frame.
"""
from copy import deepcopy
import math
import numpy as np
from task_runtime import pose_matrix


def pose_error(a, b):
    a, b = pose_matrix(a), pose_matrix(b)
    distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    angle = math.degrees(math.acos(float(np.clip(
        (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1))))
    return distance, angle


def scalar(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if value < 0 or (positive and value == 0):
        raise ValueError(f'{name} must be nonnegative/positive')
    return float(value)


class FunctionalEvaluator:
    """Acquire -> change grasp -> functional pose -> newly measured task outcome.

    Two tracks: fixed_wrist enforces a wrist motion budget during reorientation;
    free_wrist permits arm motion but still requires object/palm relative change.
    Losing all hand contacts or using external support after pickup invalidates
    this airborne track. Table-assisted manipulation needs a separate contract.
    """
    def __init__(self, spec):
        self.spec = deepcopy(spec)
        if spec['track'] not in ('fixed_wrist', 'free_wrist'):
            raise ValueError('Unknown track')
        if spec['primary_hand'] not in ('left', 'right'):
            raise ValueError('Unknown primary hand')
        self.goal = pose_matrix(spec['goal_object_in_palm'])
        self.region = np.asarray(spec['allowed_contact_box_object_m'], float)
        if (self.region.shape != (2, 3) or not np.isfinite(self.region).all()
                or np.any(self.region[0] >= self.region[1])):
            raise ValueError('Invalid contact region')
        keys = ('position_tolerance_m', 'rotation_tolerance_deg', 'min_relative_translation_m',
                'min_relative_rotation_deg', 'min_finger_change_rad', 'min_contact_force_n',
                'minimum_lift_m', 'grasp_hold_s', 'goal_hold_s', 'outcome_hold_s',
                'max_wrist_translation_m', 'max_wrist_rotation_deg', 'episode_timeout_s',
                'outcome_minimum_gain')
        self.cfg = {k: scalar(spec[k], k, True) for k in keys}
        self.support_mode = spec.get('support_mode', 'airborne')
        if self.support_mode not in ('airborne', 'articulated_handle'):
            raise ValueError('Unknown support mode')
        self.patch_threshold = scalar(spec.get('min_contact_patch_motion_m', .005), 'patch motion', True)
        self.reset()

    def reset(self):
        self.elapsed = self.grasp_hold = self.goal_hold = self.outcome_hold = 0.0
        self.anchor = None
        self.invalid = None
        self.done = self.ready = self.regrasp = False
        self.use_started = False
        self.outcome_baseline = None
        self.max_wrist_translation = self.max_wrist_rotation = 0.0

    def update(self, obs, dt):
        if self.done:
            raise RuntimeError('Episode finished; reset before updating')
        dt = scalar(dt, 'dt', True)
        obj = pose_matrix(obs['object_pose_world'])
        palms = {h: pose_matrix(obs['palm_pose_world'][h]) for h in ('left', 'right')}
        h = self.spec['primary_hand']
        relative = np.linalg.inv(palms[h]) @ obj
        joints = np.asarray(obs['finger_positions_rad'][h], float)
        if joints.ndim != 1 or not joints.size or not np.isfinite(joints).all():
            raise ValueError('Expected finite finger joint vector')
        contacts = {}
        for hand in ('left', 'right'):
            contacts[hand] = {}
            for contact in obs['contacts'][hand]:
                force = scalar(contact['force_n'], 'force_n')
                point = np.asarray(contact['point_object_m'], float)
                if point.shape != (3,) or not np.isfinite(point).all() or not contact['body']:
                    raise ValueError('Invalid measured object/hand contact')
                if force >= self.cfg['min_contact_force_n']:
                    contacts[hand].setdefault(contact['body'], []).append(point)
        lift = obs['lift_above_settled_m']
        if not isinstance(lift, (float, int)) or isinstance(lift, bool) or not math.isfinite(lift):
            raise ValueError('Invalid measured lift')
        support = scalar(obs['external_support_force_n'], 'external_support_force_n')
        outcome = scalar(obs['outcomes'][self.spec['outcome_metric']], 'outcome')
        self.elapsed += dt
        held = len(contacts[h]) >= 2
        # Task fixtures (nail, floor under broom, drawer rails) may exert forces
        # during use. They must be separated from unapproved object support by
        # the measured-state adapter; otherwise this predicate fails closed.
        task_contact = self.use_started and self.spec.get('allow_task_contact_during_use', False)
        if task_contact:
            support = scalar(obs['unapproved_support_force_n'], 'unapproved_support_force_n')
        airborne = ((lift >= self.cfg['minimum_lift_m'] or task_contact)
                    and support < self.cfg['min_contact_force_n'])
        if self.support_mode == 'articulated_handle':
            airborne = obs['handle_attached_to_drawer'] is True
        position_error, rotation_error = pose_error(relative, self.goal)
        region_ok = held and all(
            np.all(p >= self.region[0]) and np.all(p <= self.region[1])
            for points in contacts[h].values() for p in points)
        signature = set(contacts[h])
        if self.anchor is None:
            self.grasp_hold = self.grasp_hold + dt if held and airborne else 0.0
            if self.grasp_hold + 1e-10 >= self.cfg['grasp_hold_s']:
                self.anchor = (relative.copy(), palms[h].copy(), joints.copy(), signature,
                               {body: np.mean(points, axis=0) for body, points in contacts[h].items()})
        else:
            if joints.shape != self.anchor[2].shape:
                raise ValueError('Finger joint vector changed shape')
            if not any(contacts.values()):
                self.invalid = 'lost_all_hand_contacts'
            elif not airborne:
                self.invalid = 'external_support_or_insufficient_lift'
            if not self.use_started or self.spec.get('limit_wrist_during_use', False):
                distance, angle = pose_error(palms[h], self.anchor[1])
                self.max_wrist_translation = max(self.max_wrist_translation, distance)
                self.max_wrist_rotation = max(self.max_wrist_rotation, angle)
                if self.spec['track'] == 'fixed_wrist' and (
                    distance > self.cfg['max_wrist_translation_m']
                    or angle > self.cfg['max_wrist_rotation_deg']):
                    self.invalid = 'wrist_budget_exceeded'
            distance, angle = pose_error(relative, self.anchor[0])
            changed = (distance >= self.cfg['min_relative_translation_m']
                       or angle >= self.cfg['min_relative_rotation_deg'])
            if self.spec.get('require_relative_rotation', False):
                changed = angle >= self.cfg['min_relative_rotation_deg']
            finger_changed = np.max(np.abs(joints - self.anchor[2])) >= self.cfg['min_finger_change_rad']
            patch_changed = self.spec.get('allow_contact_patch_motion', False) and any(
                np.linalg.norm(np.mean(points, axis=0) - self.anchor[4][body]) >= self.patch_threshold
                for body, points in contacts[h].items() if body in self.anchor[4])
            self.regrasp = self.regrasp or bool(changed and finger_changed and held
                                               and (signature != self.anchor[3] or patch_changed))
        goal = (self.anchor is not None and self.regrasp and held and region_ok and airborne
                and position_error <= self.cfg['position_tolerance_m']
                and rotation_error <= self.cfg['rotation_tolerance_deg'] and self.invalid is None)
        self.goal_hold = self.goal_hold + dt if goal else 0.0
        was_ready = self.ready
        self.ready = self.goal_hold + 1e-10 >= self.cfg['goal_hold_s']
        if self.use_started and self.spec.get('dynamic_relative_pose_during_use', False):
            self.ready = bool(held and region_ok and airborne and self.invalid is None)
        self.use_started = self.use_started or self.ready
        if not self.ready or not was_ready:
            self.outcome_baseline = outcome
            self.outcome_hold = 0.0
        gain = outcome - self.outcome_baseline
        if self.ready and was_ready:
            self.outcome_hold = self.outcome_hold + dt if gain >= self.cfg['outcome_minimum_gain'] else 0.0
        success = self.outcome_hold + 1e-10 >= self.cfg['outcome_hold_s']
        timeout = self.elapsed + 1e-10 >= self.cfg['episode_timeout_s']
        self.done = bool(success or timeout or self.invalid)
        return {'predicate_success': bool(success), 'done': self.done,
                'reason': self.invalid or ('success' if success else 'timeout' if timeout else 'running'),
                'stage': 'use' if self.ready else 'reorient' if self.anchor is not None else 'acquire',
                'relative_position_error_m': position_error, 'relative_rotation_error_deg': rotation_error,
                'regrasp_evidence': self.regrasp, 'functional_contact_region': region_ok,
                'wrist_translation_max_m': self.max_wrist_translation,
                'wrist_rotation_max_deg': self.max_wrist_rotation,
                'outcome_gain_since_ready': gain, 'elapsed_s': self.elapsed}
