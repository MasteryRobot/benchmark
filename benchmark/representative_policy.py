"""Experimental state-based candidate controller; success requires live PhysX evaluation.

No object pose writes, artificial forces, attachments or fabricated contacts.
This controller targets world-frame object reorientation, not certified in-hand rotation.
"""
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from robot_model import transform
from task_runtime import ROOT


class CandidatePolicy:
    def __init__(self, env, variant=0):
        self.env = env
        self.hand = env.task['preferred_hand']
        self.prefix = 'l_f_' if self.hand == 'left' else 'r_f_'
        self.tip = self.hand + '_hand_base'
        self.chain = 'R' if self.hand == 'left' else 'L'
        self.arm_names = [f'Joint{k}_{self.chain}' for k in range(1, 8)]
        self.base = np.asarray(json.loads((ROOT / 'poses.json').read_text())['robots']['DualArm']['base_transform'])
        self.q = dict(zip(env.names, env.q0))
        self.initial_q = dict(self.q)
        self.variant = variant
        reach = next(x for x in json.loads((ROOT / 'tasks/task_reach_checks.json').read_text())
                     if x['task'] == env.task['id'])
        self.pregrasp = dict(zip(reach['joint_names'], reach['pregrasp_q_rad']))
        probe = {**self.q, **self.pregrasp}
        self.approach_rotation = env.model.fk(probe, self.base)[self.tip][:3, :3]
        self.grasp_relative = None
        self.grasp_start = None
        self.phase = 'approach'
        self.info = {'name': 'state_feedback_ik_candidate', 'variant': variant,
                     'status': 'EXPERIMENTAL_UNVALIDATED',
                     'uses_privileged_state': True,
                     'scope': 'World-frame grasp/lift/reorientation; no assertion of in-hand manipulation',
                     'source_file': 'representative_policy.py'}

    def hand_frame(self, observation):
        q = dict(zip(self.env.names, observation['joint_positions_rad']))
        return self.env.model.fk(q, self.base)[self.tip]

    def solve_arm(self, target, observation):
        measured = dict(zip(self.env.names, observation['joint_positions_rad']))
        seed = np.array([measured[n] for n in self.arm_names])
        bounds = np.array([self.env.limits[n] for n in self.arm_names]).T

        def residual(q):
            values = {**measured, **dict(zip(self.arm_names, q))}
            frame = self.env.model.fk(values, self.base)[self.tip]
            position = frame[:3, 3] - target[:3, 3]
            rotation = Rotation.from_matrix(target[:3, :3] @ frame[:3, :3].T).as_rotvec()
            return np.r_[position, .15 * rotation, .002 * (q - seed)]

        solution = least_squares(residual, np.clip(seed, bounds[0], bounds[1]),
                                 bounds=bounds, max_nfev=35)
        return dict(zip(self.arm_names, solution.x))

    def action(self, observation, elapsed):
        target = dict(self.q)
        contact_count = np.count_nonzero(np.linalg.norm(observation['contact_forces_n'], axis=1)
                                        >= self.env.config['min_contact_force_n'])
        if elapsed < 2.5:
            self.phase = 'approach'
            alpha = min(elapsed / 2.5, 1.)
            for name, value in self.pregrasp.items():
                target[name] = self.initial_q[name] + alpha * (value - self.initial_q[name])
        elif elapsed < 5.0:
            self.phase = 'descend'
            reference = observation['object_pose_world'][:3, 3].copy()
            reference[2] += .075 * max(0., 1 - (elapsed - 2.5) / 2.5)
            hand = np.eye(4)
            hand[:3, :3] = self.approach_rotation
            hand[:3, 3] = reference - self.approach_rotation @ np.array([.02, 0., .10])
            target.update(self.solve_arm(hand, observation))
        elif self.grasp_relative is None:
            self.phase = 'close'
            closure = min((elapsed - 5.) / 2.5, 1.)
            for name in self.env.names:
                if not name.startswith(self.prefix):
                    continue
                lo, hi = self.env.limits[name]
                part = name.rsplit('_', 1)[-1]
                if part == '1':
                    closed = (-.85 if self.hand == 'left' else .85) if 'joint1_1' in name else 0.
                else:
                    closed = (.55 + .1 * self.variant) * hi
                target[name] = (1 - closure) * self.initial_q[name] + closure * np.clip(closed, lo, hi)
            if elapsed > 6.5 and contact_count >= self.env.config['min_contact_bodies']:
                self.grasp_relative = np.linalg.inv(self.hand_frame(observation)) @ observation['object_pose_world']
                self.grasp_start = elapsed
                self.grasp_object = observation['object_pose_world'].copy()
            elif elapsed > 10.0:
                raise RuntimeError('Candidate failed to establish measured multi-body grasp contact')
        else:
            self.phase = 'lift_and_reorient'
            alpha = min((elapsed - self.grasp_start) / 6., 1.)
            goal = np.asarray(self.env.task['goal_pose_world'])
            object_target = np.eye(4)
            object_target[:3, 3] = (1-alpha)*self.grasp_object[:3, 3] + alpha*goal[:3, 3]
            delta = Rotation.from_matrix(goal[:3, :3] @ self.grasp_object[:3, :3].T).as_rotvec()
            object_target[:3, :3] = Rotation.from_rotvec(alpha*delta).as_matrix() @ self.grasp_object[:3, :3]
            hand_target = object_target @ np.linalg.inv(self.grasp_relative)
            target.update(self.solve_arm(hand_target, observation))
        # Respect per-joint URDF velocity and position limits at the command boundary.
        dt = self.env.config['physics_dt_s'] * self.env.config['control_decimation']
        limits = {j.get('name'): float(j.find('limit').get('velocity')) for j in self.env.model.revolute}
        for name in target:
            max_step = .5 * limits[name] * dt
            lo, hi = self.env.limits[name]
            target[name] = float(np.clip(self.q[name] + np.clip(target[name]-self.q[name], -max_step, max_step), lo, hi))
        self.q = target
        return target
