"""Measured-state predicates for prototype manipulation episodes.

Offline tests may inject observations to test logic, but no runner generates
fake contact forces, object trajectories or camera frames to claim success.
"""
from copy import deepcopy
import math
import numpy as np
from .contract import validate, vector, rotation


class ScenarioEvaluator:
    def __init__(self, spec):
        self.spec = deepcopy(validate(spec))
        self.cfg = self.spec['runtime']
        self.reset()

    def reset(self):
        self.elapsed = self.hold = 0.0
        self.done = False
        self.touched = set()
        self.racket_hit = False
        self.previous_ball_position = None
        self.returned = False
        self.captured = False
        self.last_capture_frame = None

    def _contacts(self, state, hand):
        f = np.asarray(state['contacts_n'][hand], dtype=float)
        if f.ndim != 2 or f.shape[1] != 3 or not np.isfinite(f).all():
            raise ValueError('Measured contacts must be a finite (bodies, 3) matrix')
        return int(np.count_nonzero(np.linalg.norm(f, axis=1) >= self.cfg['min_contact_force_n']))

    def update(self, observation, dt):
        if self.done:
            raise RuntimeError('Episode ended; reset before stepping')
        if isinstance(dt, bool) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive')
        states = observation['bodies']
        counts, slow = {}, {}
        for obj in self.spec['bodies']:
            if not obj['dynamic']:
                continue
            key = obj['id']; state = states[key]
            vector(state['position'], 3, 'position'); rotation(state['orientation_wxyz'])
            linear = vector(state['linear_velocity_m_s'], 3, 'linear velocity')
            angular = vector(state['angular_velocity_rad_s'], 3, 'angular velocity')
            counts[key] = {hand: self._contacts(state, hand) for hand in ('left', 'right')}
            if sum(counts[key].values()):
                self.touched.add(key)
            slow[key] = (np.linalg.norm(linear) <= self.cfg['max_linear_speed_m_s']
                         and np.linalg.norm(angular) <= self.cfg['max_angular_speed_rad_s'])
        kind = self.spec['task_type']
        criteria, details = {}, {}
        for target in self.spec['goals']:
            key = target['body']; state = states[key]
            distance = float(np.linalg.norm(np.asarray(state['position']) - target['position']))
            R = rotation(target['orientation_wxyz']).T @ rotation(state['orientation_wxyz'])
            angle = math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2, -1, 1))))
            c = counts[key]; minimum = self.cfg['min_contact_bodies']
            rule = target['contact']
            contacts_ok = (sum(c.values()) == 0 if rule == 'released' else
                           all(n >= minimum for n in c.values()) if rule == 'both_hands' else
                           any(n >= minimum for n in c.values()))
            tests = dict(position=distance <= target['position_tolerance_m'],
                         rotation=angle <= target['rotation_tolerance_deg'], stable=bool(slow[key]),
                         contact=contacts_ok, manipulated=key in self.touched or
                         not target.get('require_prior_hand_contact', True))
            criteria[key] = all(tests.values())
            details[key] = dict(position_error_m=distance, rotation_error_deg=angle,
                                contact_bodies=c, checks=tests)
        if kind == 'photograph':
            frame = observation.get('capture_frame_id')
            # A new frame must be captured while the camera is correctly held.
            if frame is not None and frame != self.last_capture_frame:
                self.captured = all(criteria.values()) and observation.get('capture_valid') is True
                self.last_capture_frame = frame
            if not all(criteria.values()):
                self.captured = False
            criteria['rendered_capture'] = self.captured
        elif kind == 'catch':
            key = self.spec['caught_body']
            criteria['held'] = any(n >= self.cfg['min_contact_bodies'] for n in counts[key].values())
            criteria['stable'] = bool(slow[key])
            criteria['above_table'] = states[key]['position'][2] > .86
        elif kind == 'return_ball':
            ball, racket = self.spec['ball'], self.spec['racket']
            force = vector(observation['ball_racket_contact_n'], 3, 'ball/racket force')
            if (np.linalg.norm(force) >= self.cfg['min_contact_force_n']
                    and any(n >= self.cfg['min_contact_bodies'] for n in counts[racket].values())):
                self.racket_hit = True
            pos = vector(states[ball]['position'], 3, 'ball position')
            plane = self.spec['return_plane_y']
            crossing = (self.previous_ball_position is not None
                        and self.previous_ball_position[1] < plane <= pos[1]
                        and states[ball]['linear_velocity_m_s'][1] > 0)
            if (self.racket_hit and crossing and self.spec['return_min_z'] <= pos[2] <= self.spec['return_max_z']
                    and abs(pos[0]) <= self.spec['return_max_abs_x']):
                self.returned = True
            self.previous_ball_position = pos.copy()
            criteria.update(measured_held_racket_hit=self.racket_hit, valid_return=self.returned)
        self.elapsed += dt
        self.hold = self.hold + dt if criteria and all(criteria.values()) else 0.0
        # Return is a discrete crossing event; the other tasks require dwell.
        success = self.returned if kind == 'return_ball' else self.hold + 1e-9 >= self.cfg['success_hold_s']
        fallen = any(state['position'][2] < .68 for state in states.values())
        timeout = self.elapsed + 1e-9 >= self.cfg['episode_timeout_s']
        self.done = bool(success or fallen or timeout)
        return dict(success=bool(success), terminated=bool(success or fallen),
                    truncated=bool(timeout and not (success or fallen)),
                    reason='success' if success else 'fallen' if fallen else 'timeout' if timeout else 'running',
                    elapsed_s=self.elapsed, hold_s=self.hold, reward=float(success),
                    criteria=criteria, details=details,
                    task_evaluation_status='NOT_IMPLEMENTED' if kind=='scene_only' else 'LOCAL_APPROXIMATION')
