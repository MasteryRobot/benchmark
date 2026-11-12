"""Contact-triggered grasp/lift diagnostic, not a functional regrasp policy."""
import numpy as np
from representative_policy import CandidatePolicy


class ContactGraspProbe(CandidatePolicy):
    def __init__(self, env, variant=0):
        super().__init__(env, variant)
        self.contact_hold = 0.
        self.info.update(name='contact_triggered_grasp_probe',
                         scope='Grasp prerequisite only; not in-hand regrasp or tool use')

    def action(self, observation, elapsed):
        forces = observation['contact_forces_n']
        count = np.count_nonzero(np.linalg.norm(forces, axis=1) >= self.env.config['min_contact_force_n'])
        if elapsed >= 5 and self.grasp_relative is None:
            dt = self.env.config['physics_dt_s'] * self.env.config['control_decimation']
            self.contact_hold = self.contact_hold + dt if count >= 2 else 0.
            if self.contact_hold >= .1:
                self.grasp_relative = np.linalg.inv(self.hand_frame(observation)) @ observation['object_pose_world']
                self.grasp_start = elapsed
                self.grasp_object = observation['object_pose_world'].copy()
        return super().action(observation, elapsed)
