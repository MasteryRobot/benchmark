"""Geometry-aware grasp prerequisite. No claim of in-hand or tool-use success."""
import numpy as np
from scipy.spatial.transform import Rotation
from representative_policy import CandidatePolicy


class GeometricGraspProbe(CandidatePolicy):
    def __init__(self, env, variant=0, config=None):
        super().__init__(env, variant)
        self.cfg={'yaw_deg':90.,'palm_grasp_x_m':.055,'palm_grasp_y_m':-.025,
                  'palm_grasp_z_m':.12,'thumb_opposition_rad':1.4,
                  'close_fraction':.65,'min_contact_duration_s':.1,
                  'grasp_force_n':.05,'lift_duration_s':5.0,'post_contact_closure_rad':.12,
                  'minimum_closure_fraction':.35,**(config or {})}
        self.contact_hold=0.;self.frozen_fingers=None;self.target_frame=None
        self.rotation=Rotation.from_euler('z',self.cfg['yaw_deg'],degrees=True).as_matrix()@self.approach_rotation
        self.offset=np.array([self.cfg['palm_grasp_x_m'],self.cfg['palm_grasp_y_m'],self.cfg['palm_grasp_z_m']])
        self.initial_hand=env.model.fk(self.q,self.base)[self.tip]
        self.step_count=0
        self.grip_feedback = None
        self.finger_groups = {}
        for joint in self.env.model.revolute:
            n = joint.get('name')
            if not n.startswith(self.prefix): continue
            # Preserve URDF's historical jiont typo; identify by finger number.
            import re
            finger = int(re.search(r'(?:joint|jiont)([1-5])_', n).group(1))
            self.finger_groups.setdefault(finger, []).append(n)
        self.body_fingers = {}
        for finger, joints in self.finger_groups.items():
            for joint in self.env.model.revolute:
                if joint.get('name') in joints:
                    link = joint.find('child').get('link')
                    self.body_fingers[self.env.model.groups[link]] = finger
        self.info.update(name='geometry_aware_grasp_probe',config=self.cfg,
            scope='Grasp/lift diagnostic only; no certified in-hand manipulation')

    def action(self, observation, elapsed):
        self.step_count+=1
        target=dict(self.q);dt=self.env.config['physics_dt_s']*self.env.config['control_decimation']
        point=(observation['object_pose_world']@np.r_[self.env.task['object_design']['grasp_local'],1])[:3]
        goal_hand=np.eye(4);goal_hand[:3,:3]=self.rotation
        goal_hand[:3,3]=point-self.rotation@self.offset
        if elapsed<3.0:
            self.phase='approach'
            goal_hand[2,3]+=.08
            u=np.clip(elapsed/3.,0,1);u=u*u*(3-2*u)
            goal_hand[:3,3]=(1-u)*self.initial_hand[:3,3]+u*goal_hand[:3,3]
            delta=Rotation.from_matrix(self.rotation@self.initial_hand[:3,:3].T).as_rotvec()
            goal_hand[:3,:3]=Rotation.from_rotvec(u*delta).as_matrix()@self.initial_hand[:3,:3]
        elif elapsed<6.:
            self.phase='descend';goal_hand[2,3]+=.08*max(0,1-(elapsed-3)/3)
        elif self.grasp_relative is None:
            self.phase='close'
            goal_hand=self.target_frame.copy()
            closure=min((elapsed-6)/4.,1.)
            for n in self.env.names:
                if not n.startswith(self.prefix):continue
                lo,hi=self.env.limits[n]
                if n.endswith('_1'):
                    final=(-1 if self.hand=='left' else 1)*self.cfg['thumb_opposition_rad'] if 'joint1_1' in n else 0.
                else:final=self.cfg['close_fraction']*hi
                target[n]=float((1-closure)*self.initial_q[n]+closure*np.clip(final,lo,hi))
            active=np.linalg.norm(observation['contact_forces_n'],axis=1)>=self.cfg['grasp_force_n']
            self.contact_hold=self.contact_hold+dt if np.count_nonzero(active)>=2 else 0.
            if self.contact_hold>=self.cfg['min_contact_duration_s'] and closure>=self.cfg['minimum_closure_fraction']:
                self.grasp_relative=np.linalg.inv(self.hand_frame(observation))@observation['object_pose_world']
                self.grasp_object=observation['object_pose_world'].copy();self.grasp_start=elapsed
                measured=dict(zip(self.env.names,observation['joint_positions_rad']))
                self.frozen_fingers={n:float(np.clip(measured[n]+(0 if n.endswith('_1') else self.cfg['post_contact_closure_rad']),
                    *self.env.limits[n])) for n in self.env.names if n.startswith(self.prefix)}
            elif elapsed>12:raise RuntimeError('Geometry probe failed to establish sustained multi-body contact')
        else:
            self.phase='lift_and_reorient'
            u=min((elapsed-self.grasp_start)/self.cfg['lift_duration_s'],1.)
            obj=self.grasp_object.copy();goal=np.asarray(self.env.task['goal_pose_world'])
            obj[:3,3]=(1-u)*obj[:3,3]+u*goal[:3,3]
            delta=Rotation.from_matrix(goal[:3,:3]@obj[:3,:3].T).as_rotvec()
            obj[:3,:3]=Rotation.from_rotvec(u*delta).as_matrix()@obj[:3,:3]
            goal_hand=obj@np.linalg.inv(self.grasp_relative)
            target.update(self.frozen_fingers)
            # Optional controller experiment: release excess squeeze through
            # legal joint commands; never change measured forces or object state.
            if 'hold_force_ceiling_n' in self.cfg:
                measured=dict(zip(self.env.names,observation['joint_positions_rad']))
                forces={finger:0. for finger in self.finger_groups}
                for path, force in zip(observation['contact_body_paths'], observation['contact_forces_n']):
                    finger=self.body_fingers.get(path.rsplit('/',1)[-1])
                    if finger is not None: forces[finger]+=float(np.linalg.norm(force))
                if self.grip_feedback is None: self.grip_feedback=dict(self.frozen_fingers)
                for finger,joints in self.finger_groups.items():
                    error=forces[finger]-float(self.cfg['hold_force_ceiling_n'])
                    if error>0:
                        step=min(.10*dt, float(self.cfg.get('force_release_gain',.005))*error*dt)
                        for n in joints:
                            if n.endswith('_1'): continue
                            lo,hi=self.env.limits[n]
                            self.grip_feedback[n]=float(np.clip(self.grip_feedback[n]-step,
                                max(lo,measured[n]-.01),hi))
                target.update(self.grip_feedback)
        if elapsed<6:
            for n in self.env.names:
                if n.startswith(self.prefix):
                    final=(-1 if self.hand=='left' else 1)*self.cfg['thumb_opposition_rad'] if 'joint1_1' in n else self.initial_q[n]
                    target[n]=self.initial_q[n]+min(elapsed/3.,1.)*(final-self.initial_q[n])
        target.update(self.solve_arm(goal_hand,observation));self.target_frame=goal_hand.copy()
        for j in self.env.model.revolute:
            n=j.get('name');limit=float(j.find('limit').get('velocity'))*.5*dt
            lo,hi=self.env.limits[n]
            target[n]=float(np.clip(self.q[n]+np.clip(target[n]-self.q[n],-limit,limit),lo,hi))
        self.q=target
        return target
