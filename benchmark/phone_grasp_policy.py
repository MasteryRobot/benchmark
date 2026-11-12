"""Experimental phone side-pinch controller; commands joints only."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from robot_model import origin
from representative_policy import CandidatePolicy


class PhoneGraspPolicy(CandidatePolicy):
    def __init__(self, env, parameters):
        super().__init__(env, 0)
        self.params = dict(parameters)
        self.origins = []
        self.axes = []
        for name in self.arm_names:
            joint = next(j for j in env.model.revolute if j.get('name') == name)
            self.origins.append(origin(joint.find('origin')))
            self.axes.append(np.fromstring(joint.find('axis').get('xyz'), sep=' '))
        frames = env.model.fk(self.initial_q, self.base)
        first = next(j for j in env.model.revolute if j.get('name') == self.arm_names[0])
        self.arm_base = frames[first.find('parent').get('link')]
        self.tool = np.linalg.inv(frames['Link7_' + self.chain]) @ frames[self.tip]
        self.bounds = np.array([env.limits[n] for n in self.arm_names]).T
        self.speed = {j.get('name'):float(j.find('limit').get('velocity')) for j in env.model.revolute}
        down = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
        self.rotation = Rotation.from_euler('z', parameters.get('yaw_deg', -108.), degrees=True).as_matrix() @ down
        self.rotation = self.rotation @ Rotation.from_euler('y', parameters.get('tilt_deg',0.), degrees=True).as_matrix()
        self.offset = np.array(parameters.get('offset', [.045, -.015, .145]))
        self.start_hand = frames[self.tip]
        self.initial_object = None
        self.hold_object = None
        self.hold_hand = None
        self.contact_duration = 0.
        self.triggered_at = None
        self.frozen_fingers = None
        self.relaxed_fingers = None
        self.relax_start = None
        self.info.update(name='phone_side_pinch_candidate', parameters=self.params, source_file=__file__)

    def arm_fk(self, q):
        frame = self.arm_base.copy()
        for O, a, value in zip(self.origins, self.axes, q):
            R = np.eye(4); R[:3,:3] = Rotation.from_rotvec(a*value).as_matrix()
            frame = frame @ O @ R
        return frame @ self.tool

    def solve_arm(self, target, observation):
        measured = dict(zip(self.env.names, observation['joint_positions_rad']))
        seed = np.array([measured[n] for n in self.arm_names])
        def residual(q):
            frame = self.arm_fk(q)
            return np.r_[frame[:3,3]-target[:3,3], .18*Rotation.from_matrix(target[:3,:3]@frame[:3,:3].T).as_rotvec(), .0005*(q-seed)]
        fit = least_squares(residual, np.clip(seed,*self.bounds), bounds=self.bounds, max_nfev=25)
        return dict(zip(self.arm_names, fit.x))

    def action(self, obs, elapsed):
        if self.initial_object is None:
            self.initial_object = obs['object_pose_world'].copy()
        target = dict(self.q)
        grasp = np.eye(4); grasp[:3,:3] = self.rotation
        grasp[:3,3] = self.initial_object[:3,3] - self.rotation @ self.offset
        approach_end = 3.5; descend_end = 6.0; close_end = 9.0
        if self.params.get('contact_trigger') and elapsed >= descend_end and self.triggered_at is None:
            dt=self.env.config['physics_dt_s']*self.env.config['control_decimation']
            touching=np.count_nonzero(np.linalg.norm(obs['contact_forces_n'],axis=1)>=self.params.get('trigger_force',.5))
            self.contact_duration=self.contact_duration+dt if touching>=3 else 0.
            if self.contact_duration>=.15:
                self.triggered_at=elapsed
                measured=dict(zip(self.env.names,obs['joint_positions_rad']))
                self.frozen_fingers={}
                for name in self.env.names:
                    if not name.startswith(self.prefix):continue
                    preload=self.params.get('preload_rad',.06)
                    delta=-preload if name.endswith('joint1_1') else preload if not name.endswith('_1') else 0.
                    self.frozen_fingers[name]=float(np.clip(measured[name]+delta,*self.env.limits[name]))
        if self.triggered_at is not None:close_end=self.triggered_at
        if elapsed < approach_end:
            self.phase = 'approach'
            a = min(elapsed/approach_end,1.); a = a*a*(3-2*a)
            goal = grasp.copy(); goal[2,3] += .11
            hand = self.start_hand.copy()
            hand[:3,3] = (1-a)*hand[:3,3]+a*goal[:3,3]
            delta = Rotation.from_matrix(goal[:3,:3]@hand[:3,:3].T).as_rotvec()
            hand[:3,:3] = Rotation.from_rotvec(a*delta).as_matrix()@hand[:3,:3]
        elif elapsed < descend_end:
            self.phase = 'descend'; hand = grasp.copy()
            a = (elapsed-approach_end)/(descend_end-approach_end); a=a*a*(3-2*a)
            hand[2,3] += .11*(1-a)
        elif elapsed < close_end:
            self.phase = 'close'; hand = grasp.copy()
        else:
            self.phase = 'lift'
            if self.hold_object is None:
                self.hold_object = obs['object_pose_world'].copy()
                measured = dict(zip(self.env.names,obs['joint_positions_rad']))
                self.hold_hand = self.arm_fk([measured[n] for n in self.arm_names])
                self.grasp_relative = np.linalg.inv(self.hold_hand)@self.hold_object
            a = min((elapsed-close_end)/6.,1.); a=a*a*(3-2*a)
            goal = np.asarray(self.env.task['goal_pose_world'])
            object_target = self.hold_object.copy()
            object_target[:3,3] = (1-a)*self.hold_object[:3,3]+a*goal[:3,3]
            delta=Rotation.from_matrix(goal[:3,:3]@self.hold_object[:3,:3].T).as_rotvec()
            object_target[:3,:3]=Rotation.from_rotvec(a*delta).as_matrix()@self.hold_object[:3,:3]
            hand=object_target@np.linalg.inv(self.grasp_relative)
        target.update(self.solve_arm(hand,obs))
        closure = np.clip((elapsed-descend_end)/(close_end-descend_end),0.,1.)
        for name in self.env.names:
            if not name.startswith(self.prefix): continue
            lo,hi=self.env.limits[name]
            if name.endswith('_1'):
                value=((1-closure)*self.params.get('open_thumb_opposition',-.25)+closure*self.params.get('thumb_opposition',-.85)) if 'joint1_1' in name else 0.
                if 'jiont2_1' in name:value=self.params.get('index_spread',0.)
            else:
                frac=self.params.get('thumb_close',.32) if name.startswith(self.prefix+'joint1_') else self.params.get('finger_close',.48)
                value=(self.params.get('open_fraction',.0)*(1-closure)+frac*closure)*hi
                finger_type='thumb' if name.startswith(self.prefix+'joint1_') else 'finger'
                part=int(name[-1])-2
                if finger_type+'_joints' in self.params:
                    opened=self.params.get(finger_type+'_open_joints',[0.,0.,0.])[part]
                    closed=self.params[finger_type+'_joints'][part]
                    value=(1-closure)*opened+closure*closed
            target[name]=float(np.clip(value,lo,hi))
        if self.frozen_fingers is not None:target.update(self.frozen_fingers)
        if self.params.get('relax_fraction') and elapsed>=self.params.get('relax_at',15.5):
            if self.relaxed_fingers is None:
                measured=dict(zip(self.env.names,obs['joint_positions_rad']))
                fraction=self.params['relax_fraction']
                self.relax_start={name:target[name] for name in self.env.names if name.startswith(self.prefix)}
                self.relaxed_fingers={name:(1-fraction)*target[name]+fraction*measured[name] for name in self.relax_start}
            blend=min((elapsed-self.params.get('relax_at',15.5))/1.,1.)
            for name in self.relaxed_fingers:
                target[name]=(1-blend)*self.relax_start[name]+blend*self.relaxed_fingers[name]
        dt=self.env.config['physics_dt_s']*self.env.config['control_decimation']
        for name in target:
            delta=.8*self.speed[name]*dt
            target[name]=float(np.clip(self.q[name]+np.clip(target[name]-self.q[name],-delta,delta),*self.env.limits[name]))
        self.q=target
        return target
