"""Multi-body Isaac adapter; instantiate only after SimulationApp starts.

Live physics is REQUIRED to validate this adapter. No kinematic object motion
is used during an episode. Launch velocities are explicit reset conditions.
"""
import json
import numpy as np
from .contract import ROOT, load_scenario
from .evaluator import ScenarioEvaluator
from robot_model import Robot


class SharpaScenarioEnv:
    def __init__(self, task_id, app, render=False):
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from pxr import UsdGeom, UsdPhysics
        self.spec = load_scenario(task_id); self.cfg = self.spec['runtime']
        self.app, self.render = app, render
        self.has_reset = self.launched = False
        self.capture_frame_id = None; self.capture_valid = False; self.capture_count = 0
        self.capture_sensor = None
        model = Robot(ROOT / 'assets/dual_arm/robot.urdf')
        self.limits = {j.get('name'): (float(j.find('limit').get('lower')),
                                      float(j.find('limit').get('upper'))) for j in model.revolute}
        self.contacts = {}
        for hand in ('left', 'right'):
            links = []
            def visit(link):
                links.append(link)
                for joint in model.children.get(link, []):
                    visit(joint.find('child').get('link'))
            visit(hand + '_hand_base')
            self.contacts[hand] = ['/World/Robot/DualArm/Links/' + x
                                   for x in sorted({model.groups[n] for n in links})]
        self.contact_paths = self.contacts['left'] + self.contacts['right']
        path = ROOT / 'sharpa/scenes' / task_id / 'scene.usda'
        omni.usd.get_context().open_stage(str(path))
        for _ in range(10):
            app.update()
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError('Scenario did not open')
        for path in self.contact_paths:
            if not self.stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f'Missing robot hand body: {path}')
        self.cameras = json.loads((ROOT / 'cameras.json').read_text())
        for cfg in self.cameras.values():
            if not self.stage.GetPrimAtPath(cfg['prim_path']).IsA(UsdGeom.Camera):
                raise RuntimeError('Missing workcell camera')
        self.world = World(stage_units_in_meters=1, physics_dt=self.cfg['physics_dt_s'],
                           rendering_dt=self.cfg['physics_dt_s'] * self.cfg['control_decimation'])
        self.robot = self.world.scene.add(SingleArticulation('/World/Robot/DualArm', name='robot'))
        self.bodies = {}; self.filters = {}
        for b in self.spec['bodies']:
            if not b['dynamic']:
                continue
            name = b['id']; path = '/World/Task/' + name
            if not self.stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f'Object lacks rigid body API: {name}')
            filters = list(self.contact_paths)
            if self.spec['task_type'] == 'return_ball' and name == self.spec['ball']:
                filters.append('/World/Task/' + self.spec['racket'])
            self.filters[name] = filters
            self.bodies[name] = self.world.scene.add(SingleRigidPrim(
                path, name=name, contact_filter_prim_paths=filters, track_contact_forces=True))
        self.world.reset()
        self.names = list(self.robot.dof_names)
        if set(self.names) != set(self.limits) or len(self.names) != 54:
            raise RuntimeError('Robot does not match existing 54-DOF model')
        ready = json.loads((ROOT / 'poses.json').read_text())['robots']['DualArm']['joint_positions']
        self.q0 = np.array([ready[n] for n in self.names]); self.target = self.q0.copy()
        self.evaluator = ScenarioEvaluator(self.spec)
        if 'capture_camera' in self.spec:
            from isaacsim.sensors.camera import Camera
            cfg = self.spec['capture_camera']
            self.capture_sensor = Camera('/World/Task/' + cfg['body'] + '/CaptureCamera',
                                         resolution=tuple(cfg['resolution']))
            self.capture_sensor.initialize()

    def _apply(self):
        from isaacsim.core.utils.types import ArticulationAction
        self.robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=self.target))

    def _place(self, b):
        obj = self.bodies[b['id']]
        obj.set_world_pose(position=np.array(b['position']), orientation=np.array(b['orientation_wxyz']))
        obj.set_linear_velocity(np.zeros(3)); obj.set_angular_velocity(np.zeros(3))

    def reset(self):
        self.world.reset(); self.robot.set_joint_positions(self.q0.copy())
        self.robot.set_joint_velocities(np.zeros_like(self.q0)); self.target = self.q0.copy(); self._apply()
        dynamic = [b for b in self.spec['bodies'] if b['dynamic']]
        for b in dynamic:
            self._place(b)
        for _ in range(self.cfg['settle_steps']):
            # Keep launch bodies at reset poses ONLY during settling.
            for b in dynamic:
                if 'launch_velocity_m_s' in b:
                    self._place(b)
            self.world.step(render=self.render)
        for b in dynamic:
            if 'launch_velocity_m_s' in b:
                self._place(b)
        self.launched = False; self.has_reset = True; self.evaluator.reset()
        self.capture_frame_id = None; self.capture_valid = False; self.capture_count = 0
        return self.observe()

    def observe(self):
        bodies = {}; left_count = len(self.contacts['left'])
        pair_force = np.zeros(3)
        for name, obj in self.bodies.items():
            position, orientation = obj.get_world_pose()
            raw = obj.get_contact_force_matrix(dt=self.cfg['physics_dt_s'])
            if raw is None:
                raise RuntimeError('Contact sensor returned no data')
            f = np.asarray(raw, dtype=float)
            if f.size != len(self.filters[name]) * 3 or not np.isfinite(f).all():
                raise RuntimeError(f'Unexpected/non-finite contact matrix for {name}: {f.shape}')
            f = f.reshape(-1, 3)
            bodies[name] = dict(position=np.asarray(position).copy(), orientation_wxyz=np.asarray(orientation).copy(),
                linear_velocity_m_s=np.asarray(obj.get_linear_velocity()).copy(),
                angular_velocity_rad_s=np.asarray(obj.get_angular_velocity()).copy(),
                contacts_n=dict(left=f[:left_count].copy(), right=f[left_count:len(self.contact_paths)].copy()))
            if len(self.filters[name]) > len(self.contact_paths):
                pair_force = f[-1].copy()
        return dict(bodies=bodies, joint_names=list(self.names),
                    joint_positions_rad=np.asarray(self.robot.get_joint_positions()).copy(),
                    joint_velocities_rad_s=np.asarray(self.robot.get_joint_velocities()).copy(),
                    ball_racket_contact_n=pair_force, capture_frame_id=self.capture_frame_id,
                    capture_valid=self.capture_valid)

    def step(self, joint_targets=None, capture=False):
        if not self.has_reset or self.evaluator.done:
            raise RuntimeError('Reset before stepping an uninitialized or finished episode')
        if joint_targets is not None:
            if not isinstance(joint_targets, dict):
                raise ValueError('Action must map joint names to absolute radians')
            target = self.target.copy()
            for name, value in joint_targets.items():
                if (name not in self.limits or isinstance(value, bool)
                    or not isinstance(value, (int, float)) or not np.isfinite(value)
                    or not self.limits[name][0] <= value <= self.limits[name][1]):
                    raise ValueError(f'Invalid joint target: {name}')
                target[self.names.index(name)] = value
            self.target = target
        if not isinstance(capture, bool) or (capture and self.capture_sensor is None):
            raise ValueError('capture requires a photography scenario and a boolean action')
        self._apply()
        if not self.launched:
            for b in self.spec['bodies']:
                if 'launch_velocity_m_s' in b:
                    self.bodies[b['id']].set_linear_velocity(np.array(b['launch_velocity_m_s']))
            self.launched = True
        frame = None
        for i in range(self.cfg['control_decimation']):
            # Photography renders each step; a missing RGB sample is an error.
            self.world.step(render=self.render or self.capture_sensor is not None)
            if capture and i == self.cfg['control_decimation'] - 1:
                rgba = np.asarray(self.capture_sensor.get_rgba())
                w, h = self.spec['capture_camera']['resolution']
                if rgba.shape != (h, w, 4) or not np.isfinite(rgba).all() or not np.any(rgba[:, :, :3]):
                    raise RuntimeError('Capture camera did not produce a nonempty RGBA frame')
                frame = rgba[:, :, :3].copy(); self.capture_count += 1
                self.capture_frame_id = self.capture_count; self.capture_valid = True
            obs = self.observe()
            result = self.evaluator.update(obs, self.cfg['physics_dt_s'])
            if result['terminated'] or result['truncated']:
                break
        if frame is not None:
            obs['captured_rgb'] = frame
        return obs, result

    def close(self):
        self.world.stop(); self.world.clear_instance()
