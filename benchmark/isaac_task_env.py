"""Single-task Isaac adapter. Construct only after SimulationApp initialization.

The application and stage lifecycle belongs to the caller. This adapter supplies
measured state and joint targets; it contains no automatic grasp policy.
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from robot_model import Robot
from task_runtime import ROOT, TaskEvaluator, load_task, runtime_config


class IsaacTaskEnv:
    def __init__(self, task_id, app, render=False):
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from pxr import UsdGeom, UsdPhysics

        self.task = load_task(task_id)
        self.config = runtime_config(self.task)
        self.app, self.render = app, render
        self.model = Robot(ROOT / 'assets/dual_arm/robot.urdf')
        cfg = json.loads((ROOT / 'poses.json').read_text())['robots']['DualArm']
        self.ready = cfg['joint_positions']
        self.limits = {j.get('name'): (float(j.find('limit').get('lower')),
                                      float(j.find('limit').get('upper')))
                       for j in self.model.revolute}
        hand_links = []

        def visit(link):
            hand_links.append(link)
            for joint in self.model.children.get(link, []):
                visit(joint.find('child').get('link'))

        visit(self.task['preferred_hand'] + '_hand_base')
        bodies = sorted({self.model.groups[link] for link in hand_links})
        self.contact_paths = ['/World/Robot/DualArm/Links/' + body for body in bodies]
        path = ROOT / 'tasks' / task_id / 'scene.usda'
        omni.usd.get_context().open_stage(str(path))
        for _ in range(10):
            app.update()
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError('Task stage did not open')
        for body in self.contact_paths:
            if not self.stage.GetPrimAtPath(body).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f'Contact body is missing or not rigid: {body}')
        cameras = json.loads((ROOT / 'cameras.json').read_text())
        for camera in cameras.values():
            if not self.stage.GetPrimAtPath(camera['prim_path']).IsA(UsdGeom.Camera):
                raise RuntimeError('A required camera is missing')
        self.cameras = cameras
        self.world = World(stage_units_in_meters=1, physics_dt=self.config['physics_dt_s'],
                           rendering_dt=self.config['physics_dt_s'] * self.config['control_decimation'])
        self.robot = self.world.scene.add(SingleArticulation('/World/Robot/DualArm', name='robot'))
        self.obj = self.world.scene.add(SingleRigidPrim(
            self.task['object_prim'], name='task_object',
            contact_filter_prim_paths=self.contact_paths, track_contact_forces=True))
        self.evaluator = TaskEvaluator(self.task)
        self.world.reset()
        self.names = list(self.robot.dof_names)
        if set(self.names) != set(self.limits) or len(self.names) != 54:
            raise RuntimeError('Articulation joint names differ from the authored 54-DOF model')
        self.q0 = np.array([self.ready[name] for name in self.names])
        self.target = self.q0.copy()
        self.has_reset = False

    def _apply_target(self):
        from isaacsim.core.utils.types import ArticulationAction
        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=self.target))

    def reset(self):
        self.world.reset()
        self.robot.set_joint_positions(self.q0.copy())
        self.robot.set_joint_velocities(np.zeros_like(self.q0))
        self.target = self.q0.copy()
        self._apply_target()
        pose = np.asarray(self.task['initial_pose_world'], dtype=float)
        quat_xyzw = Rotation.from_matrix(pose[:3, :3]).as_quat()
        self.obj.set_world_pose(position=pose[:3, 3], orientation=quat_xyzw[[3, 0, 1, 2]])
        self.obj.set_linear_velocity(np.zeros(3))
        self.obj.set_angular_velocity(np.zeros(3))
        for _ in range(self.config['settle_steps']):
            self.world.step(render=self.render)
        observation = self.observe()
        self.evaluator.reset(observation['object_pose_world'])
        self.has_reset = True
        return observation

    def observe(self):
        position, quat_wxyz = self.obj.get_world_pose()
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(np.asarray(quat_wxyz)[[1, 2, 3, 0]]).as_matrix()
        pose[:3, 3] = position
        raw = self.obj.get_contact_force_matrix(dt=self.config['physics_dt_s'])
        if raw is None:
            raise RuntimeError('Contact-force API returned no data; refusing inferred grasp success')
        forces = np.asarray(raw, dtype=float)
        if forces.size != len(self.contact_paths) * 3:
            raise RuntimeError(f'Unexpected contact matrix shape: {forces.shape}')
        forces = forces.reshape(len(self.contact_paths), 3)
        if not np.isfinite(forces).all():
            raise RuntimeError('Non-finite measured contact forces')
        return {
            'joint_names': list(self.names),
            'joint_positions_rad': np.asarray(self.robot.get_joint_positions()).copy(),
            'joint_velocities_rad_s': np.asarray(self.robot.get_joint_velocities()).copy(),
            'object_pose_world': pose,
            'object_linear_velocity_m_s': np.asarray(self.obj.get_linear_velocity()).copy(),
            'object_angular_velocity_rad_s': np.asarray(self.obj.get_angular_velocity()).copy(),
            'contact_body_paths': list(self.contact_paths),
            'contact_forces_n': forces.copy(),
            'goal_pose_world': np.asarray(self.task['goal_pose_world']).copy(),
        }

    def step(self, joint_targets=None, on_physics_step=None):
        if not self.has_reset:
            raise RuntimeError('Call reset before step')
        if self.evaluator.done:
            raise RuntimeError('Episode is finished; call reset before step')
        if joint_targets is not None:
            if not isinstance(joint_targets, dict):
                raise ValueError('Action must map joint names to absolute target radians')
            target = self.target.copy()
            for name, value in joint_targets.items():
                if name not in self.limits:
                    raise ValueError(f'Unknown joint: {name}')
                lo, hi = self.limits[name]
                if not np.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(f'Joint target exceeds limits: {name}')
                target[self.names.index(name)] = value
            self.target = target
        self._apply_target()
        # Evaluate every physics step so brief contact loss breaks the hold timer.
        for _ in range(self.config['control_decimation']):
            self.world.step(render=self.render)
            observation = self.observe()
            result = self.evaluator.update(
                observation['object_pose_world'], observation['object_linear_velocity_m_s'],
                observation['object_angular_velocity_rad_s'], observation['contact_forces_n'],
                self.config['physics_dt_s'])
            if on_physics_step is not None:
                on_physics_step(observation, result, self.config['physics_dt_s'])
            if result['terminated'] or result['truncated']:
                break
        return observation, result

    def close(self):
        self.world.stop()
        self.world.clear_instance()
