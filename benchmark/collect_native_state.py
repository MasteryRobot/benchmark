"""GPU PhysX diagnostic collection. Never certifies functional/RGBD success.

Loads the original USD workcell through an isolated authoring subprocess.
Only joint position targets are written after attachment. All saved rigid body,
joint and contact values are read from PhysX. No pose writes or attachments.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
from scipy.spatial.transform import Rotation
from robot_model import Robot
from task_runtime import ROOT, load_task, runtime_config, TaskEvaluator


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def author(task_id, output, physics_config=None):
    import physx_usd_schemas
    from physx_usd_schemas import PhysxSchema
    from pxr import Usd, UsdPhysics, Sdf
    source = ROOT / 'tasks' / task_id / 'scene.usda'
    if not source.exists():
        source = ROOT / 'sharpa/scenes' / task_id / 'scene.usda'
    stage = Usd.Stage.CreateNew(str(output / 'scene.usda'))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    tuning = json.loads(physics_config.read_text()) if physics_config else {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
            scene.CreateEnableGPUDynamicsAttr(True)
            scene.CreateBroadphaseTypeAttr('GPU')
            scene.CreateSolverTypeAttr('TGS')
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName().startswith(('l_f_', 'r_f_')):
            drive=UsdPhysics.DriveAPI(prim,'angular')
            if 'finger_stiffness_nm_rad' in tuning:
                drive.GetStiffnessAttr().Set(float(tuning['finger_stiffness_nm_rad'])*np.pi/180)
            if 'finger_damping_nm_s_rad' in tuning:
                drive.GetDampingAttr().Set(float(tuning['finger_damping_nm_s_rad'])*np.pi/180)
            if 'finger_max_force_nm' in tuning:
                drive.GetMaxForceAttr().Set(float(tuning['finger_max_force_nm']))
        if prim.IsA(UsdPhysics.Scene) and 'solver' in tuning:
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateSolverTypeAttr(tuning['solver'])
        if prim.IsA(UsdPhysics.Scene):
            for name in ('solveArticulationContactLast','enableExternalForcesEveryIteration'):
                if name in tuning:
                    prim.CreateAttribute('physxScene:'+name,Sdf.ValueTypeNames.Bool).Set(bool(tuning[name]))
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            for key,attr in [('articulation_position_iterations','solverPositionIterationCount'),
                             ('articulation_velocity_iterations','solverVelocityIterationCount')]:
                if key in tuning:
                    prim.CreateAttribute('physxArticulation:'+attr,Sdf.ValueTypeNames.Int).Set(int(tuning[key]))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()).startswith('/World/Task/'):
            for key,attr in [('object_position_iterations','solverPositionIterationCount'),
                             ('object_velocity_iterations','solverVelocityIterationCount')]:
                if key in tuning:
                    prim.CreateAttribute('physxRigidBody:'+attr,Sdf.ValueTypeNames.Int).Set(int(tuning[key]))
    stage.GetRootLayer().Save()
    dump(output / 'authoring.json', {'source': str(source),
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'overrides': ['GPU dynamics', 'GPU broadphase', 'TGS', 'contact reporting threshold=0'],
        'physics_geometry_changed': False, 'explicit_physics_tuning':tuning})


def hand_paths(model):
    result = {}
    for hand in ('left', 'right'):
        links = []
        def visit(link):
            links.append(link)
            for joint in model.children.get(link, []):
                visit(joint.find('child').get('link'))
        visit(hand + '_hand_base')
        result[hand] = ['/World/Robot/DualArm/Links/' + x
                        for x in sorted({model.groups[n] for n in links})]
    return result


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source_names = ['collect_native_state.py','native_cuda_buffer.py','robot_model.py',
                    'representative_policy.py','contact_grasp_probe.py','task_runtime.py',
                    'geometric_grasp_probe.py','poses.json','cameras.json','assets/dual_arm/robot.urdf']
    spec_file = ROOT/'tasks'/args.task/'task.json'
    if not spec_file.exists(): spec_file = ROOT/'sharpa/scenes'/args.task/'task.json'
    source_names.append(str(spec_file.relative_to(ROOT)))
    sources = output / 'sources'; sources.mkdir()
    source_hashes = {}
    for name in source_names:
        original = ROOT/name; contents = original.read_bytes()
        destination = sources/name; destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(contents)
        source_hashes[name] = hashlib.sha256(contents).hexdigest()
    report = {'task': args.task, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'STARTING', 'physics_started': False, 'task_success': False,
        'functional_success': False, 'training_eligible': False,
        'backend': 'Standalone NVIDIA PhysX GPU', 'rgbd_collected': False,
        'mode': args.policy, 'object_pose_writes_during_episode': 0,
        'note': 'State/contact diagnostics; no functional policy or RGBD certification.',
        'sources_at_start_sha256': source_hashes, 'source_snapshot': 'sources'}
    dump(output / 'report.json', report)
    replay = None
    if args.replay_actions:
        payload=args.replay_actions.read_bytes()
        (output/'input_actions.npz').write_bytes(payload)
        source_report=json.loads((args.replay_actions.parent/'report.json').read_text())
        replay=np.load(output/'input_actions.npz',allow_pickle=False)
        report['action_replay']={'source':str(args.replay_actions.resolve()),
            'sha256':hashlib.sha256(payload).hexdigest(),
            'relax_fraction':args.replay_relax_fraction,
            'relax_at_simulation_s':args.replay_relax_at,
            'scope':'Recorded joint targets only; object states are never replayed.'}
    physx = stage = None
    bindings = []; contact = None; rows = []; detailed = []; cuda_buffers = []
    try:
        author_command=[sys.executable, '-B', __file__, args.task, '--author-only',
                        '--output', str(output)]
        if args.physics_config:
            tuning=json.loads(args.physics_config.read_text());dump(output/'physics_config.json',tuning)
            author_command.extend(['--physics-config',str(args.physics_config.resolve())])
            report['explicit_physics_tuning']=tuning
        subprocess.run(author_command, check=True)
        import ovstage, ovphysx
        from native_cuda_buffer import CudaBuffer
        from ovphysx import PhysX, PhysXConfig
        from ovphysx.types import TensorType
        from ovphysx.samples.python_samples.schema_support import physx_schema_paths
        model = Robot(ROOT / 'assets/dual_arm/robot.urdf')
        contacts = hand_paths(model)
        filters = contacts['left'] + contacts['right']
        baseline = (ROOT / 'tasks' / args.task / 'task.json').exists()
        if baseline:
            spec = load_task(args.task); cfg = runtime_config(spec)
            objects = [spec['object_prim']]
        else:
            from sharpa.contract import load_scenario
            spec = load_scenario(args.task); cfg = spec['runtime']
            objects = ['/World/Task/' + b['id'] for b in spec['bodies'] if b['dynamic']]
        dump(output/'task_contract.json',spec)
        # Refine integration without changing episode timing or task thresholds.
        original_cfg = dict(cfg)
        cfg = dict(cfg)
        cfg['physics_dt_s'] /= args.physics_substeps
        cfg['settle_steps'] *= args.physics_substeps
        cfg['control_decimation'] *= args.physics_substeps
        report['integration'] = {'substeps': args.physics_substeps,
            'original_runtime': original_cfg, 'effective_runtime': cfg}
        report['transition_convention']='Row stores state after applied targets for this physics step; new policy targets apply to the next step.'
        report['requested_gpu']=args.gpu
        ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
        stage = ovstage.Stage('benchmark-state-collection')
        ovstage.population.open_usd(stage, str(output / 'scene.usda'), ordinal=1,
                                   domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physx = PhysX(active_cuda_gpus=str(args.gpu), config=PhysXConfig(
            carbonite_overrides={'/physics/suppressReadback': True}))
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        def bind(pattern, kind):
            b = physx.create_tensor_binding(pattern=pattern, tensor_type=kind)
            bindings.append(b)
            if b.count == 0:
                raise RuntimeError('Empty binding: ' + str(kind))
            return b
        robot = '/World/Robot/DualArm/Links/*'
        joint = bind(robot, TensorType.ARTICULATION_DOF_POSITION)
        velocity = bind(robot, TensorType.ARTICULATION_DOF_VELOCITY)
        control = bind(robot, TensorType.ARTICULATION_DOF_POSITION_TARGET)
        links = bind(robot, TensorType.ARTICULATION_LINK_POSE)
        body_bindings = [(bind(p, TensorType.RIGID_BODY_POSE),
                          bind(p, TensorType.RIGID_BODY_VELOCITY)) for p in objects]
        if joint.native_device.device_type.value != 2:
            raise RuntimeError('Expected CUDA native tensors; CPU fallback prohibited')
        names = joint.dof_names
        if replay is not None and (source_report['joint_names']!=names or
                not np.isclose(source_report['physics_dt_s'],cfg['physics_dt_s'])):
            raise ValueError('Replay requires identical joint order and timestep')
        ready = json.loads((ROOT / 'poses.json').read_text())['robots']['DualArm']['joint_positions']
        if set(names) != set(ready) or joint.shape != (1, 54):
            raise RuntimeError('54 DOF name mapping mismatch')
        contact = physx.create_contact_binding(sensor_patterns=objects,
            filter_patterns=filters * len(objects), filters_per_sensor=len(filters),
            max_contact_data_count=8192)
        report.update(version=ovphysx.__version__, device=str(joint.native_device),
            joint_names=names, body_names=links.body_names, object_paths=objects,
            contact_sensor_paths=contact.sensor_paths, contact_filter_paths=contact.filter_paths,
            physics_dt_s=cfg['physics_dt_s'], actual_capture_hz=1/cfg['physics_dt_s'])
        if contact.sensor_paths != objects:
            raise RuntimeError('Contact sensor ordering differs from object state ordering')
        resolved_filters = contact.filter_paths
        selected = [i for i,p in enumerate(resolved_filters[0])
                    if p in contacts[spec.get('preferred_hand', 'left')]]
        q = np.zeros(joint.shape, np.float32); qd = np.zeros_like(q)
        lp = np.zeros(links.shape, np.float32)
        target = np.asarray([[ready[n] for n in names]], np.float32)
        normal = np.zeros((contact.sensor_count, contact.filter_count, 3), np.float32)
        friction = np.zeros_like(normal)
        capacity = 8192
        cf = np.zeros((capacity, 1), np.float32); cp = np.zeros((capacity, 3), np.float32)
        cn = np.zeros_like(cp); sep = np.zeros_like(cf)
        counts = np.zeros(normal.shape[:2], np.int32); starts = np.zeros_like(counts)
        def cuda_array(array):
            buffer = CudaBuffer(array.shape, array.dtype, args.gpu)
            cuda_buffers.append(buffer)
            return buffer
        gpu_normal = cuda_array(normal); gpu_friction = cuda_array(friction)
        gpu_detail = [cuda_array(a) for a in (cf, cp, cn, sep, counts, starts)]
        arrays = [(np.zeros(a.shape, np.float32), np.zeros(b.shape, np.float32))
                  for a, b in body_bindings]
        limits = {j.get('name'): (float(j.find('limit').get('lower')),
                  float(j.find('limit').get('upper'))) for j in model.revolute}
        def observation():
            joint.read(q); velocity.read(qd); links.read(lp)
            contact.read_normal_force_matrix(gpu_normal.tensor)
            contact.read_friction_force_matrix(gpu_friction.tensor)
            normal[:] = gpu_normal.numpy(); friction[:] = gpu_friction.numpy()
            poses, speeds = [], []
            for (pb, vb), (pa, va) in zip(body_bindings, arrays):
                pb.read(pa); vb.read(va); poses.append(pa[0].copy()); speeds.append(va[0].copy())
            all_arrays = [q, qd, lp, normal, friction, np.asarray(poses), np.asarray(speeds)]
            if not all(np.isfinite(x).all() for x in all_arrays):
                raise RuntimeError('Nonfinite measured state')
            return np.asarray(poses), np.asarray(speeds)
        dt = cfg['physics_dt_s']; settle = cfg['settle_steps']
        policy = evaluator = None
        physx.warmup(); physx.wait_all(); report['physics_started'] = True
        start = time.monotonic()
        relaxed_target=None;relax_start_target=None
        for step in range(settle + round(args.duration / dt)):
            if replay is not None:
                target=np.asarray(replay['targets'][min(step,len(replay['targets'])-1)],np.float32)[None].copy()
                if args.replay_relax_fraction and step*dt>=args.replay_relax_at:
                    if relaxed_target is None:
                        relax_start_target=target.copy();relaxed_target=target.copy()
                        prefix='l_f_' if spec['preferred_hand']=='left' else 'r_f_'
                        for index,name in enumerate(names):
                            if name.startswith(prefix):
                                relaxed_target[0,index]=(1-args.replay_relax_fraction)*target[0,index]+args.replay_relax_fraction*q[0,index]
                    blend=min(step*dt-args.replay_relax_at,1.)
                    target=(1-blend)*relax_start_target+blend*relaxed_target
            applied_target = target.copy()
            control.write(target); physx.step_sync(dt)
            poses, speeds = observation()
            phase = 'settle' if step < settle else policy.phase if policy else 'hold'
            if replay is not None: phase='joint_action_replay' if relaxed_target is None else 'late_grip_relaxation'
            force = normal + friction
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(poses[0, 3:]).as_matrix()
            pose[:3, 3] = poses[0, :3]
            obs = {'joint_positions_rad': q[0].copy(), 'object_pose_world': pose,
                   'contact_forces_n': force[0, selected].copy(),
                   'contact_body_paths': [resolved_filters[0][i] for i in selected],
                   'object_velocity_world': speeds[0].copy()}
            if step == settle - 1:
                report['settled_object_poses_xyzw'] = poses.tolist()
                report['settled_max_joint_error_rad'] = float(np.max(np.abs(q-target)))
                if baseline:
                    evaluator = TaskEvaluator(spec); evaluator.reset(pose)
                if args.policy in ('baseline_candidate', 'contact_grasp_probe','geometric_grasp_probe'):
                    if not baseline:
                        raise ValueError('Baseline candidate requires an original baseline task')
                    from types import SimpleNamespace
                    from representative_policy import CandidatePolicy
                    from contact_grasp_probe import ContactGraspProbe
                    from geometric_grasp_probe import GeometricGraspProbe
                    env = SimpleNamespace(task=spec, config=cfg, names=names, q0=target[0].copy(),
                                          model=model, limits=limits)
                    if args.policy=='geometric_grasp_probe':
                        config=json.loads(args.policy_config.read_text()) if args.policy_config else {}
                        policy=GeometricGraspProbe(env,variant=args.variant,config=config)
                    else:
                        policy_class = ContactGraspProbe if args.policy == 'contact_grasp_probe' else CandidatePolicy
                        policy = policy_class(env, variant=args.variant)
                    report['policy_info']=policy.info
            result = None
            if step >= settle:
                if evaluator:
                    result = evaluator.update(pose, speeds[0,:3], speeds[0,3:],
                                               obs['contact_forces_n'], dt)
                if policy and (step-settle) % cfg['control_decimation'] == 0:
                    values = policy.action(obs, (step-settle)*dt)
                    target = np.array([[values[n] for n in names]], np.float32)
                    phase = policy.phase
            required = contact.read_normal_contact_data(*(b.tensor for b in gpu_detail))
            for dest, src in zip((cf, cp, cn, sep, counts, starts), gpu_detail):
                dest[:] = src.numpy()
            if required > capacity:
                raise RuntimeError('Contact buffer overflow; trace rejected')
            record = {'time_s': (step+1)*dt, 'phase': phase, 'joints': q[0].copy(),
                      'joint_velocities': qd[0].copy(), 'targets': applied_target[0].copy(),
                      'links': lp[0].copy(), 'objects': poses, 'object_velocities': speeds,
                      'normal_forces': normal.copy(), 'friction_forces': friction.copy()}
            rows.append(record)
            points = []
            for si in range(counts.shape[0]):
                for fi in range(counts.shape[1]):
                    begin = int(starts[si,fi]); end = begin + int(counts[si,fi])
                    for ci in range(begin, end):
                        points.append([si,fi,*cp[ci].tolist(),*cn[ci].tolist(),
                                       float(cf[ci,0]),float(sep[ci,0])])
            detailed.append({'time_s': (step+1)*dt, 'contacts': points, 'evaluation': result})
            if step % 240 == 0:
                report.update(status='RUNNING',recorded_physics_frames=len(rows),
                              current_simulation_time_s=(step+1)*dt,phase=phase)
                dump(output / 'report.json', report)
                print(json.dumps({'task':args.task,'time_s':round((step+1)*dt,3),
                    'phase':phase,'max_hand_force_n':float(np.linalg.norm(force,axis=-1).max()),
                    'object_z':poses[:,2].tolist()}), flush=True)
            if result and (result['terminated'] or result['truncated']):
                report['baseline_predicate_success'] = result['success']
                report['final_evaluation'] = result
                break
            if time.monotonic() - start > args.wall_timeout:
                raise TimeoutError('Collection exceeded wall timeout')
        report['status'] = 'DIAGNOSTIC_RECORDED'
    except Exception as exc:
        report.update(status='PHYSICS_FAILED' if report['physics_started'] else 'STARTUP_ERROR',
                      error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if rows:
            np.savez_compressed(output / 'measured_states.npz',
                **{k:np.asarray([r[k] for r in rows]) for k in rows[0] if k != 'phase'},
                phase=np.array([r['phase'] for r in rows]))
            with (output / 'contacts.jsonl').open('w') as f:
                for row in detailed:
                    f.write(json.dumps(row) + '\n')
        if contact is not None: contact.destroy()
        for buffer in reversed(cuda_buffers): buffer.close()
        for b in reversed(bindings): b.destroy()
        if physx is not None:
            if stage is not None and physx.is_stage_attached(stage):
                physx.wait_task(physx.detach_ovstage())
            physx.destroy()
        if stage is not None: stage.destroy()
        report['recorded_physics_frames'] = len(rows)
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        report['source_sha256'] = source_hashes['collect_native_state.py']
        report['artifacts_sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in output.iterdir() if p.is_file() and p.name != 'report.json'}
        dump(output / 'report.json', report)
        print(json.dumps({k:report.get(k) for k in ['task','status','physics_started',
            'recorded_physics_frames','error','baseline_predicate_success','functional_success']}),flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_RECORDED' else 2


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('task'); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--author-only', action='store_true'); p.add_argument('--gpu', type=int, default=0)
    p.add_argument('--duration', type=float, default=20.)
    p.add_argument('--wall-timeout', type=float, default=600.)
    p.add_argument('--policy', choices=['hold','baseline_candidate','contact_grasp_probe','geometric_grasp_probe'], default='hold')
    p.add_argument('--policy-config',type=Path)
    p.add_argument('--physics-config',type=Path)
    p.add_argument('--variant', type=int, default=0)
    p.add_argument('--physics-substeps', type=int, choices=[1,2,4], default=1)
    p.add_argument('--replay-actions',type=Path)
    p.add_argument('--replay-relax-fraction',type=float,default=0.)
    p.add_argument('--replay-relax-at',type=float,default=20.)
    args = p.parse_args()
    if args.replay_actions and args.policy!='hold':p.error('Replay requires --policy hold')
    if not 0<=args.replay_relax_fraction<=1:p.error('Relax fraction must be within [0,1]')
    if args.author_only: author(args.task, args.output, args.physics_config)
    else: sys.exit(run(args))
