"""Measured GPU PhysX cloth interaction; standalone backend, not Isaac Kit.

Preserves the benchmark USD assets and drives robot joints only. This probe
cannot certify functional manipulation or rolling without contact evaluation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import traceback
import numpy as np
from .config import FOLDER


def build(output, variant, fixed_wrist, tuning=None):
    import physx_usd_schemas
    from pxr import Usd, UsdGeom, UsdPhysics
    from physx_usd_schemas import PhysxSchema
    from .surface import author
    from .joint_plan import generate, validate
    config = json.loads((FOLDER / 'interaction_config.json').read_text())
    if variant != 'thick8':
        variants = json.loads((FOLDER / 'isaac_config.json').read_text())['variants']
        config['variant'] = next(v for v in variants if v['id'] == variant)
    config['surface_mesh_spacing_m'] = .012
    tuning = tuning or {}
    config['variant'].update(tuning.get('variant', {}))
    config.update(tuning.get('control', {}))
    plan = validate(generate(config))
    if fixed_wrist:
        reference = next(s['joint_targets_rad'] for s in reversed(plan['samples']) if s['phase'] == 'descend')
        for sample in plan['samples']:
            if sample['phase'] not in ('settle', 'approach', 'descend'):
                sample['joint_targets_rad'].update({n: q for n, q in reference.items() if n.startswith('Joint')})
        plan['scope'] = 'Fixed-arm finger contact probe after approach; not a rolling policy'
        validate(plan)
    path = output / 'scene.usda'
    stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = [str((FOLDER / 'scene_isaac.usda').resolve())]
    author(stage, config['variant'], config['surface_mesh_spacing_m'])
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            api.CreateEnableGPUDynamicsAttr(True)
            api.CreateBroadphaseTypeAttr('GPU')
            api.CreateSolverTypeAttr('TGS')
        if 'Attachment' in str(prim.GetTypeName()):
            raise RuntimeError('Cloth attachments are prohibited')
        if prim.IsA(UsdPhysics.RevoluteJoint) and not prim.GetName().startswith('Joint'):
            drive = UsdPhysics.DriveAPI(prim, 'angular')
            drive.GetMaxForceAttr().Set(tuning.get('finger_max_force_nm', .25))
            drive.GetStiffnessAttr().Set(tuning.get('finger_stiffness_nm_rad', 2.) * np.pi/180)
            drive.GetDampingAttr().Set(tuning.get('finger_damping_nm_s_rad', .1) * np.pi/180)
    stage.GetRootLayer().Save()
    (output / 'config.json').write_text(json.dumps(config, indent=2))
    (output / 'joint_plan.json').write_text(json.dumps(plan, indent=2))
    return path, plan, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', default='thick8')
    parser.add_argument('--duration', type=float, default=0, help='0 runs the full plan')
    parser.add_argument('--allow-arm-curl', action='store_true')
    parser.add_argument('--scene', type=Path, help='Use a separately authored scene and adjacent plan/config')
    parser.add_argument('--tuning', type=Path, help='Explicit physical/control parameters for a new attempt')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    report = {'task': 'bimanual_roll_towel', 'created_utc': datetime.now(timezone.utc).isoformat(),
              'status': 'NOT_RUN', 'physics_started': False, 'task_success': False,
              'finger_contact_verified': False, 'cloth_position_commands': 0, 'cloth_attachments': 0,
              'backend': 'Standalone NVIDIA PhysX GPU; NOT Isaac Sim Kit',
              'arm_locked_after_approach': not args.allow_arm_curl}
    stage = physx = None
    bindings = []; frames = []; qs = []; poses = []; targets = []; times = []
    try:
        import subprocess, sys
        # Stock USD authoring and ovstage use separate processes to avoid mixing
        # two independent USD runtimes in one address space.
        if args.scene is None:
            command = ('from pathlib import Path; from towel_roll.native_physx import build; '
                       'import sys,json; build(Path(sys.argv[1]),sys.argv[2],sys.argv[3]=="True",'
                       'json.loads(sys.argv[4]))')
            tuning = json.loads(args.tuning.read_text()) if args.tuning else {}
            subprocess.run([sys.executable, '-B', '-c', command, str(output), args.variant,
                            str(not args.allow_arm_curl),json.dumps(tuning)], check=True)
            (output/'tuning.json').write_text(json.dumps(tuning,indent=2))
            scene = output / 'scene.usda'
        else:
            scene = args.scene.resolve()
        plan = json.loads((scene.parent/'joint_plan.json').read_text())
        config = json.loads((scene.parent/'config.json').read_text())
        import ovstage, ovphysx
        from ovphysx import PhysX, PhysXConfig
        from ovphysx.types import TensorType
        from ovphysx.samples.python_samples.schema_support import physx_schema_paths
        ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
        stage = ovstage.Stage('benchmark-towel')
        ovstage.population.open_usd(stage, str(scene), ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physx = PhysX(active_cuda_gpus='0', config=PhysXConfig(
            carbonite_overrides={'/physics/suppressReadback': True}))
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        def binding(pattern, kind):
            result = physx.create_tensor_binding(pattern=pattern, tensor_type=kind)
            bindings.append(result)
            if result.count == 0: raise RuntimeError('Empty binding: ' + str(kind))
            return result
        robot = '/World/Robot/DualArm/Links/*'
        joint = binding(robot, TensorType.ARTICULATION_DOF_POSITION)
        control = binding(robot, TensorType.ARTICULATION_DOF_POSITION_TARGET)
        links = binding(robot, TensorType.ARTICULATION_LINK_POSE)
        cloth = binding('/World/TowelRoll/Towel', TensorType.SURFACE_DEFORMABLE_SIM_POSITION)
        vel = binding('/World/TowelRoll/Towel', TensorType.SURFACE_DEFORMABLE_SIM_VELOCITY)
        report['native_devices'] = {k: str(v.native_device) for k,v in [('joints',joint),('cloth',cloth)]}
        if any(v.native_device.device_type.value != 2 for v in (joint, cloth)):
            raise RuntimeError('GPU dynamics required; refusing CPU fallback')
        names = joint.dof_names
        if joint.shape != (1,54) or set(names) != set(plan['initial_joint_positions_rad']):
            raise RuntimeError('54-DOF name mapping mismatch')
        report.update(joint_names=names, body_names=links.body_names, cloth_nodes=cloth.shape[1],
                      version=ovphysx.__version__, variant=config['variant'])
        current = np.zeros(joint.shape,np.float32); p = np.zeros(cloth.shape,np.float32)
        v = np.zeros(vel.shape,np.float32); body = np.zeros(links.shape,np.float32)
        dt = config['physics_dt_s']; stride = max(1,round(1/(config['capture_fps']*dt)))
        samples = plan['samples']; duration = args.duration or samples[-1]['time_s']
        idx = 0; start = time.monotonic(); max_speed = 0.
        physx.warmup(); physx.wait_all()
        joint.read(current); report['initial_joint_error_rad'] = float(np.max(np.abs(
            current[0]-np.array([plan['initial_joint_positions_rad'][n] for n in names]))))
        report['physics_started'] = True
        for step in range(int(np.ceil(duration/dt))):
            t = step*dt
            while idx < len(samples)-2 and t > samples[idx+1]['time_s']: idx += 1
            a,b = samples[idx:idx+2]
            u = float(np.clip((t-a['time_s'])/(b['time_s']-a['time_s']),0,1))
            target = np.asarray([[(1-u)*a['joint_targets_rad'][n]+u*b['joint_targets_rad'][n]
                                  for n in names]],np.float32)
            control.write(target); physx.step_sync(dt)
            if step % stride == 0:
                joint.read(current); cloth.read(p); vel.read(v); links.read(body)
                if not all(np.isfinite(x).all() for x in (current,p,v,body)): raise RuntimeError('Nonfinite physics')
                max_speed = max(max_speed, float(np.linalg.norm(v,axis=-1).max()))
                frames.append(p[0].copy()); qs.append(current[0].copy()); poses.append(body[0].copy())
                targets.append(target[0].copy()); times.append((step+1)*dt)
                if p[0,:,2].min() < .65: raise RuntimeError('Cloth fell through/below table')
                if max_speed > 20: raise RuntimeError('Cloth numerical explosion')
            if step % 240 == 0: print(f'GPU cloth time={t:.1f}s phase={b["phase"]}',flush=True)
            if time.monotonic()-start > 600: raise TimeoutError('Physics probe exceeded 600 seconds')
        report.update(status='PHYSICS_ATTEMPT_RECORDED', simulated_seconds=duration,
                      max_cloth_speed_m_s=max_speed,
                      final_cloth_extent_m=np.ptp(frames[-1],axis=0).tolist(),
                      final_cloth_height_range_m=[float(frames[-1][:,2].min()),float(frames[-1][:,2].max())],
                      final_max_joint_tracking_error_rad=float(np.max(np.abs(qs[-1]-targets[-1]))),
                      note='Measured GPU interaction only. Finger/cloth contact and rolling success remain unverified.')
    except Exception as exc:
        report.update(status='PHYSICS_FAILED' if report['physics_started'] else 'STARTUP_ERROR',
                      error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    finally:
        if frames:
            np.savez_compressed(output/'measured_states.npz',cloth_vertices=frames,
                                joint_positions_rad=qs,link_poses_xyzw=poses,
                                joint_targets_rad=targets,time_s=times)
        for b in reversed(bindings): b.destroy()
        if physx is not None:
            if stage is not None and physx.is_stage_attached(stage): physx.wait_task(physx.detach_ovstage())
            physx.destroy()
        if stage is not None: stage.destroy()
        report['recorded_frames'] = len(frames)
        report['code_sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                 [Path(__file__),FOLDER/'surface.py',FOLDER/'joint_plan.py']}
        (output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['status']=='PHYSICS_ATTEMPT_RECORDED' else 2

if __name__ == '__main__': raise SystemExit(main())
