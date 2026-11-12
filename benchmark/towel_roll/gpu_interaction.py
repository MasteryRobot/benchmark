"""GPU joint-controlled towel attempt. No commands ever move cloth vertices.

The runner records actual articulation and deformable tensor states. A completed
attempt is not automatically a successful roll or verified fingertip grasp.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from .config import FOLDER
from .joint_plan import generate, validate


def as_numpy(value):
    if hasattr(value,'detach'): return value.detach().cpu().numpy()
    if hasattr(value,'numpy'): return value.numpy()
    return np.asarray(value)


def run(output):
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    cfg_path=FOLDER/'interaction_config.json'
    cfg=json.loads(cfg_path.read_text())
    report={'task':'bimanual_roll_towel','created_utc':datetime.now(timezone.utc).isoformat(),
            'status':'PREFLIGHT','physics_started':False,'physics_success_verified':False,
            'finger_contact_verified':False,'cloth_position_commands':0,'cloth_attachments':0,
            'variant':cfg['variant'],'blockers':[],
            'config_sha256':hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
            'code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                           [Path(__file__),FOLDER/'joint_plan.py',FOLDER/'surface.py']}}
    app=world=None
    frames=[]; joint_states=[]; times=[]; phases=[]
    try:
        probe=subprocess.run(['nvidia-smi','--query-gpu=index,name,memory.used','--format=csv,noheader'],
                             capture_output=True,text=True,timeout=15)
        report['gpu_probe']={'returncode':probe.returncode,'output':(probe.stdout+probe.stderr).strip()}
        if probe.returncode: report['blockers'].append('GPU driver is inaccessible in this execution environment')
        if importlib.util.find_spec('isaacsim') is None: report['blockers'].append('Isaac Sim is unavailable')
        if report['blockers']:
            report['status']='BLOCKED';return 2
        plan=validate(generate(cfg))
        (output/'joint_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True})
        import omni.usd
        import omni.physics.tensors as tensors
        from pxr import Sdf, UsdGeom, UsdPhysics, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from .surface import author
        scene=output/'scene.usda'
        layer=Sdf.Layer.CreateNew(str(scene.resolve()))
        layer.subLayerPaths=[str((FOLDER/'scene_isaac.usda').resolve())];layer.Save()
        omni.usd.get_context().open_stage(str(scene.resolve()))
        for _ in range(10):app.update()
        stage=omni.usd.get_context().get_stage()
        author(stage,cfg['variant'],cfg['surface_mesh_spacing_m'])
        scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1: raise RuntimeError('Expected exactly one physics scene')
        gpu=PhysxSchema.PhysxSceneAPI.Apply(scenes[0])
        gpu.CreateEnableGPUDynamicsAttr(True);gpu.CreateBroadphaseTypeAttr('GPU')
        for prim in stage.Traverse():
            if 'Attachment' in str(prim.GetTypeName()):
                raise RuntimeError('Unexpected attachment in interaction scene')
        mesh=UsdGeom.Mesh.Get(stage,'/World/TowelRoll/Towel')
        expected_nodes=len(mesh.GetPointsAttr().Get())
        faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),dtype=np.int32).reshape(-1,3)
        stage.GetRootLayer().Save()
        dt=cfg['physics_dt_s']
        world=World(stage_units_in_meters=1,physics_dt=dt,rendering_dt=dt,backend='torch',device='cuda:0')
        robot=world.scene.add(SingleArticulation('/World/Robot/DualArm',name='towel_robot'))
        world.reset()
        names=list(robot.dof_names)
        if len(names)!=54 or set(names)!=set(plan['initial_joint_positions_rad']):
            raise RuntimeError('Articulation joint mapping differs from the 54-DOF model')
        q0=np.array([plan['initial_joint_positions_rad'][n] for n in names])
        # Only robot initialization uses state setters. All subsequent commands
        # are drive targets. The cloth is never repositioned by this runner.
        robot.set_joint_positions(q0);robot.set_joint_velocities(np.zeros_like(q0))
        robot.apply_action(ArticulationAction(joint_positions=q0))
        sim=tensors.create_simulation_view('torch');sim.set_subspace_roots('/')
        cloth=sim.create_surface_deformable_body_view('/World/TowelRoll/Towel')
        if cloth.count!=1:raise RuntimeError('Expected one native surface deformable')
        nodes=cloth.get_simulation_nodal_positions()
        if not getattr(nodes,'is_cuda',False):raise RuntimeError('Cloth tensors are not on CUDA; refusing CPU fallback')
        if tuple(nodes.shape)!=(1,expected_nodes,3):raise RuntimeError('Cloth topology differs from authored mesh')
        kinematic=as_numpy(cloth.get_simulation_nodal_kinematic_targets())
        if not np.all(kinematic[0,:,3]==1):raise RuntimeError('Cloth contains kinematically controlled nodes')
        report['cuda_device']=str(nodes.device)
        report['joint_names']=names;report['physics_started']=True
        report['contact_validation']='Native cloth/hand contact classification is pending; no success claim is made.'
        samples=plan['samples'];idx=0;stride=max(1,round(1/(cfg['capture_fps']*dt)))
        start=time.monotonic()
        for step in range(int(np.ceil(samples[-1]['time_s']/dt))):
            t=step*dt
            if time.monotonic()-start>900:raise TimeoutError('Attempt exceeded 900 seconds')
            while idx<len(samples)-2 and t>samples[idx+1]['time_s']:idx+=1
            a,b=samples[idx:idx+2]
            u=float(np.clip((t-a['time_s'])/(b['time_s']-a['time_s']),0,1))
            q=np.array([(1-u)*a['joint_targets_rad'][n]+u*b['joint_targets_rad'][n] for n in names])
            robot.apply_action(ArticulationAction(joint_positions=q))
            world.step(render=False)
            if step%stride==0:
                points=as_numpy(cloth.get_simulation_nodal_positions())[0].copy()
                measured=as_numpy(robot.get_joint_positions()).copy().reshape(-1)
                if not np.isfinite(points).all() or not np.isfinite(measured).all():
                    raise RuntimeError('Non-finite native physics state')
                frames.append(points);joint_states.append(measured);times.append((step+1)*dt);phases.append(b['phase'])
            if step%240==0:print(f'GPU joint interaction: {t:.1f}s / {samples[-1]["time_s"]:.1f}s',flush=True)
        np.savez_compressed(output/'measured_states.npz',cloth_vertices=np.asarray(frames),faces=faces,
                            joint_positions_rad=np.asarray(joint_states),joint_names=np.asarray(names),
                            time_s=np.asarray(times),phases=np.asarray(phases))
        report.update(status='ATTEMPT_RECORDED_NOT_VALIDATED',recorded_frames=len(frames),
                      simulated_seconds=times[-1],recording='measured_states.npz')
        return 0
    except Exception as exc:
        report['status']='ERROR' if report['physics_started'] else 'BLOCKED'
        report['blockers'].append(f'{type(exc).__name__}: {exc}')
        return 2
    finally:
        try:
            if world is not None:world.stop()
            if app is not None:app.close()
        finally:
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    return run(p.parse_args().output)


if __name__=='__main__':raise SystemExit(main())
