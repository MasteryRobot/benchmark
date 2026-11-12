import os,json
from pathlib import Path
os.environ['OMNI_KIT_ACCEPT_EULA']='YES'
ROOT=Path(__file__).resolve().parent
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'limit_cpu_threads':8})
import omni.usd,numpy as np,traceback
def array(x):return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
try:
 path=ROOT/'tasks/03_pick_phone/scene.usda'
 omni.usd.get_context().open_stage(str(path));stage=omni.usd.get_context().get_stage()
 world=World(stage_units_in_meters=1,physics_dt=1/240,rendering_dt=1/30,backend='torch',device='cuda:0')
 pc=world.get_physics_context();pc.enable_gpu_dynamics(True);pc.set_broadphase_type('GPU')
 robot=world.scene.add(SingleArticulation('/World/Robot/DualArm',name='robot'))
 obj=world.scene.add(SingleRigidPrim('/World/Task/Object',name='object'))
 world.reset();q0=array(robot.get_joint_positions());p0=array(obj.get_world_pose()[0])
 for _ in range(240):world.step(render=False)
 q=array(robot.get_joint_positions());p=array(obj.get_world_pose()[0])
 r={'scene':'03_pick_phone','physics_device':world.device,'gpu_dynamics':pc.is_gpu_dynamics_enabled(),'broadphase':pc.get_broadphase_type(),'steps':240,'seconds':1.,'dof_count':len(q),'joint_drift_rad':float(abs(q-q0).max()),'object_position_m':p.tolist(),'object_displacement_m':float(np.linalg.norm(p-p0)),'finite_state':bool(np.isfinite(q).all() and np.isfinite(p).all()),'scope':'GPU physics smoke check only; not task success or all-scene dynamics verification'}
 (ROOT/'output/gpu_physics_smoke.json').write_text(json.dumps(r,indent=2));print('GPU_SMOKE',json.dumps(r),flush=True)
except Exception:
 traceback.print_exc()
finally:app.close()
