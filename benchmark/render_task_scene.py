"""Validate one task with PhysX and capture five sensors plus inspection views."""
import argparse,os,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('task');args=p.parse_args()
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':720,'renderer':'RayTracedLighting','limit_cpu_threads':8,'anti_aliasing':3})
import faulthandler
faulthandler.dump_traceback_later(45,repeat=True)
import numpy as np,omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdPhysics,Usd,Gf
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
ROOT=Path(__file__).resolve().parent;folder=ROOT/'tasks'/args.task;out=folder/'images';out.mkdir(exist_ok=True);task=json.loads((folder/'task.json').read_text());specs=json.loads((ROOT/'cameras.json').read_text())
omni.usd.get_context().open_stage(str(folder/'scene.usda'));stage=omni.usd.get_context().get_stage()
for _ in range(10):app.update()
world=World(stage_units_in_meters=1,physics_dt=1/240,rendering_dt=1/30)
robot=world.scene.add(SingleArticulation('/World/Robot/DualArm',name='robot'));obj=world.scene.add(SingleRigidPrim(task['object_prim'],name='object'))
world.reset();q0=robot.get_joint_positions().copy();p0,rot0=obj.get_world_pose();p0=p0.copy()
for _ in range(1200):world.step(render=False)
position,rotation=obj.get_world_pose();speed=np.linalg.norm(obj.get_linear_velocity());angular=np.linalg.norm(obj.get_angular_velocity());drift=float(np.max(abs(robot.get_joint_positions()-q0)))
world.render();bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetPrimAtPath(task['object_prim'])).ComputeAlignedRange()
report={'task':args.task,'physics_steps':1200,'seconds':5,'robot_joint_drift_rad':drift,'initial_object_position_m':p0.tolist(),'settled_object_position_m':position.tolist(),'settled_orientation_wxyz':rotation.tolist(),'object_displacement_m':float(np.linalg.norm(position-p0)),'object_min_z_m':float(bounds.GetMin()[2]),'linear_speed_m_s':float(speed),'angular_speed_rad_s':float(angular),'sensor_count':sum(p.IsA(UsdGeom.Camera) and str(p.GetPath()).startswith('/World/') for p in stage.Traverse())}
report['stable']=bool(drift<.02 and bounds.GetMin()[2]>.793 and np.linalg.norm(position[:2]-p0[:2])<.035 and speed<.01 and angular<.05)

def capture(path,name,res=(1280,720),depth=False):
 rp=rep.create.render_product(path,res);rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp]);da=None
 if depth:da=rep.AnnotatorRegistry.get_annotator('distance_to_camera');da.attach([rp])
 rep.orchestrator.step(rt_subframes=16,pause_timeline=True,delta_time=0)
 arr=rgb.get_data();assert arr.size;Image.fromarray(arr).save(out/(name+'.png'))
 if da:
  np.savez_compressed(out/(name+'_depth.npz'),distance_m=da.get_data());da.detach([rp])
 rgb.detach([rp]);rp.destroy()
for name,spec in specs.items():capture(spec['prim_path'],name.lower(),depth=True)
with Usd.EditContext(stage,stage.GetSessionLayer()):
 x,y,z=position;inspection=[('detail',(x+.32,y+.44,1.29),(x,y,.91),32),('overview',(2.65,3.1,2.40),(0,.18,.90),46)]
 for name,eye,target,focal in inspection:
  cam=UsdGeom.Camera.Define(stage,'/TaskInspection');cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(0,0,1)).GetInverse());cam.CreateFocalLengthAttr(focal);cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(20.25);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,20));capture('/TaskInspection',name,(1600,900));stage.RemovePrim('/TaskInspection')
(folder/'validation.json').write_text(json.dumps(report,indent=2));print('VALIDATION',json.dumps(report),flush=True)
app.close()
