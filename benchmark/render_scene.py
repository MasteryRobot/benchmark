"""Isaac Sim RTX five-camera capture, PhysX hold test, and depth coverage."""
import argparse,os,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--views',nargs='+',default=['LeftWrist','RightWrist','Head','LeftSide','RightSide']);p.add_argument('--steps',type=int,default=600);p.add_argument('--width',type=int,default=1920);p.add_argument('--out',default='images');args=p.parse_args()
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':args.width,'height':int(args.width*9/16),'renderer':'RayTracedLighting','limit_cpu_threads':8,'anti_aliasing':3})
from pxr import UsdGeom,UsdPhysics,Gf,Usd
import numpy as np,omni.usd,omni.replicator.core as rep
from PIL import Image
from isaacsim.core.api import World
ROOT=Path(__file__).resolve().parent;camera_specs=json.loads((ROOT/'cameras.json').read_text());out=ROOT/args.out;out.mkdir(exist_ok=True)
omni.usd.get_context().open_stage(str(ROOT/'scene.usda'));stage=omni.usd.get_context().get_stage()
for _ in range(15):app.update()
report={'renderer':'Isaac Sim 6.0.1 RTX','views':[],'articulations':[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)],'revolute_joints':sum(p.IsA(UsdPhysics.RevoluteJoint) for p in stage.Traverse()),'sensor_cameras':sum(p.IsA(UsdGeom.Camera) and str(p.GetPath()).startswith('/World/') for p in stage.Traverse())}
if args.steps:
 from isaacsim.core.prims import SingleRigidPrim,SingleArticulation
 world=World(stage_units_in_meters=1,physics_dt=1/120,rendering_dt=1/30);poses=json.loads((ROOT/'poses.json').read_text());arms=[]
 for name in poses['robots']:arms.append(world.scene.add(SingleArticulation(prim_path='/World/Robot/'+name,name=name)))
 rods=[world.scene.add(SingleRigidPrim('/World/Props/Chopstick'+str(i),name='stick'+str(i))) for i in [1,2]]
 world.reset();report['initial_state_errors_rad']=[]
 for arm,(name,cfg) in zip(arms,poses['robots'].items()):
  q=np.array([cfg['joint_positions'][joint] for joint in arm.dof_names]);report['initial_state_errors_rad'].append(float(np.max(np.abs(arm.get_joint_positions()-q))))
 q0=[arm.get_joint_positions().copy() for arm in arms];p0=[rod.get_world_pose()[0].copy() for rod in rods]
 for i in range(args.steps):world.step(render=False)
 report['physics_steps']=args.steps;report['simulated_seconds']=args.steps/120;report['max_joint_drift_rad']=[float(np.max(np.abs(arm.get_joint_positions()-initial))) for arm,initial in zip(arms,q0)]
 report['chopstick_positions']=[rod.get_world_pose()[0].tolist() for rod in rods];report['chopstick_displacement_m']=[float(np.linalg.norm(rod.get_world_pose()[0]-initial)) for rod,initial in zip(rods,p0)]
 report['stable']=all(d<.04 for d in report['max_joint_drift_rad']) and all(.80<p[2]<.85 for p in report['chopstick_positions']);print('PHYSICS',json.dumps(report),flush=True)
for view in args.views:
 spec=camera_specs[view];rp=rep.create.render_product(spec['prim_path'],(args.width,round(args.width*spec['resolution'][1]/spec['resolution'][0])));rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_camera');rgb.attach([rp]);depth.attach([rp]);rep.orchestrator.step(rt_subframes=32,pause_timeline=True,delta_time=0.0)
 data=rgb.get_data();d=depth.get_data()
 if data.size==0:raise RuntimeError('Empty render '+view)
 filename=out/(view.lower()+'.png');Image.fromarray(data).save(filename);np.savez_compressed(out/(view.lower()+'_depth.npz'),distance_m=d);report['views'].append(str(filename.relative_to(ROOT)));print('RENDERED',view,flush=True);rgb.detach([rp]);depth.detach([rp]);rp.destroy()
# Inspection views exist only in the session layer, not as saved sensors.
with Usd.EditContext(stage,stage.GetSessionLayer()):
 for label,eye,target,focal in [('overview',(3.1,3.8,2.6),(0,.05,.86),50),('camera_mounts',(.95,1.30,1.24),(0,-.08,1.16),45)]:
  c=UsdGeom.Camera.Define(stage,'/InspectionCamera');c.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());c.CreateFocalLengthAttr(focal);c.CreateHorizontalApertureAttr(36);c.CreateVerticalApertureAttr(27)
  rp=rep.create.render_product('/InspectionCamera',(1600,1200));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp]);rep.orchestrator.step(rt_subframes=32,pause_timeline=True,delta_time=0);Image.fromarray(rgb.get_data()).save(out/(label+'.png'));rgb.detach([rp]);rp.destroy();stage.RemovePrim('/InspectionCamera')
(ROOT/'validation.json').write_text(json.dumps(report,indent=2));app.close()
