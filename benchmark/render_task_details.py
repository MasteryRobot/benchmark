"""Frame every task's complete initial object and target using a front inspection view."""
import os,json
from pathlib import Path
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'limit_cpu_threads':8,'renderer':'RayTracedLighting'})
import numpy as np,omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,Gf,Usd
from PIL import Image
from scipy.spatial.transform import Rotation
from usd_scene import xform
ROOT=Path(__file__).resolve().parent
for task in json.loads((ROOT/'tasks/index.json').read_text()):
 folder=ROOT/'tasks'/task['id'];report=json.loads((folder/'validation.json').read_text());omni.usd.get_context().open_stage(str(folder/'scene.usda'));stage=omni.usd.get_context().get_stage()
 for _ in range(3):app.update()
 with Usd.EditContext(stage,stage.GetSessionLayer()):
  T=np.eye(4);T[:3,3]=report['settled_object_position_m'];w,x,y,z=report['settled_orientation_wxyz'];T[:3,:3]=Rotation.from_quat([x,y,z,w]).as_matrix();xform(UsdGeom.Xform(stage.GetPrimAtPath(task['object_prim'])),T)
  x,y,z=T[:3,3];eye=[x+.32,y+.44,1.29];target=[x,y,.91];focal=32
  if task['action']=='portrait_to_landscape':eye=[x-.15,y-.12,1.65];target=[x,y,.95];focal=48
  c=UsdGeom.Camera.Define(stage,'/TaskDetail');c.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(0,0,1)).GetInverse());c.CreateFocalLengthAttr(focal);c.CreateHorizontalApertureAttr(36);c.CreateVerticalApertureAttr(20.25);c.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
  rp=rep.create.render_product('/TaskDetail',(1600,900));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp]);rep.orchestrator.step(rt_subframes=16,pause_timeline=True,delta_time=0);Image.fromarray(rgb.get_data()).save(folder/'images/detail.png');rgb.detach([rp]);rp.destroy()
 print('DETAIL',task['id'],flush=True)
app.close()
