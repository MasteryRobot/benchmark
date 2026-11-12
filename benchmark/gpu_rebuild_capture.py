import os,json,runpy,sys,time,hashlib,traceback
from pathlib import Path
os.environ['OMNI_KIT_ACCEPT_EULA']='YES'
os.environ['PYTHONDONTWRITEBYTECODE']='1'
ROOT=Path(__file__).resolve().parent
os.chdir(ROOT);sys.path.insert(0,str(ROOT))
OUT=ROOT/'output';OUT.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':1280,'height':720,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'limit_cpu_threads':8})
import numpy as np
from PIL import Image
from pxr import Usd,UsdGeom,Gf,PhysxSchema
import omni.usd,omni.replicator.core as rep
try:
 if '--rebuild' in sys.argv:
  for script in ['build_scene.py','build_task_scenes.py','build_sharpa_scenes.py']:
   print('REBUILD',script,flush=True)
   old=sys.argv;sys.argv=[script];runpy.run_path(str(ROOT/script),run_name='__main__');sys.argv=old
  print('REBUILD_DONE',flush=True)
 entries=[]
 for d in sorted((ROOT/'tasks').glob('*/task.json'))+sorted((ROOT/'sharpa/scenes').glob('*/task.json'))+[ROOT/'towel_roll/task.json']:
  task=json.loads(d.read_text());entries.append((task,d.parent/'scene.usda'))
 cameras=json.loads((ROOT/'cameras.json').read_text())
 manifest={'renderer':'Isaac Sim 6.0.1 RTX RayTracedLighting','gpu':'NVIDIA GeForce RTX 5090','resolution':[1280,720],'capture_state':'authored initial state; timeline paused; no task success claim','scenes':[]}
 for i,(task,path) in enumerate(entries):
  sid=task['id'];dest=OUT/'views'/sid;dest.mkdir(parents=True,exist_ok=True)
  if (dest/'capture.json').exists():
   manifest['scenes'].append(json.loads((dest/'capture.json').read_text()));continue
  print('SCENE',i+1,len(entries),sid,flush=True)
  omni.usd.get_context().open_stage(str(path));stage=omni.usd.get_context().get_stage()
  for _ in range(12):app.update()
  row={'id':sid,'title':task.get('title',sid),'scene':str(path.relative_to(ROOT)),'scene_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'views':{}}
  for name in ['Head','LeftWrist','RightWrist','LeftSide','RightSide']:
   cam=cameras[name]['prim_path']
   assert stage.GetPrimAtPath(cam),cam
   rp=rep.create.render_product(cam,(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp])
   dep=rep.AnnotatorRegistry.get_annotator('distance_to_camera');dep.attach([rp])
   rep.orchestrator.step(rt_subframes=16,pause_timeline=True,delta_time=0)
   arr=np.asarray(rgb.get_data()).copy();depth=np.asarray(dep.get_data()).copy()
   assert arr.shape[:2]==(720,1280) and arr[:,:,:3].std()>1,(sid,name,arr.shape)
   p=dest/(name+'.png');Image.fromarray(arr).save(p)
   np.savez_compressed(dest/(name+'_depth.npz'),distance_m=depth)
   row['views'][name]={'file':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'std':float(arr[:,:,:3].std()),'finite_depth_fraction':float(np.isfinite(depth).mean())}
   rgb.detach([rp]);dep.detach([rp]);rp.destroy()
  with Usd.EditContext(stage,stage.GetSessionLayer()):
   cam=UsdGeom.Camera.Define(stage,'/InspectionCamera');cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.65,3.1,2.4),Gf.Vec3d(0,.18,.90),Gf.Vec3d(0,0,1)).GetInverse());cam.CreateFocalLengthAttr(46);cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(20.25);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
   rp=rep.create.render_product('/InspectionCamera',(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp]);rep.orchestrator.step(rt_subframes=16,pause_timeline=True,delta_time=0);Image.fromarray(rgb.get_data()).save(dest/'Overview.png');rgb.detach([rp]);rp.destroy()
  (dest/'capture.json').write_text(json.dumps(row,ensure_ascii=False,indent=2));manifest['scenes'].append(row)
  (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
  print('CAPTURED',sid,flush=True)
 (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
 print('ALL_DONE',len(manifest['scenes']),flush=True)
finally:app.close()
