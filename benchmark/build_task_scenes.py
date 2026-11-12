"""Build ten individually loadable manipulation environments over shared workcell.usdc."""
from pathlib import Path
import json,math
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
from scipy.spatial.transform import Rotation
from robot_model import transform
from usd_scene import Scene,xform,attr,add_api
from task_objects import BUILDERS,rounded,rod
from task_runtime import RUNTIME_DEFAULTS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'tasks';TABLE=.795
OUT.mkdir(parents=True,exist_ok=True)
if not (OUT/'workcell.usdc').is_file():
 raise FileNotFoundError('Missing tasks/workcell.usdc; run build_scene.py first')
TASKS=[
 ('01_pick_spoon','拿勺子','spoon','left','lift'),
 ('02_pick_knife','拿刀具','knife','right','lift'),
 ('03_pick_phone','拿手机','phone','left','lift'),
 ('04_phone_portrait_to_landscape','手机竖屏变横屏','phone','right','portrait_to_landscape'),
 ('05_flip_phone','手机翻面','phone','left','flip'),
 ('06_reorient_battery','拿电池调整 pose','battery','right','positive_up'),
 ('07_reorient_lighter','拿打火机调整到使用姿态','lighter','left','upright'),
 ('08_pick_lipstick','拿口红','lipstick','right','lift'),
 ('09_rotate_cotton_swab','拿棉签转棉签','swab','left','roll'),
 ('10_pick_scissors','拿剪刀','scissors','right','lift'),
]
index=[]
for slug,title,kind,hand,action in TASKS:
 folder=OUT/slug;folder.mkdir(exist_ok=True)
 stage=Usd.Stage.CreateNew(str(folder/'scene.usda'));stage.GetRootLayer().subLayerPaths=['../workcell.usdc'];stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
 a=Scene(stage,material_prefix='/World/TaskLooks');UsdGeom.Xform.Define(stage,'/World/Task')
 side=-1 if hand=='left' else 1;x=side*.25;y=.42;angle=np.deg2rad(18*side)
 # Flush colored region, purely visual; the original tabletop remains the collider.
 pad=rounded(a,'/World/Task/WorkArea',.29,.30,TABLE+.0001,TABLE+.0003,.018,[.10,.135,.15],False);pad.AddTranslateOp().Set(Gf.Vec3d(x,y,0))
 p='/World/Task/Object';obj=UsdGeom.Xform.Define(stage,p);meta=BUILDERS[kind](a,p)
 R=Rotation.from_euler('z',angle).as_matrix();z=TABLE+.012
 if kind=='spoon':z=TABLE+.0025
 elif kind=='knife':z=TABLE+.010
 elif kind=='phone':z=TABLE+.019
 elif kind in ['battery','lipstick','lighter']:
  R=R@Rotation.from_euler('x',np.pi/2).as_matrix();z=TABLE+(.018 if kind=='lighter' else .017)
 elif kind=='swab':z=TABLE+.008
 elif kind=='scissors':z=TABLE+.006
 if action=='portrait_to_landscape':
  R=Rotation.from_euler('x',np.deg2rad(70)).as_matrix();z=TABLE+.0755*np.sin(np.deg2rad(70))+.016
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=[x,y,z];xform(obj,T)
 UsdPhysics.RigidBodyAPI.Apply(obj.GetPrim());UsdPhysics.MassAPI.Apply(obj.GetPrim()).CreateMassAttr(meta['mass']);add_api(obj.GetPrim(),'PhysxRigidBodyAPI');attr(obj.GetPrim(),'physxRigidBody:enableCCD',kind!='scissors',Sdf.ValueTypeNames.Bool)
 attr(obj.GetPrim(),'physxRigidBody:solverPositionIterationCount',32,Sdf.ValueTypeNames.Int);attr(obj.GetPrim(),'physxRigidBody:solverVelocityIterationCount',16,Sdf.ValueTypeNames.Int)
 attr(obj.GetPrim(),'physxRigidBody:angularDamping',2.0 if kind=='swab' else .5)
 support='/World/Task/Fixture';fix=UsdGeom.Xform.Define(stage,support)
 if action=='portrait_to_landscape':
  xform(fix,T)
  a.cube(support+'/Back',[0,0,-.009],[.09,.166,.008],[.22,.27,.30])
  a.cube(support+'/Stop',[0,-.080,.005],[.09,.008,.032],[.12,.16,.18])
  a.cube('/World/Task/StandFoot',[x,y-.014,TABLE+.006],[.105,.095,.012],[.14,.18,.21])
 elif kind=='phone':
  S=T.copy();S[2,3]=TABLE;xform(fix,S)
  for k,yy in enumerate([-.052,.052]):a.cube(support+'/Rail'+str(k),[0,yy,.006],[.082,.009,.012],[.065,.10,.12])
 elif kind in ['battery','lipstick','lighter','swab']:
  # Two transverse shallow V seats prevent cylinders rolling away during reset.
  S=transform([x,y,TABLE],rpy=[0,0,angle]);xform(fix,S)
  for k,yy in enumerate([-.030,.030] if kind=='swab' else [-.015,.015]):
   h=.004 if kind=='swab' else .009
   a.cube(support+'/Seat'+str(k),[0,yy,h/2],[.025,.006,h],[.14,.20,.23])
   if kind not in ['lighter','swab']:
    for sign in [-1,1]:a.cube(support+f'/Lip{k}_{sign+1}',[sign*(.0035 if kind=='swab' else .0105),yy,h+.001],[.002,.006,.003],[.20,.27,.30])
 # Goal is a wire box and local orientation axes, never a second physical object.
 goal=T.copy();goal[:3,3]=[x,y+.015,max(1.01,z+.13)]
 if action=='portrait_to_landscape':goal[:3,:3]=R@Rotation.from_euler('z',np.pi/2).as_matrix();goal[2,3]=1.055
 elif action=='flip':goal[:3,:3]=R@Rotation.from_euler('x',np.pi).as_matrix()
 elif action in ['positive_up','upright']:goal[:3,:3]=np.eye(3)
 elif action=='roll':goal[:3,:3]=R@Rotation.from_euler('y',np.pi).as_matrix()
 gp='/World/Task/Goal';g=UsdGeom.Xform.Define(stage,gp);xform(g,goal);g.GetPrim().CreateAttribute('task:isGoalMarker',Sdf.ValueTypeNames.Bool).Set(True)
 dims=np.maximum(np.array(meta['size'])+.008,.018)/2;corners=np.array([[i*dims[0],j*dims[1],k*dims[2]] for i in [-1,1] for j in [-1,1] for k in [-1,1]])
 for i,c in enumerate(corners):
  for j in range(i+1,len(corners)):
   if np.count_nonzero(c!=corners[j])==1:rod(a,gp+f'/Edge{i}_{j}',c,corners[j],.0006,[.14,.72,.65],False)
 for i,col in enumerate([[.86,.16,.12],[.18,.78,.29],[.15,.41,.92]]):
  end=np.eye(3)[i]*.045;rod(a,gp+'/Axis'+str(i),[0,0,0],end,.0011,col,False)
 # Material and small-object contact offsets.
 material=UsdShade.Material.Define(stage,'/World/TaskLooks/Contact');mp=UsdPhysics.MaterialAPI.Apply(material.GetPrim());mp.CreateStaticFrictionAttr(.8);mp.CreateDynamicFrictionAttr(.65);mp.CreateRestitutionAttr(.01)
 for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Task')):
  if prim.HasAPI(UsdPhysics.CollisionAPI):
   attr(prim,'physxCollision:contactOffset',.0003);attr(prim,'physxCollision:restOffset',0);UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
 # Author task metadata at a stable path for scripts and future policies.
 local_grasp=np.array(meta['grasp_local']);grasp=(T@np.r_[local_grasp,1])[:3]
 data={'id':slug,'title':title,'object':kind,'preferred_hand':hand,'action':action,'scene':'scene.usda','object_prim':p,'fixture_prim':support,'goal_prim':gp,'object_design':meta,'initial_pose_world':T.tolist(),'goal_pose_world':goal.tolist(),'grasp_reference_world_m':grasp.tolist(),'success':{'position_tolerance_m':.015,'rotation_tolerance_deg':12,'minimum_lift_m':.10,'requires_grasp_contact':True},'units':'m','object_source':'Locally authored parametric task props; dimensions and masses are design estimates, not manufacturer specifications.','status':'Scene and target definition only; no autonomous grasp/reorientation policy.'}
 data['runtime']=dict(RUNTIME_DEFAULTS)
 data['status']='Scene, reset/control adapter and success evaluator implemented; live runtime verification required; no autonomous manipulation policy.'
 prim=stage.GetPrimAtPath('/World/Task');prim.CreateAttribute('task:id',Sdf.ValueTypeNames.String).Set(slug);prim.CreateAttribute('task:title',Sdf.ValueTypeNames.String).Set(title);prim.CreateAttribute('task:goalPose',Sdf.ValueTypeNames.Matrix4d).Set(Gf.Matrix4d(goal.T.tolist()))
 stage.SetMetadata('customLayerData',{'task':slug,'title':title,'sceneVersion':'6','sharedWorkcell':'../workcell.usdc'})
 stage.GetRootLayer().Save();(folder/'task.json').write_text(json.dumps(data,ensure_ascii=False,indent=2));index.append(data)
(OUT/'index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2));print('Built',len(index),'task scenes')
if (ROOT/'assets/realistic/catalog.json').is_file():
 from tools.real_assets.upgrade_baseline import main as attach_scan_visuals
 attach_scan_visuals()
