"""Rebuild the dealer-table Generic dual arm + Articulated hand + five mounted cameras scene (metres, Z up).
Run: python build_scene.py
"""
from pathlib import Path
import json,math
import numpy as np
from shapely.geometry import Polygon,box
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf,Sdf
from robot_model import Robot,transform
from usd_scene import Scene,vec,xform,add_api,attr
ROOT=Path(__file__).resolve().parent
(ROOT/'tasks').mkdir(parents=True,exist_ok=True)
stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1);stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim());stage.SetTimeCodesPerSecond(120)
a=Scene(stage)
physics=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');physics.CreateGravityDirectionAttr(vec([0,0,-1]));physics.CreateGravityMagnitudeAttr(9.81);add_api(physics.GetPrim(),'PhysxSceneAPI');attr(physics.GetPrim(),'physxScene:solverType','TGS',Sdf.ValueTypeNames.Token)
# Quiet studio floor, with enough space to read the table and pedestal silhouette.
a.cube('/World/Studio/Floor',[0,0,-.035],[12,12,.07],[.17,.195,.23],rough=.8)
# Semi-ellipse front; concave rounded dealer bay at the rear. Boundary has no
# raster sampling or sawtooth edges. All slabs have finite physical thickness.
outer=[(1.04*math.cos(t),-.22+1.10*math.sin(t)) for t in np.linspace(0,math.pi,181)]
notch=[(.40*math.cos(t),-.22+.36*math.sin(t)) for t in np.linspace(math.pi,0,81)]
outline=Polygon(outer+notch).buffer(.025,quad_segs=12).buffer(-.025,quad_segs=12)
felt=outline.buffer(-.036,quad_segs=12)
a.extrusion('/World/Table/WoodCore',outline,.724,.775,[.095,.060,.042],rough=.5)
a.extrusion('/World/Table/EdgeBand',outline,.765,.793,[.48,.52,.57],metal=.55,rough=.25)
a.extrusion('/World/Table/Felt',felt,.791,.795,[.035,.045,.052],rough=.95)
# Pale thin perimeter lip, including the operator recess.
a.extrusion('/World/Table/Rim',outline.difference(felt),.793,.799,[.65,.70,.74],rough=.35)
for i,(x,y) in enumerate([(-.76,.04),(.76,.04),(-.55,.63),(.55,.63)]):
 a.cylinder(f'/World/Table/Leg{i}/Foot',(x,y,.022),.055,.044,[.035,.04,.05],metal=.1)
 a.cylinder(f'/World/Table/Leg{i}/Column',(x,y,.375),.026,.69,[.17,.20,.24],metal=.7)
 a.cylinder(f'/World/Table/Leg{i}/Flange',(x,y,.708),.06,.018,[.17,.20,.24],metal=.7)
for y in [.04,.63]:a.cube(f'/World/Table/Frame/Cross{int(y*100)}',[0,y,.69],[1.52 if y<.1 else 1.10,.035,.04],[.12,.14,.17],metal=.65)
# Original DualArm assembly (CAD pedestal and both chains), all source mounts
# preserved. Only the whole-robot world transform centers it at the dealer bay.
poses=json.loads((ROOT/'poses.json').read_text())
for name,cfg in poses['robots'].items():
 robot=Robot(ROOT/cfg['urdf']);a.robot('/World/Robot/'+name,robot,cfg['joint_positions'],np.array(cfg['base_transform']))
from sensor_rig import add_sensors
frames=robot.fk(cfg['joint_positions'],np.array(cfg['base_transform']))
add_sensors(a,robot,frames,ROOT)
# Freestanding chopstick rack: two grooved walnut bridges with small feet.
# Groove section is extruded along Y, leaving two true concave seats.
rail_y=[.35,.51]
xs=[-.021,.021];groove_radius=.0050;bar_top=.824
for index,y in enumerate(rail_y):
 top=[(-.082,bar_top)]
 for cx in xs:
  top.append((cx-groove_radius,bar_top))
  top.extend((cx+groove_radius*math.cos(t),bar_top+groove_radius*math.sin(t)) for t in np.linspace(math.pi,2*math.pi,21))
 top.append((.082,bar_top))
 cross=Polygon([(-.082,.802),(.082,.802)]+list(reversed(top)))
 # Build section in XY then rotate: section X->world X, section Y->world Z,
 # extrusion Z->world -Y.
 c=a.extrusion(f'/World/Rack/Bridge{index}',cross,-.014,.014,[.25,.115,.048],coll=True,rough=.48)
 T=transform([0,y,0],[math.pi/2,0,0]);xform(c,T)
 for k,x in enumerate([-.066,.066]):a.cube(f'/World/Rack/Foot{index}_{k}',[x,y,.7985],[.020,.025,.007],[.08,.06,.045],rough=.6)
# Exactly two tapered chopsticks, independent free rigid bodies.
for i,x in enumerate(xs):
 n=32;length=.265;r0=.0039;r1=.0027;v=[]
 for y,r in [(-length/2,r0),(length/2,r1)]:
  for t in np.linspace(0,2*math.pi,n,endpoint=False):v.append([r*math.cos(t),y,r*math.sin(t)])
 f=[]
 for k in range(n):
  j=(k+1)%n;f.extend([[k,j,n+j],[k,n+j,n+k]])
 for k in range(1,n-1):f.extend([[0,k+1,k],[n,n+k,n+k+1]])
 # Seats constrain lateral roll; center height accounts for radius at bridges.
 c=a.mesh(f'/World/Props/Chopstick{i+1}',v,f,[.58,.32,.12],True,'convexHull',rough=.45);c.AddTranslateOp().Set(Gf.Vec3d(x,.43,.824));UsdPhysics.RigidBodyAPI.Apply(c.GetPrim());UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(.009)
 add_api(c.GetPrim(),'PhysxRigidBodyAPI');attr(c.GetPrim(),'physxRigidBody:enableCCD',True,Sdf.ValueTypeNames.Bool)
# Shared contact material: holds the two light rods on their shallow grooves.
from pxr import UsdShade
pm=UsdShade.Material.Define(stage,'/World/Looks/Contact');pa=UsdPhysics.MaterialAPI.Apply(pm.GetPrim());pa.CreateStaticFrictionAttr(.8);pa.CreateDynamicFrictionAttr(.65);pa.CreateRestitutionAttr(.03)
for prim in stage.Traverse():
 if prim.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm,materialPurpose='physics')
# Balanced studio lighting for RTX; do not use the Storm-specific intensity.
d=UsdLux.DomeLight.Define(stage,'/World/Lights/Fill');d.CreateIntensityAttr(350);d.CreateColorAttr(vec([.78,.85,1.0]))
for name,loc,intensity,size in [('Key',[-1.5,1.5,3.5],2100,3),('Rim',[1,-1.8,2.8],1600,2)]:
 light=UsdLux.RectLight.Define(stage,'/World/Lights/'+name);light.CreateIntensityAttr(intensity);light.CreateWidthAttr(size);light.CreateHeightAttr(size);T=np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*loc),Gf.Vec3d(0,.1,.8),Gf.Vec3d(0,0,1)).GetInverse()).T;xform(light,T)
stage.SetMetadata('customLayerData',{'sceneVersion':'6','armModels':'Generic dual arm dual','handModels':'Articulated hand left / right','sensorCount':5,'sensorModel':'Generic RGBD','revision':'neutral_scene_v1','sourcePreset':poses['preset'],'units':'m','tableTopHeight':.795,'adapterThickness':.01})
# Export the composed geometry to a compact binary layer.
# One-file binary stage is also convenient for moving this scene to Isaac Sim.
stage.Flatten(addSourceFileComment=False).Export(str(ROOT/'dealer_table.usdc'));print('Saved standalone dealer_table.usdc',flush=True)

entry=Usd.Stage.CreateNew(str(ROOT/'scene.usda'));entry.GetRootLayer().subLayerPaths=['dealer_table.usdc'];entry.GetRootLayer().Save()
workcell=Usd.Stage.Open(str(ROOT/'dealer_table.usdc'));workcell.RemovePrim('/World/Props');workcell.RemovePrim('/World/Rack');workcell.Flatten(addSourceFileComment=False).Export(str(ROOT/'tasks/workcell.usdc'))
