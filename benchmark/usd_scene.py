"""USD geometry, materials and native articulation authoring helpers."""
from pathlib import Path
import re
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from shapely.ops import triangulate
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Gf,Sdf
from robot_model import origin

def vec(a):return Gf.Vec3f(*map(float,a))
def quat(R):
 q=Rotation.from_matrix(R).as_quat();return Gf.Quatf(float(q[3]),vec(q[:3]))
def xform(obj,T):
 x=UsdGeom.Xformable(obj);x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(*map(float,T[:3,3])));q=Rotation.from_matrix(T[:3,:3]).as_quat();x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*q[:3])))
def add_api(prim,schema):
 ap=list(prim.GetAppliedSchemas());ap.append(schema);prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(list(dict.fromkeys(ap))))
def attr(prim,name,value,typ=Sdf.ValueTypeNames.Float):prim.CreateAttribute(name,typ,custom=False).Set(value)
class Scene:
 def __init__(self,stage,material_prefix='/World/Looks'):self.s=stage;self.materials={};self.material_prefix=material_prefix
 def material(self,color,metal=.0,rough=.5):
  key=tuple(np.round(color,3))+(metal,rough)
  if key in self.materials:return self.materials[key]
  p=f'{self.material_prefix}/M{len(self.materials)}';m=UsdShade.Material.Define(self.s,p);sh=UsdShade.Shader.Define(self.s,p+'/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(vec(color));sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal);sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');self.materials[key]=m;return m
 def finish(self,obj,color,coll=False,metal=0,rough=.5):
  obj.CreateDisplayColorAttr([vec(color)]);UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(self.material(color,metal,rough))
  if coll:
   UsdPhysics.CollisionAPI.Apply(obj.GetPrim());add_api(obj.GetPrim(),'PhysxCollisionAPI');attr(obj.GetPrim(),'physxCollision:contactOffset',.001);attr(obj.GetPrim(),'physxCollision:restOffset',0)
  return obj
 def cube(self,path,pos,size,color,coll=True,metal=0,rough=.5):
  c=UsdGeom.Cube.Define(self.s,path);c.CreateSizeAttr(1);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(vec(size));return self.finish(c,color,coll,metal,rough)
 def cylinder(self,path,pos,radius,height,color,axis='Z',coll=True,metal=.0,T=None):
  c=UsdGeom.Cylinder.Define(self.s,path);c.CreateAxisAttr(axis);c.CreateRadiusAttr(radius);c.CreateHeightAttr(height)
  if T is not None:xform(c,T)
  else:c.AddTranslateOp().Set(Gf.Vec3d(*pos))
  return self.finish(c,color,coll,metal)
 def mesh(self,path,vertices,faces,color,coll=False,approx='none',normals=None,metal=0,rough=.5):
  c=UsdGeom.Mesh.Define(self.s,path);c.CreatePointsAttr(np.asarray(vertices,dtype=np.float32));c.CreateFaceVertexCountsAttr([3]*len(faces));c.CreateFaceVertexIndicesAttr(np.asarray(faces,dtype=np.int32).ravel());c.CreateSubdivisionSchemeAttr('none')
  if normals is not None:c.CreateNormalsAttr(np.asarray(normals,dtype=np.float32));c.SetNormalsInterpolation('vertex')
  self.finish(c,color,coll,metal,rough)
  if coll:UsdPhysics.MeshCollisionAPI.Apply(c.GetPrim()).CreateApproximationAttr(approx)
  return c
 def extrusion(self,path,polygon,z0,z1,color,coll=True,metal=0,rough=.5):
  pts=[];faces=[]
  def tri(v):
   k=len(pts);pts.extend(v);faces.append([k,k+1,k+2])
  for t in triangulate(polygon):
   if not polygon.covers(t.representative_point()):continue
   xy=list(t.exterior.coords)[:3]
   tri([(x,y,z1) for x,y in xy]);tri([(x,y,z0) for x,y in reversed(xy)])
  for ring in [polygon.exterior]+list(polygon.interiors):
   xy=list(ring.coords)
   for (x,y),(a,b) in zip(xy[:-1],xy[1:]):tri([(x,y,z0),(a,b,z0),(a,b,z1)]);tri([(x,y,z0),(a,b,z1),(x,y,z1)])
  return self.mesh(path,pts,faces,color,coll,metal=metal,rough=rough)
 def load_geometry(self,path,node,folder,T,color=None,collision=False):
  g=node.find('geometry');mesh=g.find('mesh')
  if mesh is not None:
   asset=folder/mesh.get('filename');scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
   scene=trimesh.load(asset,force='scene',process=False)
   for i,n in enumerate(scene.graph.nodes_geometry):
    mat,name=scene.graph[n];m=scene.geometry[name].copy();m.apply_transform(mat);m.vertices*=scale;m.apply_transform(T)
    col=color
    if col is None:
     material=getattr(m.visual,'material',None);c=getattr(material,'diffuse',None)
     col=(np.array(c)[:3]/255.).tolist() if c is not None else [.62,.64,.68]
    obj=self.mesh(path+f'/mesh{i}',m.vertices,m.faces,col,collision,'convexHull',None if collision else m.vertex_normals,metal=.15 if color is None else .05,rough=.37 if color is None else .5)
    if collision:obj.CreatePurposeAttr('guide');obj.CreateVisibilityAttr('invisible')
  elif g.find('cylinder') is not None:
   c=g.find('cylinder');obj=self.cylinder(path+'/cylinder',(0,0,0),float(c.get('radius')),float(c.get('length')),color or [.2,.23,.27],coll=collision,metal=.6,T=T)
   if collision:obj.CreateVisibilityAttr('invisible')
  else:raise ValueError('Unsupported geometry')
 def robot(self,path,robot,q,base):
  UsdGeom.Xform.Define(self.s,path)
  zero=robot.fk();pose=robot.fk(q,base);groups=sorted(set(robot.groups.values()));paths={g:path+'/Links/'+g for g in groups}
  for g in groups:
   b=UsdGeom.Xform.Define(self.s,paths[g]);xform(b,pose[g]);UsdPhysics.RigidBodyAPI.Apply(b.GetPrim());ma=UsdPhysics.MassAPI.Apply(b.GetPrim())
   components=[]
   for name,link in robot.links.items():
    if robot.groups[name]!=g:continue
    T=np.linalg.inv(zero[g])@zero[name];i=link.find('inertial')
    if i is not None:
     it=T@origin(i.find('origin'));mass=float(i.find('mass').get('value'));vals=i.find('inertia');I=np.array([[float(vals.get('i'+a+b,vals.get('i'+b+a,'0'))) for b in 'xyz'] for a in 'xyz']);components.append((mass,it[:3,3],it[:3,:3]@I@it[:3,:3].T))
    for typ in ['visual','collision']:
     for k,node in enumerate(link.findall(typ)):
      c=None
      material=node.find('material')
      if material is not None:
       color_node=material.find('color')
       if color_node is None:
        global_material=robot.xml.find("material[@name='%s']"%material.get('name',''))
        if global_material is not None:color_node=global_material.find('color')
       if color_node is not None:c=np.fromstring(color_node.get('rgba'),sep=' ')[:3].tolist()
      self.load_geometry(paths[g]+'/'+typ+'/'+name+str(k),node,robot.filename.parent,T@origin(node.find('origin')),c,typ=='collision')
   mass=sum(c[0] for c in components);com=sum(m*p for m,p,I in components)/mass
   I=sum(I+m*(np.dot(p-com,p-com)*np.eye(3)-np.outer(p-com,p-com)) for m,p,I in components);vals,axes=np.linalg.eigh(I)
   if np.linalg.det(axes)<0:axes[:,0]*=-1
   ma.CreateMassAttr(float(mass));ma.CreateCenterOfMassAttr(vec(com));ma.CreateDiagonalInertiaAttr(vec(np.maximum(vals,1e-8)));ma.CreatePrincipalAxesAttr(quat(axes))
   add_api(b.GetPrim(),'PhysxRigidBodyAPI');attr(b.GetPrim(),'physxRigidBody:disableGravity',True,Sdf.ValueTypeNames.Bool)
  fix=UsdPhysics.FixedJoint.Define(self.s,path+'/Joints/FixedBase');fix.CreateBody1Rel().SetTargets([paths[robot.root]]);fix.CreateLocalPos0Attr(vec(base[:3,3]));fix.CreateLocalRot0Attr(quat(base[:3,:3]));UsdPhysics.ArticulationRootAPI.Apply(fix.GetPrim());add_api(fix.GetPrim(),'PhysxArticulationAPI');attr(fix.GetPrim(),'physxArticulation:enabledSelfCollisions',False,Sdf.ValueTypeNames.Bool);attr(fix.GetPrim(),'physxArticulation:solverPositionIterationCount',32,Sdf.ValueTypeNames.Int);attr(fix.GetPrim(),'physxArticulation:solverVelocityIterationCount',4,Sdf.ValueTypeNames.Int)
  for j in robot.revolute:
   name=j.get('name');parent=j.find('parent').get('link');child=j.find('child').get('link');gp=robot.groups[parent];gc=robot.groups[child]
   A=np.fromstring(j.find('axis').get('xyz'),sep=' ');aR=Rotation.align_vectors([A],[[0,0,1]])[0].as_matrix();jt=np.linalg.inv(zero[gp])@zero[parent]@origin(j.find('origin'));ct=np.linalg.inv(zero[gc])@zero[child]
   joint=UsdPhysics.RevoluteJoint.Define(self.s,path+'/Joints/'+name);joint.CreateBody0Rel().SetTargets([paths[gp]]);joint.CreateBody1Rel().SetTargets([paths[gc]]);joint.CreateAxisAttr('Z');joint.CreateLocalPos0Attr(vec(jt[:3,3]));joint.CreateLocalRot0Attr(quat(jt[:3,:3]@aR));joint.CreateLocalPos1Attr(vec(ct[:3,3]));joint.CreateLocalRot1Attr(quat(ct[:3,:3]@aR));lim=j.find('limit');joint.CreateLowerLimitAttr(np.rad2deg(float(lim.get('lower'))));joint.CreateUpperLimitAttr(np.rad2deg(float(lim.get('upper'))));joint.CreateCollisionEnabledAttr(False)
   arm=name.lower().startswith('joint');d=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(float((5500 if arm else 18)*np.pi/180));d.CreateDampingAttr(float((180 if arm else .6)*np.pi/180));d.CreateMaxForceAttr(float(lim.get('effort')));d.CreateTargetPositionAttr(float(np.rad2deg(q.get(name,0))))
   add_api(joint.GetPrim(),'PhysicsJointStateAPI:angular');attr(joint.GetPrim(),'state:angular:physics:position',float(np.rad2deg(q.get(name,0))));attr(joint.GetPrim(),'state:angular:physics:velocity',0)
  return pose
