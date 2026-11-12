"""Parametric tabletop task props, in metres. Explicit task-design geometry, not vendor CAD."""
import math
import numpy as np
import trimesh
from shapely.geometry import Point,Polygon,box
from pxr import UsdGeom,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
from usd_scene import xform
from robot_model import transform

SILVER=[.62,.67,.72];DARK=[.025,.035,.045]

def rounded(a,path,w,h,z0,z1,r,color,coll=True,metal=0):
 p=box(-w/2+r,-h/2+r,w/2-r,h/2-r).buffer(r,quad_segs=12)
 o=a.extrusion(path,p,z0,z1,color,coll,metal=metal,rough=.3)
 if coll:UsdPhysics.MeshCollisionAPI.Apply(o.GetPrim()).CreateApproximationAttr('convexHull')
 return o

def ellipsoid(a,path,pos,scale,color,coll=True,metal=0):
 m=trimesh.creation.icosphere(subdivisions=3);m.vertices*=scale;m.vertices+=pos
 return a.mesh(path,m.vertices,m.faces,color,coll,'convexHull',m.vertex_normals,metal=metal,rough=.4)

def rod(a,path,p0,p1,r,color,coll=True,metal=0):
 p0,p1=np.array(p0),np.array(p1);d=p1-p0;T=np.eye(4);T[:3,3]=(p0+p1)/2;T[:3,:3]=Rotation.align_vectors([d/np.linalg.norm(d)],[[0,0,1]])[0].as_matrix()
 return a.cylinder(path,(0,0,0),r,float(np.linalg.norm(d)),color,coll=coll,metal=metal,T=T)

def spoon(a,p):
 # Closed thin shell: elliptical concave bowl, 1.3 mm steel thickness.
 n=64;nr=12;vs=[]
 for layer in [0,1]:
  for r in np.linspace(.001,1,nr):
   for t in np.linspace(0,2*np.pi,n,endpoint=False):vs.append([.020*r*np.cos(t),.054+.029*r*np.sin(t),.010*r*r-layer*.0013])
 fs=[]
 for layer in [0,1]:
  off=layer*nr*n
  for j in range(nr-1):
   for k in range(n):
    u=off+j*n+k;v=off+j*n+(k+1)%n;fs.extend([[u,v,v+n],[u,v+n,u+n]] if layer==0 else [[u,v+n,v],[u,u+n,v+n]])
 for k in range(n):
  u=(nr-1)*n+k;v=(nr-1)*n+(k+1)%n;fs.extend([[u,v,u+nr*n],[v,v+nr*n,u+nr*n]])
 m=trimesh.Trimesh(vs,fs,process=False);a.mesh(p+'/Bowl',m.vertices,m.faces,SILVER,True,'convexHull',m.vertex_normals,metal=.9,rough=.2)
 shape=Polygon([(-.008,-.087),(.008,-.087),(.006,-.05),(.004,.022),(.008,.033),(-.008,.033),(-.004,.022),(-.006,-.05)]).buffer(.002)
 o=a.extrusion(p+'/Handle',shape,.006,.009,SILVER,True,metal=.9,rough=.2);UsdPhysics.MeshCollisionAPI.Apply(o.GetPrim()).CreateApproximationAttr('convexHull')
 return {'mass':.035,'size':[.040,.174,.012],'long_axis':'Y','grasp_local':[0,-.043,.008]}

def knife(a,p):
 shape=Polygon([(-.014,-.004),(.013,-.004),(.013,.087),(.004,.115),(-.007,.095),(-.014,.052)])
 o=a.extrusion(p+'/Blade',shape,-.0008,.0008,[.72,.76,.80],True,metal=.95,rough=.17);UsdPhysics.MeshCollisionAPI.Apply(o.GetPrim()).CreateApproximationAttr('convexHull')
 handle=rounded(a,p+'/Handle',.026,.091,-.008,.008,.008,[.055,.065,.080]);handle.AddTranslateOp().Set(Gf.Vec3d(0,-.047,0))
 for j,y in enumerate([-.075,-.045,-.018]):a.cylinder(p+'/Rivet'+str(j),(0,y,.0083),.0028,.001,SILVER,coll=False,metal=.85)
 return {'mass':.095,'size':[.030,.211,.016],'long_axis':'Y','grasp_local':[0,-.045,0]}

def phone(a,p):
 rounded(a,p+'/Frame',.073,.151,-.0042,.0042,.009,[.30,.34,.39],metal=.8)
 screen=rounded(a,p+'/Screen',.068,.144,.0043,.0046,.007,[.017,.041,.075],False)
 rounded(a,p+'/Back',.070,.147,-.0046,-.0043,.008,[.075,.105,.135],False)
 for row in range(3):
  for col in range(4):
   item=rounded(a,p+f'/App{row}_{col}',.010,.010,.00465,.0048,.0025,[[.14,.47,.77],[.19,.66,.49],[.91,.52,.18],[.58,.33,.74]][col],False);item.AddTranslateOp().Set(Gf.Vec3d((col-1.5)*.014,-.036+row*.017,0))
 for j in range(3):a.cube(p+'/ScreenLine'+str(j),[0,.025+j*.008,.0048],[.040-j*.008,.002,.0001],[.64,.77,.86],coll=False)
 e=rounded(a,p+'/Earpiece',.016,.002,.0047,.0049,.0008,[.015,.018,.02],False);e.AddTranslateOp().Set(Gf.Vec3d(0,.065,0))
 for j,(x,y) in enumerate([(-.021,.054),(-.021,.037),(-.006,.046)]):
  a.cylinder(p+'/RearLensRing'+str(j),(x,y,-.0053),.006,.0017,[.18,.21,.25],coll=False,metal=.9)
  a.cylinder(p+'/RearLens'+str(j),(x,y,-.0062),.0045,.0003,[.009,.025,.035],coll=False,metal=.5)
 a.cube(p+'/PowerButton',[.037, .025,0],[.001,.016,.003],SILVER,coll=False,metal=.8)
 return {'mass':.185,'size':[.075,.151,.011],'long_axis':'Y','screen_normal':'+Z','grasp_local':[0,0,0]}

def battery(a,p):
 a.cylinder(p+'/Cell',(0,0,0),.0072,.047,[.045,.065,.09],metal=.35)
 a.cylinder(p+'/CopperPositiveBand',(0,0,.018),.00725,.012,[.70,.30,.10],metal=.7)
 a.cylinder(p+'/NegativeTerminal',(0,0,-.024),.0068,.001,SILVER,metal=.8)
 a.cylinder(p+'/PositiveTerminal',(0,0,.025),.0025,.002,SILVER,metal=.8)
 a.cube(p+'/Plus1',[0,-.0072,.017],[.005,.00025,.001],[.92,.92,.84],coll=False)
 a.cube(p+'/Plus2',[0,-.0072,.017],[.001,.00025,.005],[.92,.92,.84],coll=False)
 return {'mass':.023,'size':[.0145,.0145,.051],'long_axis':'Z','positive_axis':'+Z','grasp_local':[0,0,0]}

def lighter(a,p):
 rounded(a,p+'/Body',.025,.015,-.032,.018,.006,[.065,.33,.68])
 rounded(a,p+'/SteelHood',.025,.015,.018,.031,.004,SILVER,metal=.85)
 a.cylinder(p+'/Striker',(0,-.005,.031),.0043,.017,DARK,axis='X',metal=.8)
 a.cube(p+'/GasLever',[0,-.008,.022],[.016,.004,.006],[.78,.12,.09])
 for i,x in enumerate([-.007,0,.007]):a.cube(p+'/Vent'+str(i),[x,.0076,.025],[.003,.0002,.005],DARK,coll=False)
 a.cube(p+'/OrientationStripe',[.006,-.0076,-.005],[.003,.0002,.029],[.92,.94,.95],coll=False)
 return {'mass':.020,'size':[.025,.020,.067],'long_axis':'Z','use_axis':'+Z; striker upward, gas lever toward robot','grasp_local':[0,0,-.008]}

def lipstick(a,p):
 a.cylinder(p+'/Tube',(0,0,-.013),.009,.045,[.025,.022,.03],metal=.45)
 a.cylinder(p+'/GoldBand',(0,0,.007),.0093,.004,[.75,.52,.18],metal=.85)
 a.cylinder(p+'/Collar',(0,0,.015),.0068,.013,[.71,.49,.20],metal=.75)
 # Slanted bullet end: closed smooth cylindrical wall, angled top surface.
 n=64;v=[]
 for z in [.014,.034]:
  for t in np.linspace(0,2*np.pi,n,endpoint=False):
   x=.0058*np.cos(t);v.append([x,.0058*np.sin(t),z+(x*.55 if z>.02 else 0)])
 faces=[]
 for k in range(n):j=(k+1)%n;faces.extend([[k,j,n+j],[k,n+j,n+k]])
 for k in range(1,n-1):faces.extend([[0,k+1,k],[n,n+k,n+k+1]])
 a.mesh(p+'/Bullet',v,faces,[.62,.035,.11],True,'convexHull',metal=.05,rough=.32)
 return {'mass':.028,'size':[.019,.019,.073],'long_axis':'Z','grasp_local':[0,0,-.015]}

def swab(a,p):
 rod(a,p+'/Stem',[0,-.030,0],[0,.030,0],.00125,[.14,.51,.72])
 for j,y in enumerate([-.030,.030]):
  # Slightly flattened cotton represents the compressed contact patch at rest.
  m=trimesh.creation.icosphere(subdivisions=3);m.vertices*=np.array([.0027,.0075,.0027]);m.vertices[:,2]=np.maximum(m.vertices[:,2],-.0021);m.vertices[:,1]+=y
  a.mesh(p+'/Cotton'+str(j),m.vertices,m.faces,[.92,.91,.86],False,normals=m.vertex_normals,rough=.7)
  c=a.cube(p+'/CottonContact'+str(j),[0,y,0],[.0038,.010,.0042],[.92,.91,.86]);c.CreateVisibilityAttr('invisible')
 # Small one-sided stripe makes a roll about the long axis observable.
 a.cube(p+'/RollMarker',[0,0,.00128],[.001,.014,.0001],[.93,.23,.16],coll=False)
 return {'mass':.0005,'size':[.0054,.075,.0054],'long_axis':'Y','roll_marker':'+Z red stripe','grasp_local':[0,0,0],'contact_model':'compressed cotton pads approximated by two small box colliders; rigid shaft'}

def scissors(a,p):
 # Closed scissors; handle rings remain hollow (individual segment colliders).
 for side in [-1,1]:
  cx=side*.018;cy=-.045;rx=.014;ry=.021
  points=np.array([[cx+rx*np.cos(t),cy+ry*np.sin(t),0] for t in np.linspace(0,2*np.pi,25)])
  for k in range(24):rod(a,p+f'/Handle{side+1}_{k}',points[k],points[k+1],.004,[.12,.34,.42],False)
  # Box compound follows the open ring, avoiding pathological native-cylinder CCD contacts.
  for k in range(8):
   p0=points[k*3];p1=points[(k+1)*3];d=p1-p0;mid=(p0+p1)/2
   c=a.cube(p+f'/RingCollision{side+1}_{k}',[0,0,0],[1,1,1],[.12,.34,.42])
   M=transform(mid,[0,0,np.arctan2(d[1],d[0])]);xform(c,M);c.AddScaleOp().Set(Gf.Vec3f(float(np.linalg.norm(d)+.003),.008,.008));c.CreateVisibilityAttr('invisible')
  shape=Polygon([(side*.010,-.029),(side*.004,.006),(side*.002,.092),(side*.012,.061),(side*.016,.012)])
  o=a.extrusion(p+f'/Blade{side+1}',shape,side*.001-.001,side*.001+.001,SILVER,True,metal=.85,rough=.2);UsdPhysics.MeshCollisionAPI.Apply(o.GetPrim()).CreateApproximationAttr('convexHull')
 a.cylinder(p+'/Pivot',(0,.006,.003),.006,.004,[.28,.32,.36],metal=.8)
 return {'mass':.055,'size':[.072,.162,.009],'long_axis':'Y','grasp_local':[0,-.045,0],'state':'closed; rigid pickup prop, blade hinge not actuated'}

BUILDERS={'spoon':spoon,'knife':knife,'phone':phone,'battery':battery,'lighter':lighter,'lipstick':lipstick,'swab':swab,'scissors':scissors}
