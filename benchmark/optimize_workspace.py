"""Sample bounded IK reach at tabletop height for candidate whole-robot translations.
Conservative link-centerline and pedestal surface clearance filters, not a motion planner.
"""
from pathlib import Path
import json,math
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from shapely.geometry import Polygon
from shapely import contains_xy
from robot_model import Robot,origin,transform
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'tasks';OUT.mkdir(exist_ok=True)
old=json.loads((ROOT/'tasks/baseline_poses.json').read_text());cfg=old['robots']['DualArm'];robot=Robot(ROOT/cfg['urdf']);T0=np.array(cfg['base_transform']);f0=robot.fk({},T0)
outer=[(1.04*np.cos(t),-.22+1.10*np.sin(t)) for t in np.linspace(0,np.pi,181)];notch=[(.40*np.cos(t),-.22+.36*np.sin(t)) for t in np.linspace(np.pi,0,81)]
poly=Polygon(outer+notch).buffer(.025).buffer(-.061)
xy=np.array([(x,y) for x in np.arange(-.95,.951,.05) for y in np.arange(-.15,.851,.05) if poly.contains(__import__('shapely').geometry.Point(x,y))]);targets=np.c_[xy,np.full(len(xy),.86)]
Rdown=np.array([[0,-1,0],[0,0,1],[-1,0,0]])
rng=np.random.default_rng(123)
rootmesh=trimesh.load(ROOT/'assets/dual_arm/meshes/pedestal.stl');rootmesh.apply_transform(T0);surface,_=trimesh.sample.sample_surface(rootmesh,40000,seed=123)

def obstacle(points,margin=0.):
 p=np.asarray(points);x,y,z=p.T
 slab=contains_xy(Polygon(outer+notch).buffer(margin),x,y)&(z>.724-margin)&(z<.799+margin)
 bars=np.zeros(len(p),bool)
 for yy,ww in [(.04,1.52),(.63,1.10)]:bars|=(abs(x)<ww/2+margin)&(abs(y-yy)<.0175+margin)&(abs(z-.69)<.020+margin)
 legs=np.zeros(len(p),bool)
 for xx,yy in [(-.76,.04),(.76,.04),(-.55,.63),(.55,.63)]:legs|=((x-xx)**2+(y-yy)**2<(.035+margin)**2)&(z>.01)&(z<.73+margin)
 return slab|bars|legs

class Arm:
 def __init__(self,side,chain,dy):
  self.side=side;self.chain=chain;self.joints=[next(j for j in robot.revolute if j.get('name')==f'Joint{k}_{chain}') for k in range(1,8)];self.names=[j.get('name') for j in self.joints];self.O=np.array([origin(j.find('origin')) for j in self.joints]);self.base=f0['Base_'+chain].copy();self.base[1,3]+=dy
  self.tip=np.linalg.inv(f0['Link7_'+chain])@f0[side+'_hand_base']@transform([.02,0,.10]);self.lo=np.array([float(j.find('limit').get('lower'))+.02 for j in self.joints]);self.hi=np.array([float(j.find('limit').get('upper'))-.02 for j in self.joints]);self.ready=np.array([cfg['joint_positions'][n] for n in self.names])
 def fk(self,q):
  n=len(q);T=np.broadcast_to(self.base,(n,4,4)).copy();P=[];A=[]
  for i in range(7):
   T=T@self.O[i];P.append(T[:,:3,3].copy());A.append(T[:,:3,2].copy());c=np.cos(q[:,i]);s=np.sin(q[:,i]);R=np.tile(np.eye(4),(n,1,1));R[:,0,0]=c;R[:,1,1]=c;R[:,0,1]=-s;R[:,1,0]=s;T=T@R
  return T@self.tip,np.stack(P,axis=1),np.stack(A,axis=1)
 def solve(self,target,Rgoal=Rdown,seeds=3):
  n=len(target);success=np.zeros(n,bool);solutions=np.zeros((n,7));besterr=np.full(n,np.inf)
  for trial in range(seeds):
   q=np.tile(self.ready,(n,1)) if trial==0 else rng.uniform(self.lo,self.hi,(n,7))
   q=np.clip(q,self.lo,self.hi)
   for _ in range(100):
    h,P,A=self.fk(q);pe=target-h[:,:3,3];re=Rotation.from_matrix(Rgoal@np.transpose(h[:,:3,:3],(0,2,1))).as_rotvec();e=np.c_[pe,re*.20]
    J=np.concatenate([np.cross(A,h[:,:3,3][:,None,:]-P).transpose(0,2,1),.20*A.transpose(0,2,1)],axis=1)
    delta=(J.transpose(0,2,1)@np.linalg.solve(J@J.transpose(0,2,1)+.0004*np.eye(6),e[...,None])).squeeze(-1);q=np.clip(q+np.clip(delta,-.18,.18),self.lo,self.hi)
   h,P,A=self.fk(q);pe=np.linalg.norm(target-h[:,:3,3],axis=1);re=Rotation.from_matrix(Rgoal@h[:,:3,:3].transpose(0,2,1)).magnitude()
   # Upper-arm/forearm centerlines with 55 mm table-clearance envelope.
   samples=np.concatenate([P[:,k:k+1]*(1-t)+P[:,k+1:k+2]*t for k in range(1,6) for t in np.linspace(0,1,5)],axis=1)
   collision=obstacle(samples.reshape(-1,3),.055).reshape(n,-1).any(axis=1)
   side=-1 if self.side=='left' else 1
   crossing=(P[:,2:5,0]*side<.08).any(axis=1)
   ok=(pe<.008)&(re<np.deg2rad(8))&~collision&~crossing
   new=ok&(~success | (pe<besterr));solutions[new]=q[new];besterr[new]=pe[new];success|=ok
  return success,solutions

if __name__=='__main__':
 results=[];maps={}
 for dy in [*np.arange(0,.361,.06),.37,.38,.39,.40,.42]:
  hits=obstacle(surface+[0,dy,0],.012);valid=not hits.any()
  row={'forward_translation_m':round(float(dy),3),'shoulder_mid_y_m':round(-.43+float(dy),3),'pedestal_clearance_pass':valid,'pedestal_collision_samples':int(hits.sum())}
  if valid:
   masks=[]
   for side,ch in [('left','R'),('right','L')]:
    arm=Arm(side,ch,dy);mask,sol=arm.solve(targets);masks.append(mask)
   union=masks[0]|masks[1];both=masks[0]&masks[1];row.update(left=int(masks[0].sum()),right=int(masks[1].sum()),either=int(union.sum()),both=int(both.sum()),either_fraction=float(union.mean()),both_fraction=float(both.mean()))
   maps[str(row['forward_translation_m'])]=[m.tolist() for m in masks]
  results.append(row);print(row,flush=True)
 valid=[r for r in results if r['pedestal_clearance_pass']];best=max(valid,key=lambda r:(r['either']+.5*r['both'],-r['forward_translation_m']))
 report={'method':'Bounded 7-DOF IK, palm down, grasp reference hand+[0.02,0,0.10] m at z=0.86 m; 3 seeds, 100 DLS iterations; 8 mm position / 8 deg rotation tolerance. 55 mm arm centerline/table clearance proxy, no centerline crossing. 40k CAD pedestal samples with 12 mm furniture clearance. Not full-mesh swept-motion collision verification or global optimization.','grid_spacing_m':.05,'grid_count':len(targets),'candidates':results,'selected':best,'baseline':results[0],'xy':xy.tolist(),'masks':maps}
 (OUT/'reachability.json').write_text(json.dumps(report,indent=2));print('SELECTED',best,flush=True)
