"""Depth-tested visibility of a 2 cm tabletop grid and chopstick centerlines."""
from pathlib import Path
import json,math
import numpy as np
from shapely.geometry import Polygon,Point
ROOT=Path(__file__).resolve().parent
cameras=json.loads((ROOT/'cameras.json').read_text())
outer=[(1.04*math.cos(t),-.22+1.10*math.sin(t)) for t in np.linspace(0,math.pi,181)]
notch=[(.40*math.cos(t),-.22+.36*math.sin(t)) for t in np.linspace(math.pi,0,81)]
poly=Polygon(outer+notch).buffer(.025,quad_segs=12).buffer(-.061,quad_segs=12)
points=np.array([[x,y,.7955] for x in np.arange(-1.04,1.05,.02) for y in np.arange(-.22,.89,.02) if poly.contains(Point(x,y))])
rodpts=np.array([[x,y,.8223] for x in [-.021,.021] for y in np.linspace(.30,.56,31)])
def visible(spec,points,depth):
 T=np.array(spec['T_world_camera_opencv']);cam=(points-T[:3,3])@T[:3,:3];K=np.array(spec['K']);pixels=cam@K.T;uv=pixels[:,:2]/pixels[:,2:];ix=np.rint(uv).astype(int);h,w=depth.shape
 mask=(cam[:,2]>.02)&(ix[:,0]>=0)&(ix[:,0]<w)&(ix[:,1]>=0)&(ix[:,1]<h);obs=np.full(len(points),np.inf);obs[mask]=depth[ix[mask,1],ix[mask,0]];dist=np.linalg.norm(cam,axis=1)
 return mask&(np.abs(obs-dist)<.02),mask
vis={};result={}
for name,spec in cameras.items():
 depth=np.load(ROOT/'images'/(name.lower()+'_depth.npz'))['distance_m'];v,frustum=visible(spec,points,depth);r,_=visible(spec,rodpts,depth);vis[name]=v;result[name]={'tabletop_visible_fraction':float(v.mean()),'tabletop_in_frame_fraction':float(frustum.mean()),'chopstick_samples_visible_fraction':float(r.mean())}
allvis=np.array(list(vis.values()));globalvis=np.array([vis[k] for k in ['Head','LeftSide','RightSide']]);result['combined']={'grid_points':len(points),'grid_spacing_m':.02,'depth_tolerance_m':.02,'tabletop_visible_fraction':float(allvis.any(axis=0).mean()),'global_three_visible_fraction':float(globalvis.any(axis=0).mean()),'seen_by_at_least_two':float((allvis.sum(axis=0)>=2).mean()),'scope':'Current table-ready pose only; 2 cm sampling, 2 cm depth tolerance; objects/arms can occlude tabletop.'}
(ROOT/'camera_coverage.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
