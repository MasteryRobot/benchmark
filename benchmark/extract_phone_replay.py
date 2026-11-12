"""Export scene meshes into rigid-body coordinates for measured-state replay."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics
from reports.scene_views_20261031.extract_geometry import primitive

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
stage=Usd.Stage.Open('tasks/03_pick_phone/scene.usda');cache=UsdGeom.XformCache()
groups={};cameras={};reference={}
for prim in stage.Traverse():
    path=str(prim.GetPath())
    if prim.IsA(UsdGeom.Camera):
        cam=UsdGeom.Camera(prim);owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        world=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        body=str(owner.GetPath()) if owner else ''
        base=np.asarray(cache.GetLocalToWorldTransform(owner)).T if owner else np.eye(4)
        cameras[path.split('/')[-2].lower()]={'world':world.tolist(),'body':body,'local':(np.linalg.inv(base)@world).tolist(),'focal':cam.GetFocalLengthAttr().Get(),'ha':cam.GetHorizontalApertureAttr().Get()}
    if '/collision/' in path or path.startswith('/World/Task/Goal'):continue
    if prim.GetTypeName() not in ('Mesh','Cube','Sphere','Cylinder','Cone','Capsule'):continue
    im=UsdGeom.Imageable(prim)
    if im.ComputeVisibility()==UsdGeom.Tokens.invisible or im.ComputePurpose() not in ('default','render'):continue
    tris=primitive(prim)
    if tris is None or not len(tris):continue
    owner=prim
    while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
    body=str(owner.GetPath()) if owner else ''
    base=np.asarray(cache.GetLocalToWorldTransform(owner)).T if owner else np.eye(4)
    matrix=np.linalg.inv(base)@np.asarray(cache.GetLocalToWorldTransform(prim)).T
    tris=tris@matrix[:3,:3].T+matrix[:3,3]
    if len(tris)>3000:
        tris=np.round(tris/.0018)*.0018
        tris=tris[np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)>1e-10]
        _,idx=np.unique(tris.reshape(-1,9),axis=0,return_index=True)
        tris=tris[np.sort(idx)]
    color=np.array([.57,.60,.65]);dc=UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
    if dc:color=np.asarray(dc[0])
    material=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if material:
        shader=material.ComputeSurfaceSource()[0]
        if shader:
            for key in ('diffuseColor','diffuse_color_constant','base_color'):
                attr=shader.GetInput(key)
                if attr and attr.Get() is not None:color=np.asarray(attr.Get())[:3];break
    groups.setdefault(body,[[],[]]);groups[body][0].append(tris.astype('float32'))
    groups[body][1].append(np.tile(color,(len(tris),1)).astype('float32'));reference[body]=base.tolist()
out={'names':np.array(list(groups)),'reference':json.dumps(reference),'cameras':json.dumps(cameras)}
for i,(name,(tri,col)) in enumerate(groups.items()):
    out['tri_'+str(i)]=np.concatenate(tri);out['col_'+str(i)]=np.concatenate(col)
a.output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(a.output,**out)
print(json.dumps({'groups':len(groups),'triangles':sum(len(v) for k,v in out.items() if k.startswith('tri_')),'output':str(a.output)}))
