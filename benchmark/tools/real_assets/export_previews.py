"""Export composed USD task visuals, including face-varying texture coordinates."""
import sys, json, importlib.util
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, UsdShade
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'reports/real_assets_20261031'
spec=importlib.util.spec_from_file_location('atlas_extract',ROOT/'reports/scene_views_20261031/extract_geometry.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

def main():
    report=json.loads((OUT/'integration.json').read_text())
    folder=OUT/'geometry'; folder.mkdir(exist_ok=True)
    checks=[]
    for entry in report['scenes']:
        stage=Usd.Stage.Open(str(ROOT/entry['scene']))
        assert not stage.GetCompositionErrors(),entry['id']
        cache=UsdGeom.XformCache(); plain=[]; colors=[]; textured=[]
        for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Task')):
            if p.GetTypeName() not in ('Mesh','Cube','Sphere','Cylinder','Cone','Capsule'): continue
            imageable=UsdGeom.Imageable(p)
            if imageable.ComputeVisibility()=='invisible': continue
            tri=mod.primitive(p)
            if tri is None or not len(tri): continue
            world=np.asarray(cache.GetLocalToWorldTransform(p)); tri=tri@world[:3,:3]+world[3,:3]
            material=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            shader=material.ComputeSurfaceSource()[0] if material else None
            diffuse=shader.GetInput('diffuseColor') if shader else None
            tex=None
            if diffuse and diffuse.HasConnectedSource():
                source=diffuse.GetConnectedSource()[0]; image=UsdShade.Shader(source.GetPrim()).GetInput('file').Get()
                tex=image.resolvedPath
                assert tex and Path(tex).is_file(),str(p.GetPath())
            if tex:
                uv=np.asarray(UsdGeom.PrimvarsAPI(p).GetPrimvar('st').ComputeFlattened()).reshape(-1,3,2)
                assert len(uv)==len(tri) and np.isfinite(uv).all()
                target=folder/(entry['id']+'__'+p.GetParent().GetParent().GetName()+'.npz')
                np.savez_compressed(target,triangles=tri.astype('float32'),uv=uv.astype('float32'),texture=tex)
                textured.append(target.name)
                checks.append({'scene':entry['id'],'prim':str(p.GetPath()),'triangles':len(tri),'min_world_z':float(tri[:,:,2].min()),'texture':str(Path(tex).relative_to(ROOT))})
            else:
                color=np.array([.57,.60,.65]); dc=UsdGeom.Gprim(p).GetDisplayColorAttr().Get()
                if dc: color=np.asarray(dc[0])
                if diffuse and diffuse.Get() is not None: color=np.asarray(diffuse.Get())
                n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12)
                shade=.52+.48*np.abs(n@np.array([-.35,-.45,.8216]))
                plain.append(tri); colors.append(np.clip(shade[:,None]*color*255,0,255).astype('uint8'))
        np.savez_compressed(folder/(entry['id']+'.npz'),triangles=np.concatenate(plain).astype('float32') if plain else np.zeros((0,3,3),dtype='float32'),colors=np.concatenate(colors) if colors else np.zeros((0,3),dtype='uint8'),textured=json.dumps(textured))
        print(entry['id'],len(textured),flush=True)
    (OUT/'texture_checks.json').write_text(json.dumps(checks,indent=2))
if __name__=='__main__': main()
