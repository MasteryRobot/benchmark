"""Attach scan visuals to three baseline tasks, preserving their physics prims."""
import json,sys
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'reports/real_assets_20261031'
MAP={'01_pick_spoon':'031_spoon','02_pick_knife':'032_knife','10_pick_scissors':'037_scissors'}
def main():
    catalog={a['id']:a for a in json.loads((ROOT/'assets/realistic/catalog.json').read_text())['assets']}
    records=[]
    for task,name in MAP.items():
        path=ROOT/'tasks'/task/'scene.usda';backup=OUT/'baseline_originals'/task/'scene.usda';backup.parent.mkdir(parents=True,exist_ok=True)
        if not backup.exists():backup.write_bytes(path.read_bytes())
        stage=Usd.Stage.Open(str(path));body=stage.GetPrimAtPath('/World/Task/Object')
        xf=UsdGeom.XformCache(); inv=xf.GetLocalToWorldTransform(body).GetInverse();points=[]
        for p in Usd.PrimRange(body):
            if '/RealisticVisual' in str(p.GetPath()):continue
            if not p.IsA(UsdGeom.Gprim):continue
            bound=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False,True).ComputeRelativeBound(p,body).ComputeAlignedRange()
            if not bound.IsEmpty():points.extend([list(bound.GetMin()),list(bound.GetMax())])
        points=np.asarray(points);lo=points.min(0);hi=points.max(0);asset=catalog[name];size=np.array(asset['size_m'])
        scale=float(np.min((hi-lo)[:2]/size[:2]));center=(lo+hi)/2;center[2]=lo[2]+size[2]*scale/2
        for p in Usd.PrimRange(body):
            if '/RealisticVisual' not in str(p.GetPath()) and p.IsA(UsdGeom.Gprim):UsdGeom.Imageable(p).CreateVisibilityAttr('invisible')
        vis=UsdGeom.Xform.Define(stage,'/World/Task/Object/RealisticVisual');vis.GetPrim().GetReferences().ClearReferences();vis.GetPrim().GetReferences().AddReference('../../'+asset['usd'],'/Asset');vis.ClearXformOpOrder()
        vis.AddTranslateOp().Set(Gf.Vec3d(*center));vis.AddScaleOp().Set(Gf.Vec3f(*([scale]*3)));stage.GetRootLayer().Save()
        records.append({'id':task,'scene':str(path.relative_to(ROOT)),'title':json.loads(path.with_name('task.json').read_text())['title'],'bindings':[{'body_id':'Object','asset_id':name,'usd':asset['usd'],'scale':scale,'translate':center.tolist(),'source':asset['url'],'license':'CC-BY-4.0','physics_status':'Original colliders, mass, poses and grasp reference unchanged; scan/proxy fidelity not validated.'}]})
        print(task,name,flush=True)
    (OUT/'baseline_integration.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
