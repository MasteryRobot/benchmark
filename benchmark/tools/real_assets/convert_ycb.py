"""Convert YCB textured OBJ scans to portable USD visuals without physics schemas."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf,Vt
ROOT=Path(__file__).resolve().parents[2];SOURCE=ROOT/'assets/external/ycb';OUT=ROOT/'assets/realistic'
TOOLS={'030_fork','031_spoon','032_knife','033_spatula','043_phillips_screwdriver','048_hammer','037_scissors','040_large_marker'}
def read_obj(path):
 v=[];uv=[];faces=[];st=[]
 for line in path.read_text().splitlines():
  a=line.split()
  if not a:continue
  if a[0]=='v':v.append(list(map(float,a[1:4])))
  elif a[0]=='vt':uv.append(list(map(float,a[1:3])))
  elif a[0]=='f':
   corners=[q.split('/') for q in a[1:]]
   for j in range(1,len(corners)-1):
    tri=[corners[k] for k in (0,j,j+1)];faces.append([int(c[0])-1 for c in tri]);st.append([int(c[1])-1 for c in tri])
 return np.array(v),np.array(faces),np.array(uv)[np.array(st)]
def main():
 OUT.mkdir(exist_ok=True);records=[]
 sources=json.loads((SOURCE/'sources.json').read_text())['assets']
 for source in sources:
  name=source['id'];obj=ROOT/source['obj'];v,f,uv=read_obj(obj);R=np.eye(3)
  if name in TOOLS:
   q=v[:,:2]-v[:,:2].mean(0);_,axis=np.linalg.eigh(q.T@q);y=axis[:,-1]
   if y[1]<0:y=-y
   x=np.array([y[1],-y[0]]);R[:2,:2]=np.array([x,y]).T;v=v@R
  if name=='043_phillips_screwdriver':
   flip=np.diag([-1.,-1.,1.]);v=v@flip;R=R@flip
  lo=v.min(0);hi=v.max(0);center=(lo+hi)/2;v-=center;size=hi-lo
  stage=Usd.Stage.CreateNew(str(OUT/f'{name}.usdc'));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
  root=UsdGeom.Xform.Define(stage,'/Asset');stage.SetDefaultPrim(root.GetPrim());root.GetPrim().SetCustomData({'source':source['url'],'license':'CC-BY-4.0','source_id':name})
  mesh=UsdGeom.Mesh.Define(stage,'/Asset/Mesh');mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype('float32')));mesh.CreateFaceVertexCountsAttr([3]*len(f));mesh.CreateFaceVertexIndicesAttr(f.reshape(-1).tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
  primvar=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying);primvar.Set(Vt.Vec2fArray.FromNumpy(uv.reshape(-1,2).astype('float32')))
  material=UsdShade.Material.Define(stage,'/Asset/Looks/Material');shader=UsdShade.Shader.Define(stage,'/Asset/Looks/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.48);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.0);material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
  tex=UsdShade.Shader.Define(stage,'/Asset/Looks/Material/Texture');tex.CreateIdAttr('UsdUVTexture');texture=obj.parent/'texture_map.png';assert texture.is_file();relative='../external/ycb/'+str(texture.relative_to(SOURCE));tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(relative));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
  st=UsdShade.Shader.Define(stage,'/Asset/Looks/Material/UV');st.CreateIdAttr('UsdPrimvarReader_float2');st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');st.CreateOutput('result',Sdf.ValueTypeNames.Float2);tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material);stage.GetRootLayer().Save()
  record={**source,'usd':str((OUT/f'{name}.usdc').relative_to(ROOT)),'size_m':size.tolist(),'canonical_rotation':R.tolist(),'canonical_center':center.tolist(),'triangles':len(f),'texture':str(texture.relative_to(ROOT)),'texture_sha256':hashlib.sha256(texture.read_bytes()).hexdigest(),'physics':'visual mesh only; scene retains its explicit collision proxy'};records.append(record)
  np.savez_compressed(OUT/f'{name}.npz',vertices=v.astype('float32'),faces=f.astype('int32'),uv=uv.astype('float32'))
  print(name,len(f),size.round(4).tolist(),flush=True)
 (OUT/'catalog.json').write_text(json.dumps({'assets':records},indent=2))
if __name__=='__main__':main()
