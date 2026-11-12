"""CPU perspective-correct texture previews of the composed scan scene assets."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
from numba import njit
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[2]; OUT=ROOT/'reports/real_assets_20261031'
spec=importlib.util.spec_from_file_location('original_raster',ROOT/'reports/scene_views_20261031/render_views.py')
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
W,H=1280,720

@njit(cache=True)
def texture_raster(triangles,uv,texture,inverse,fx,fy,img,depth):
    h,w=depth.shape; th,tw=texture.shape[:2]
    for k in range(len(triangles)):
        v=np.empty((3,5),np.float64)
        for i in range(3):
            for j in range(3):
                v[i,j]=inverse[3,j]
                for n in range(3):v[i,j]+=triangles[k,i,n]*inverse[n,j]
            v[i,2]=-v[i,2];v[i,3]=uv[k,i,0];v[i,4]=uv[k,i,1]
        normal=np.cross(triangles[k,1]-triangles[k,0],triangles[k,2]-triangles[k,0])
        length=np.sqrt(np.sum(normal*normal))
        shade=.78+.22*abs((-normal[0]*.35-normal[1]*.45+normal[2]*.8216)/max(length,1e-12))
        poly=np.empty((6,5),np.float64);count=0
        for i in range(3):
            a=v[(i+2)%3];b=v[i];ina=a[2]>=.01;inb=b[2]>=.01
            if ina!=inb:
                t=(.01-a[2])/(b[2]-a[2]);poly[count]=a+t*(b-a);count+=1
            if inb:poly[count]=b;count+=1
        for fi in range(1,count-1):
            q=np.empty((3,5),np.float64)
            for i,idx in enumerate((0,fi,fi+1)):
                z=poly[idx,2];q[i,0]=w/2+fx*poly[idx,0]/z;q[i,1]=h/2-fy*poly[idx,1]/z
                q[i,2]=1/z;q[i,3]=poly[idx,3]/z;q[i,4]=poly[idx,4]/z
            a,b,c=q
            xmin=max(0,int(np.floor(min(a[0],b[0],c[0]))));xmax=min(w-1,int(np.ceil(max(a[0],b[0],c[0]))))
            ymin=max(0,int(np.floor(min(a[1],b[1],c[1]))));ymax=min(h-1,int(np.ceil(max(a[1],b[1],c[1]))))
            if xmin>xmax or ymin>ymax:continue
            den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(den)<1e-10:continue
            for y in range(ymin,ymax+1):
                for x in range(xmin,xmax+1):
                    u=((b[1]-c[1])*(x+.5-c[0])+(c[0]-b[0])*(y+.5-c[1]))/den
                    vv=((c[1]-a[1])*(x+.5-c[0])+(a[0]-c[0])*(y+.5-c[1]))/den;ww=1-u-vv
                    if u>=-1e-8 and vv>=-1e-8 and ww>=-1e-8:
                        iz=u*a[2]+vv*b[2]+ww*c[2]
                        if iz<=depth[y,x]:continue
                        depth[y,x]=iz
                        tx=((u*a[3]+vv*b[3]+ww*c[3])/iz)%1.0
                        ty=((u*a[4]+vv*b[4]+ww*c[4])/iz)%1.0
                        sx=tx*(tw-1);sy=(1-ty)*(th-1);x0=int(sx);y0=int(sy);x1=min(x0+1,tw-1);y1=min(y0+1,th-1);dx=sx-x0;dy=sy-y0
                        for ch in range(3):
                            color=(1-dy)*((1-dx)*texture[y0,x0,ch]+dx*texture[y0,x1,ch])+dy*((1-dx)*texture[y1,x0,ch]+dx*texture[y1,x1,ch])
                            img[y,x,ch]=min(255,max(0,int(color*shade)))
    return img,depth

def camera(eye,target,focal=46):
    eye=np.array(eye,dtype=float);target=np.array(target,dtype=float);z=eye-target;z/=np.linalg.norm(z)
    x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    mat=np.eye(4);mat[:3,:3]=np.array([x,y,z]);mat[3,:3]=eye
    return {'world':mat.tolist(),'focal':focal,'ha':36.,'va':20.25}

def draw_texture(triangles,uv,texture,cam,buffers):
    return texture_raster(triangles,uv,texture,np.linalg.inv(np.array(cam['world'])),W*cam['focal']/cam['ha'],H*cam['focal']/cam['va'],*buffers)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--limit',type=int);parser.add_argument('--only-missing',action='store_true');args=parser.parse_args()
    entries=json.loads((OUT/'integration.json').read_text())['scenes']
    base,bc,cameras=old.load(ROOT/'reports/scene_views_20261031/geometry/workcell.npz');cameras['overview']=old.overview()
    cached={name:old.draw(base,bc,cameras[name]) for name in old.ORDER}
    catalog=json.loads((ROOT/'assets/realistic/catalog.json').read_text())['assets']
    assets=OUT/'asset_previews';assets.mkdir(exist_ok=True)
    for entry in catalog:
        if args.only_missing and (assets/(entry['id']+'.png')).exists():continue
        data=np.load(ROOT/'assets/realistic'/(entry['id']+'.npz'));verts=data['vertices'];tris=verts[data['faces']];span=max(verts.max(0)-verts.min(0))
        cam=camera([span*1.3,-span*1.7,span*1.9],[0,0,0],50)
        tex=np.array(Image.open(ROOT/entry['texture']).convert('RGB'))
        im=np.full((H,W,3),[237,243,245],dtype='uint8');depth=np.zeros((H,W),dtype='float64')
        im,_=draw_texture(tris,data['uv'],tex,cam,(im,depth));Image.fromarray(im).save(assets/(entry['id']+'.png'))
    for index,entry in enumerate(entries):
        folder=OUT/'views'/entry['id'];folder.mkdir(parents=True,exist_ok=True)
        if args.only_missing and all((folder/(name+'.png')).exists() for name in old.ORDER+['detail']):continue
        data=np.load(OUT/'geometry'/(entry['id']+'.npz'));textured=[]
        for name in json.loads(str(data['textured'])):
            d=np.load(OUT/'geometry'/name);tex=np.array(Image.open(str(d['texture'])).convert('RGB'));textured.append((d['triangles'],d['uv'],tex))
        bounds=np.concatenate([t.reshape(-1,3) for t,_,_ in textured]+[data['triangles'].reshape(-1,3)]);lo=bounds.min(0);hi=bounds.max(0);center=(lo+hi)/2
        span=max(hi[0]-lo[0],hi[1]-lo[1],.30);cameras['detail']=camera(center+np.array([.05,1.5*span,1.8*span]),center,40)
        for name in old.ORDER+['detail']:
            buffers=cached[name] if name in cached else old.draw(base,bc,cameras[name])
            image,depth=old.draw(data['triangles'],data['colors'],cameras[name],buffers)
            for triangles,uv,tex in textured:image,depth=draw_texture(triangles,uv,tex,cameras[name],(image,depth))
            Image.fromarray(image).save(folder/(name+'.png'))
        print('RENDERED',index+1,entry['id'],flush=True)
        if args.limit and index+1>=args.limit:break
if __name__=='__main__':main()
