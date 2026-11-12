"""Small orthographic z-buffer renderer for correct CAD/table occlusion."""
import numpy as np
from numba import njit
from PIL import Image, ImageDraw, ImageFont
from .geometry import cad_meshes, towel_surface
from .config import SPEC, FOLDER

@njit(cache=True)
def rasterize(projected, colors, width, height):
    image=np.empty((height,width,3),np.uint8); image[:]=np.array([237,243,245],np.uint8)
    depth=np.full((height,width),-1e30)
    for i in range(len(projected)):
        a,b,c=projected[i]
        xmin=max(0,int(np.floor(min(a[0],b[0],c[0])))); xmax=min(width-1,int(np.ceil(max(a[0],b[0],c[0]))))
        ymin=max(0,int(np.floor(min(a[1],b[1],c[1])))); ymax=min(height-1,int(np.ceil(max(a[1],b[1],c[1]))))
        den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den)<1e-9: continue
        for y in range(ymin,ymax+1):
            for x in range(xmin,xmax+1):
                u=((b[1]-c[1])*(x+.5-c[0])+(c[0]-b[0])*(y+.5-c[1]))/den
                v=((c[1]-a[1])*(x+.5-c[0])+(a[0]-c[0])*(y+.5-c[1]))/den
                w=1-u-v
                if u>=-1e-8 and v>=-1e-8 and w>=-1e-8:
                    d=u*a[2]+v*b[2]+w*c[2]
                    if d>depth[y,x]:
                        depth[y,x]=d;image[y,x]=colors[i]
    return image

def mesh_scene(q, centerline=None):
    triangles=[]; colors=[]
    def add(tris,color):
        tris=np.asarray(tris); triangles.append(tris)
        normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        light=np.array([-.4,-.35,.85]);light/=np.linalg.norm(light)
        shade=.5+.5*np.abs(normals@light)
        colors.append(np.clip(shade[:,None]*np.asarray(color)*255,0,255).astype(np.uint8))
    for _,tris,color in cad_meshes(q):add(tris,color)
    th=np.linspace(0,np.pi,120);z=SPEC['towel']['table_height_m']
    outer=np.c_[1.04*np.cos(th),-.22+1.10*np.sin(th),np.full(len(th),z)]
    inner=np.c_[.40*np.cos(th),-.22+.36*np.sin(th),np.full(len(th),z)]
    for i in range(len(th)-1):
        add([[outer[i],outer[i+1],inner[i]],[inner[i],outer[i+1],inner[i+1]]],[.15,.23,.27])
    v,f=towel_surface(centerline);nx=SPEC['towel']['grid_width'];ny=len(v)//nx
    color=np.tile(SPEC['towel']['color'],(len(f),1))
    for i,face in enumerate(f):
        x=face[0]%nx;y=face[0]//nx
        if x in (0,1,nx-3,nx-2) or y in (0,1,ny-3,ny-2):color[i]=[.72,.93,.91]
        elif (x+y)%2:color[i]*=.94
    add(v[f],color)
    return np.concatenate(triangles),np.concatenate(colors)

def view(tris,colors,eye,target,span,width=1000,height=800):
    eye=np.array(eye);target=np.array(target);forward=eye-target;forward/=np.linalg.norm(forward)
    right=np.cross([0,0,1],forward);right/=np.linalg.norm(right);up=np.cross(forward,right)
    pts=(tris-target)@np.array([right,up,forward]).T
    pts[:,:,0]=width/2+pts[:,:,0]*width/span
    pts[:,:,1]=height/2-pts[:,:,1]*width/span
    return Image.fromarray(rasterize(pts,colors,width,height))

def render_preview(q,path=None,centerline=None,title='BIMANUAL TOWEL ROLL | INITIAL SCENE'):
    path=path or FOLDER/'images/scene_preview.png';path.parent.mkdir(parents=True,exist_ok=True)
    tris,colors=mesh_scene(q,centerline)
    im=Image.new('RGB',(1800,990),'#edf3f5')
    im.paste(view(tris,colors,[2.4,3.0,3.0],[0,.18,.92],1.9,900,800),(0,120))
    im.paste(view(tris,colors,[0.1,.10,1.75],[0,.45,.86],.90,900,800),(900,120))
    draw=ImageDraw.Draw(im);font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    draw.text((40,24),title,fill='#183440',font_size=32,stroke_width=1)
    draw.text((40,76),'Existing dual-arm CAD | Left + right dexterous hands | 36 x 40 cm deformable towel',fill='#436473',font_size=20,stroke_width=0)
    draw.text((40,945),'CPU CAD preview | Not an Isaac sensor image | Physical success pending',fill='#9a5a14',font_size=20,stroke_width=0)
    im.save(path);return path

if __name__ == '__main__':
    from .geometry import initial_pose
    render_preview(initial_pose()[0])
