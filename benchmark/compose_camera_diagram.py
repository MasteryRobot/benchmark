"""Compose six unmodified screenshot assets into an annotated SVG diagram and PNG."""
from pathlib import Path
import base64
import hashlib
import json
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'images'
W, H = 3840, 2700
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
       '<rect width="100%" height="100%" fill="#f4f6f9"/>',
       '<g font-family="Noto Sans CJK SC, sans-serif">']
manifest = {}

def text(x, y, value, size=40, fill='#192b40', anchor='start', weight=500):
    svg.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" text-anchor="{anchor}" font-weight="{weight}">{escape(value)}</text>')

def panel(name, title, x, y, w, h, number=None, color='#192b40'):
    # Original PNG bytes are embedded without decoding or altering the source.
    raw = (OUT / f'{name}.png').read_bytes()
    manifest[name] = {'source': f'images/{name}.png', 'sha256': hashlib.sha256(raw).hexdigest(),
                      'image_rect': [x, y, w, h], 'label': title}
    svg.append(f'<rect x="{x-12}" y="{y-70}" width="{w+24}" height="{h+82}" rx="16" fill="white" stroke="#d6dfe9" stroke-width="3"/>')
    if number:
        svg.append(f'<rect x="{x+7}" y="{y-56}" width="68" height="43" rx="9" fill="{color}"/>')
        text(x+41, y-23, number, 28, 'white', 'middle', 700)
        text(x+92, y-22, title, 35, weight=700)
    else:
        text(x+18, y-22, title, 40, weight=700)
    data = base64.b64encode(raw).decode()
    svg.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet" xlink:href="data:image/png;base64,{data}"/>')

text(W/2, 82, 'RGBD 五机位视角总览', 60, anchor='middle', weight=700)
# Labels follow the robot's own left/right; the overview faces the robot.
colors = {'Head':'#187bb9','LeftWrist':'#d17b15','RightWrist':'#8258b3','LeftSide':'#159374','RightSide':'#d25260'}
panel('head','头部相机',1360,225,1120,630,'01',colors['Head'])
panel('overview','整体布局',1240,960,1360,1020)
panel('rightside','桌右侧相机',70,1130,1080,607.5,'05',colors['RightSide'])
panel('leftside','桌左侧相机',2690,1130,1080,607.5,'04',colors['LeftSide'])
panel('rightwrist','右腕相机',80,2020,1120,630,'03',colors['RightWrist'])
panel('leftwrist','左腕相机',2640,2020,1120,630,'02',colors['LeftWrist'])
# Source locations obtained by projecting cameras.json optical centers through
# the exact overview camera saved in render_scene.py (1600 x 1200).
import math
specs=json.loads((ROOT/'cameras.json').read_text())
eye=[3.1,3.8,2.6];target=[0,.05,.86]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def unit(a):return [x/math.sqrt(dot(a,a)) for x in a]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
forward=unit([t-e for t,e in zip(target,eye)]);right=unit(cross(forward,[0,0,1]));up=cross(right,forward)
uv={}
for name,spec in specs.items():
 d=[p-e for p,e in zip(spec['eye_m'],eye)];depth=dot(d,forward)
 uv[name]=(.5+dot(d,right)*50/(36*depth),.5-dot(d,up)*50/(27*depth))

positions = {k:(1240+1360*u,960+1020*v) for k,(u,v) in uv.items()}
x,y=positions['Head']
paths={'Head':f'M {x},{y} L {x},915 Q {x},892 {x+23},892 L 1897,892 Q 1920,892 1920,869 L 1920,855'}
x,y=positions['LeftSide'];paths['LeftSide']=f'M {x},{y} L 2638,{y} Q 2660,{y} 2660,{y+22} L 2660,1434 L 2690,1434'
x,y=positions['RightSide'];paths['RightSide']=f'M {x},{y} L 1208,{y} Q 1185,{y} 1185,{y-23} L 1185,1434 L 1150,1434'
x,y=positions['LeftWrist'];paths['LeftWrist']=f'M {x},{y} C {x+70},{y+155} 2490,1780 2540,1875 L 2540,1920 Q 2540,1940 2560,1940 L 2900,1940 L 2900,1950'
x,y=positions['RightWrist'];paths['RightWrist']=f'M {x},{y} C {x-55},{y+155} 1360,1780 1310,1875 L 1310,1920 Q 1310,1940 1290,1940 L 940,1940 L 940,1950'
for number,name in enumerate(['Head','LeftWrist','RightWrist','LeftSide','RightSide'],1):
    color=colors[name];d=paths[name];x,y=positions[name]
    svg.append(f'<path d="{d}" fill="none" stroke="white" stroke-width="10" stroke-linecap="round" stroke-linejoin="round" opacity=".85"/>')
    svg.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append(f'<circle cx="{x}" cy="{y}" r="24" fill="{color}" stroke="white" stroke-width="4"/>')
    text(x,y+10,str(number),29,'white','middle',700)
    manifest[name] = {'overview_source_uv':uv[name], 'marker_position':[x,y], 'number':number}
text(1920,2150,'左右以机器人自身为准',32,'#596b7d','middle')
text(1920,2210,'引线编号与对应机位画面一致',27,'#758496','middle')
svg.extend(['</g>','</svg>'])
svg_path = OUT/'camera_views_overview.svg'
svg_path.write_text('\n'.join(svg))
(OUT/'camera_views_overview_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
import cairosvg
cairosvg.svg2png(url=str(svg_path),write_to=str(OUT/'camera_views_overview.png'))
print(OUT/'camera_views_overview.png')
print(svg_path)
