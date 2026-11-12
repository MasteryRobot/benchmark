"""CPU schematic of ALL layouts; never represents Isaac camera output."""
import html
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from sharpa.contract import ROOT, load_scenarios
from check_sharpa_scenes import part_bounds
from build_sharpa_scenes import atomic_write


def main():
    specs = load_scenarios(); out = ROOT/'sharpa/previews'; out.mkdir(exist_ok=True)
    fontpath = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font = ImageFont.truetype(str(fontpath),12) if fontpath.exists() else ImageFont.load_default()
    cells=[]; links=[]
    for spec in specs:
        im=Image.new('RGB',(480,340),'#eef2f5'); d=ImageDraw.Draw(im)
        d.text((12,8),spec['id'].replace('sharpa_','')[:54],fill='#14212b',font=font)
        d.text((12,31),'LOCAL LAYOUT / UNVERIFIED SOURCE / NOT SIM RENDER',fill='#a75d10',font=font)
        def pixel(x,y): return (240+x*300,290-(y-.15)*300)
        d.rectangle([pixel(-.66,.88),pixel(.66,.15)],fill='#263a44',outline='#71808a',width=2)
        for b in spec['bodies']:
            for p,(lo,hi) in zip(b['parts'],part_bounds(b)):
                rgb=tuple(int(c*255) for c in p['color'])
                rect=[pixel(lo[0],hi[1]),pixel(hi[0],lo[1])]
                if p['kind']=='sphere' or (p['kind']=='cylinder' and p.get('axis','Z')=='Z' and b['orientation_wxyz']==[1,0,0,0]):
                    d.ellipse(rect,fill=rgb,outline='#d9dfe2')
                else: d.rectangle(rect,fill=rgb,outline='#d9dfe2')
        for g in spec['goals']:
            x,y=pixel(*g['position'][:2])
            d.line([(x-5,y),(x+5,y)],fill='#29ffbd',width=2)
            d.line([(x,y-5),(x,y+5)],fill='#29ffbd',width=2)
        label='PROPS ONLY / TASK PHYSICS MISSING' if spec['task_type']=='scene_only' else 'RIGID APPROXIMATION / LIVE CHECK PENDING'
        d.text((12,307),label,fill='#a75d10',font=font)
        d.text((12,323),'Top view; robot and camera rig inherited from workcell',fill='#5b6470',font=font)
        im.save(out/(spec['id']+'.png')); cells.append(im)
        links.append('<article><h3>'+html.escape(spec['title'])+'</h3><img loading="lazy" src="previews/'+spec['id']+'.png"><p>'+
                     html.escape(spec['coverage_level'])+'</p><ul>'+''.join('<li>'+html.escape(x)+'</li>' for x in spec['limitations'])+
                     '</ul><a href="scenes/'+spec['id']+'/task.json">Task metadata</a> · <a href="scenes/'+spec['id']+'/scene.usda">USD layer</a></article>')
    cols=4; gallery=Image.new('RGB',(cols*480,math.ceil(len(cells)/cols)*340),'white')
    for i,im in enumerate(cells): gallery.paste(im,((i%cols)*480,(i//cols)*340))
    gallery.save(ROOT/'sharpa/scene_layouts.jpg',quality=90)
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>场景扩展原型</title>
<style>body{font:15px system-ui;margin:32px;background:#edf1f4;color:#182d3a}header{max-width:1000px}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(400px,1fr));gap:20px}article{background:white;border-radius:8px;padding:16px}img{width:100%}li{margin:8px 0}a{color:#075985}.warning{background:#fff1d8;padding:16px}</style>
<header><h1>72 个本地场景原型</h1><p class="warning">来源待复核。这里是本地布置示意，不是 Isaac 渲染或任务成功证据。25 个场景含近似任务判定，47 个仅有道具布置，缺失完整任务物理。继续使用现有中性 54 自由度双臂模型。</p><p>官方 demo 总量未知，不能据此计算覆盖率。</p></header><main>'''+''.join(links)+'</main></html>'
    atomic_write(ROOT/'sharpa/gallery.html',page)
    print(f'Wrote {len(cells)} labeled schematic previews and gallery.html')


if __name__=='__main__': main()
