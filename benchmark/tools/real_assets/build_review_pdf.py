"""Build a reviewable before/after PDF with all upgraded scenes and scan sources."""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/benchmark_mpl')
import json,hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'reports/real_assets_20261031'
PDF=ROOT/'reports/benchmark_real_assets_upgrade_20261031.pdf'
BG='#edf3f5';INK='#183247';MUTED='#526d7c';ACCENT='#0d858b'
def page():return plt.figure(figsize=(16.54,11.69),facecolor=BG)
def text(f,x,y,s,size=12,color=INK,**kw):f.text(x,y,s,fontsize=size,color=color,**kw)
def picture(f,path,rect):
    with Image.open(path) as im:
        ax=f.add_axes(rect);ax.imshow(im.convert('RGB'),interpolation='none');ax.axis('off')
def finish(pdf,f,n):
    text(f,.035,.018,'YCB SCANS / CC BY 4.0 / STATIC ASSET REVIEW / 31 OCT 2026',10,color=MUTED)
    text(f,.965,.018,str(n),10,ha='right',color=MUTED);pdf.savefig(f,facecolor=BG);plt.close(f)
def main():
    report=json.loads((OUT/'integration.json').read_text());assets=json.loads((ROOT/'assets/realistic/catalog.json').read_text())['assets']
    with PdfPages(PDF,metadata={'Title':'Benchmark real scan asset upgrade | 28 scenes, 45 props','Author':'Benchmark workspace'}) as pdf:
        f=page();text(f,.05,.89,'REAL OBJECT SCANS',32,color=ACCENT,weight='bold');text(f,.05,.79,'Benchmark asset upgrade',39,weight='bold')
        text(f,.05,.70,'23 downloaded scans / 28 scenes updated / 45 object instances',23)
        lines=['The previous scene expansion used simple geometric placeholders.','This update imports actual textured YCB scans into the existing USD scenes.','UV textures, source URLs, CC BY 4.0 attribution and archive checksums are retained.','25 Sharpa scenes and 3 baseline scenes now reference the imported scan assets.','55 registered scenes are unchanged; unmapped props remain explicitly listed.','Existing collision proxies, body masses, joints and task goals are preserved.','Appearance is upgraded; mesh/contact agreement and live physics remain unvalidated.','Previews use CPU texture rendering, not an Isaac RTX run or success demonstration.']
        for i,line in enumerate(lines):text(f,.05,.58-i*.047,line,16,color=MUTED)
        finish(pdf,f,1)
        for k in range(2):
            f=page();text(f,.035,.94,'Downloaded YCB scan assets / '+str(k+1)+' of 2',24,weight='bold')
            for j,a in enumerate(assets[k*12:(k+1)*12]):
                x=.03+(j%4)*.245;y=.67-(j//4)*.285
                picture(f,OUT/'asset_previews'/(a['id']+'.png'),[x,y,.235,.195]);text(f,x+.006,y-.018,a['id'],12)
            finish(pdf,f,k+2)
        for i,e in enumerate(report['scenes']):
            f=page();title=e['id'].replace('sharpa_dex_','').replace('sharpa_demo_','').replace('_',' ')
            text(f,.035,.95,title,23,weight='bold');text(f,.035,.91,e['scene'],10,color=MUTED)
            folder=OUT/'views'/e['id'];before=ROOT/'reports/scene_views_20261031/views'/e['id']/'head.png'
            if e['id'][0].isdigit():before=ROOT/'tasks'/e['id']/'images/head.png'
            picture(f,before,[.035,.49,.455,.34]);picture(f,folder/'head.png',[.51,.49,.455,.34])
            text(f,.035,.845,'BEFORE / saved preview',14,color=MUTED,weight='bold');text(f,.51,.845,'AFTER / textured scan geometry',14,color=ACCENT,weight='bold')
            picture(f,folder/'detail.png',[.035,.185,.47,.28]);text(f,.035,.475,'DETAIL / current scene',13,color=ACCENT,weight='bold')
            text(f,.53,.45,'Imported object instances',15,weight='bold')
            for j,b in enumerate(e['bindings']):text(f,.53,.408-j*.029,b['body_id']+'  ->  '+b['asset_id'],12)
            text(f,.53,.235,'Original physics proxies retained.',12,color=MUTED);text(f,.53,.205,'Static preview; task success not verified.',12,color=MUTED)
            for j,name in enumerate(['head','leftwrist','rightwrist','leftside','rightside']):
                x=.035+j*.188;picture(f,folder/(name+'.png'),[x,.052,.178,.13]);text(f,x,.038,name.upper(),9,color=MUTED)
            if i in (0,12,27):f.savefig(OUT/('review_page_'+str(i+1)+'.jpg'),dpi=100,facecolor=BG)
            finish(pdf,f,i+4)
        assert pdf.get_pagecount()==31
    (OUT/'pdf_manifest.json').write_text(json.dumps({'pdf':str(PDF.relative_to(ROOT)),'pages':31,'bytes':PDF.stat().st_size,'sha256':hashlib.sha256(PDF.read_bytes()).hexdigest(),'scenes':28,'updated_instances':45},indent=2))
    print(PDF,PDF.stat().st_size,flush=True)
if __name__=='__main__':main()
