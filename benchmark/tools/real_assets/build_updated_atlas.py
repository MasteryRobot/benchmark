"""Refresh the complete 83-scene atlas while labeling upgraded versus legacy images."""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/benchmark_mpl')
import json,hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_pdf import PdfPages
from build_review_pdf import page,text,picture,finish,ROOT,OUT,MUTED,ACCENT
PDF=ROOT/'reports/benchmark_all_83_scenes_textured_update_20261031.pdf'
def main():
    entries=json.loads((ROOT/'reports/scene_views_20261031/inventory.json').read_text())
    updates={e['id']:e for e in json.loads((OUT/'integration.json').read_text())['scenes']}
    manifest=[]
    with PdfPages(PDF,metadata={'Title':'All 83 benchmark scenes | 28 scenes with real scanned assets','Author':'Benchmark workspace'}) as pdf:
        f=page();text(f,.05,.88,'BENCHMARK / UPDATED ATLAS',28,color=ACCENT,weight='bold')
        text(f,.05,.76,'All 83 task scenes',43,weight='bold');text(f,.05,.67,'Global overview + five camera views',24)
        lines=['28 scenes now contain 45 textured YCB scan instances.','55 scenes retain their previous images and are labeled as unchanged.','23 scan assets were downloaded; only semantically matching objects were integrated.','Mapped visuals retain UV textures and the CC BY 4.0 source attribution.','Scans: YCB Object and Model Set, UC Berkeley / Google.','Source: ycb-benchmarks.s3.amazonaws.com/index.html','Existing collision proxies and task contracts remain; contact fidelity is unvalidated.','Updated images are offline CPU texture renders, not Isaac RTX or task-success evidence.','83 scene pages + cover + 3 index pages = 87 pages; 498 images.']
        for i,line in enumerate(lines):text(f,.05,.55-i*.043,line,15,color=MUTED)
        finish(pdf,f,1)
        for k in range(3):
            f=page();text(f,.035,.94,'Scene index / '+str(k+1)+' of 3',25,weight='bold');text(f,.035,.895,'* = real scan assets integrated in this update',12,color=ACCENT)
            for j,e in enumerate(entries[k*30:(k+1)*30]):
                n=k*30+j;text(f,.035,.86-j*.0255,('* ' if e['id'] in updates else '  ')+e['id'],12);text(f,.96,.86-j*.0255,str(n+5),12,ha='right')
            finish(pdf,f,k+2)
        for i,e in enumerate(entries):
            f=page();changed=e['id'] in updates;name=e['id'].replace('sharpa_dex_','').replace('sharpa_demo_','').replace('_',' ')
            text(f,.035,.945,f'{i+1:02d} / '+name,23,weight='bold');text(f,.035,.905,e['scene'],10,color=MUTED)
            text(f,.965,.905,'TEXTURED SCAN UPDATE' if changed else 'UNCHANGED / PREVIOUS IMAGE',12,color=ACCENT if changed else MUTED,ha='right',weight='bold')
            folder=OUT/'views'/e['id'] if changed else ROOT/'tasks'/e['id']/'images' if e['suite']=='baseline' else ROOT/'reports/scene_views_20261031/views'/e['id']
            paths=[]
            for k,view in enumerate(['overview','head','leftwrist','rightwrist','leftside','rightside']):
                x=.035+(k%3)*.317;y=.52-(k//3)*.355;path=folder/(view+'.png')
                picture(f,path,[x,y,.296,.235]);text(f,x,y+.25,view.upper(),12,color=ACCENT,weight='bold');paths.append(str(path.relative_to(ROOT)))
            text(f,.035,.10,'YCB scans with original physics proxies; static CPU texture rendering.' if changed else ('Existing saved simulation RGB.' if e['suite']=='baseline' else 'Prior offline geometry preview; asset fidelity has not been upgraded.'),12,color=MUTED)
            finish(pdf,f,i+5);manifest.append({'id':e['id'],'page':i+5,'updated':changed,'images':paths})
        assert pdf.get_pagecount()==87
    (OUT/'updated_atlas_manifest.json').write_text(json.dumps({'pdf':str(PDF.relative_to(ROOT)),'pages':87,'bytes':PDF.stat().st_size,'sha256':hashlib.sha256(PDF.read_bytes()).hexdigest(),'entries':manifest},indent=2))
    print(PDF,PDF.stat().st_size,flush=True)
if __name__=='__main__':main()
