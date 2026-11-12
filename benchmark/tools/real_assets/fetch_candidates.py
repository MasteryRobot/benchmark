"""Download only attributed open-license candidates selected from public metadata."""
import argparse,concurrent.futures,gzip,hashlib,json,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'reports/full_asset_replacement_20261031';ASSETS=ROOT/'assets/external/objaverse';BASE='https://huggingface.co/datasets/allenai/objaverse/resolve/main/'
def get(url,path):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():return
    for attempt in range(3):
        try:
            req=urllib.request.Request(url,headers={'User-Agent':'BenchmarkOpenAssetImporter/1.0'})
            with urllib.request.urlopen(req,timeout=90) as response:data=response.read()
            temp=path.with_suffix('.partial');temp.write_bytes(data);temp.replace(path);return
        except Exception:
            if attempt==2:raise
            time.sleep(attempt+1)
def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['thumbnails','models'],default='thumbnails');p.add_argument('--limit',type=int,default=3);args=p.parse_args()
    candidates=json.loads((OUT/'candidates.json').read_text());models={}
    if args.mode=='thumbnails':
        for kind,items in candidates.items():
            for item in items[:args.limit]:models[item['uid']]=item
    else:
        selected=json.loads((OUT/'selected.json').read_text());uids={x['uid'] for x in selected.values() if x.get('uid')}
        for items in candidates.values():
            for item in items:
                if item['uid'] in uids:models[item['uid']]=item
        missing=uids-set(models)
        if missing:
            for shard in sorted((ASSETS/'catalog/metadata').glob('*.gz')):
                raw=json.loads(gzip.decompress(shard.read_bytes()))
                for uid in missing&set(raw):models[uid]=raw[uid]
                missing-=set(raw)
                if not missing:break
        assert not missing,missing
    paths=json.loads(gzip.decompress((ASSETS/'catalog/object-paths.json.gz').read_bytes()))
    def run(item):
        uid=item['uid']
        try:
            assert item['license'] in ('by','cc0','by-sa')
            if args.mode=='thumbnails':
                images=item.get('thumbnails',[]);images=images.get('images',[]) if isinstance(images,dict) else images
                image=min(images,key=lambda i:abs(i['width']-512))
                get(image['url'],OUT/'thumbnails'/(uid+'.jpg'))
            else:
                dest=ASSETS/'models'/uid;dest.mkdir(parents=True,exist_ok=True)
                url=BASE+paths[uid];get(url,dest/'model.glb')
                if (dest/'model.glb').read_bytes()[:4]!=b'glTF':raise ValueError('Not a GLB')
                metadata={**item,'download_url':url,'sha256':hashlib.sha256((dest/'model.glb').read_bytes()).hexdigest(),'bytes':(dest/'model.glb').stat().st_size}
                (dest/'source.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
                print('MODEL',uid,metadata['bytes'],item['name'],flush=True)
            return {'uid':uid,'status':'OK'}
        except Exception as e:print('ERROR',uid,str(e),flush=True);return {'uid':uid,'status':'ERROR','error':str(e)}
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:results=list(pool.map(run,models.values()))
    (OUT/(args.mode+'_download_status.json')).write_text(json.dumps(results,indent=2));print('COMPLETE',args.mode,len(results),'errors',sum(r['status']=='ERROR' for r in results),flush=True)
if __name__=='__main__':main()
