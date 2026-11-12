"""Retrieve public asset catalogs and per-asset metadata for complete replacement."""
import argparse,concurrent.futures,gzip,json,urllib.request,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'assets/external/objaverse/catalog'
BASE='https://huggingface.co/datasets/allenai/objaverse/resolve/main/'
def fetch(url,path):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():return path
    for attempt in range(3):
        try:
            request=urllib.request.Request(url,headers={'User-Agent':'BenchmarkAssetResearch/1.0'})
            with urllib.request.urlopen(request,timeout=90) as response: data=response.read()
            temp=path.with_suffix(path.suffix+'.partial');temp.write_bytes(data);temp.replace(path)
            print('SAVED',path.name,len(data),flush=True);return path
        except Exception as e:
            if attempt==2:raise
            time.sleep(attempt+1)
def main():
    p=argparse.ArgumentParser();p.add_argument('--metadata',action='store_true');p.add_argument('--shard');args=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    for name in ['object-paths.json.gz']:
        fetch(BASE+name,OUT/name)
    try:fetch('https://raw.githubusercontent.com/allenai/objaverse/main/objaverse/__init__.py',OUT/'upstream_loader.py')
    except Exception as e:print('loader',str(e),flush=True)
    if args.shard:fetch(BASE+'metadata/'+args.shard+'.json.gz',OUT/'metadata'/(args.shard+'.json.gz'))
    if args.metadata:
        paths=json.loads(gzip.decompress((OUT/'object-paths.json.gz').read_bytes()))
        shards=sorted({path.split('/')[1] for path in paths.values()})
        def run(shard):
            try:return str(fetch(BASE+'metadata/'+shard+'.json.gz',OUT/'metadata'/(shard+'.json.gz')))
            except Exception as e:print('ERROR',shard,str(e),flush=True);return {'shard':shard,'error':str(e)}
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:results=list(pool.map(run,shards))
        (OUT/'metadata_download_status.json').write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
