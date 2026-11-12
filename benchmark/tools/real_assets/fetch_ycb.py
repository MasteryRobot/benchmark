"""Download selected official YCB 16k scans, preserving sources and checksums."""
import argparse,concurrent.futures,hashlib,json,re,tarfile,urllib.request,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'assets/external/ycb'
BASE='https://ycb-benchmarks.s3.amazonaws.com/'
OBJECTS=['011_banana','013_apple','014_lemon','015_peach','017_orange','024_bowl','025_mug','029_plate','030_fork','031_spoon','032_knife','033_spatula','037_scissors','040_large_marker','043_phillips_screwdriver','048_hammer','056_tennis_ball','065-a_cups','065-b_cups','065-c_cups','003_cracker_box','004_sugar_box','036_wood_block']
def get(url):
 req=urllib.request.Request(url,headers={'User-Agent':'BenchmarkAssetImporter/1.0'})
 with urllib.request.urlopen(req,timeout=45) as r:return r.read()
def main():
 OUT.mkdir(parents=True,exist_ok=True)
 index=get(BASE+'index.html');(OUT/'SOURCE_INDEX.html').write_bytes(index)
 urls=re.findall(r'href="([^"]+)"',index.decode())
 def download(obj):
  links=[u for u in urls if obj+'_google_16k.tgz' in u]
  if len(links)!=1:raise ValueError((obj,links))
  url=links[0]
  if not url.startswith('http'):url=BASE+url.lstrip('/')
  url=url.replace('http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/','https://ycb-benchmarks.s3.amazonaws.com/')
  archive=OUT/(obj+'_google_16k.tgz')
  for attempt in range(3):
   try:
    if not archive.exists():
     data=get(url);temp=archive.with_suffix('.partial');temp.write_bytes(data);temp.replace(archive)
    with tarfile.open(archive,'r:gz') as tf:
     for member in tf.getmembers():
      dest=(OUT/member.name).resolve()
      if not dest.is_relative_to(OUT.resolve()) or member.issym() or member.islnk():raise ValueError('Unsafe archive member')
     tf.extractall(OUT)
    meshes=list((OUT/obj).rglob('textured.obj'));assert len(meshes)==1
    record={'id':obj,'url':url,'license':'CC-BY-4.0','license_url':'https://creativecommons.org/licenses/by/4.0/','attribution':'YCB Object and Model Set — Berk Calli, Arjun Singh, Aaron Walsman, Siddhartha Srinivasa, Pieter Abbeel, Aaron M. Dollar; UC Berkeley / Google scans','archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'archive_bytes':archive.stat().st_size,'obj':str(meshes[0].relative_to(ROOT))}
    print(json.dumps(record),flush=True);return record
   except Exception as err:
    if attempt==2:return {'id':obj,'error':str(err)}
    time.sleep(1)
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:records=list(pool.map(download,OBJECTS))
 (OUT/'sources.json').write_text(json.dumps({'source_page':BASE+'index.html','source_page_sha256':hashlib.sha256(index).hexdigest(),'assets':records},indent=2))
 errors=[r for r in records if 'error' in r];print('FINISHED',len(records)-len(errors),'ERRORS',errors,flush=True)
 if errors:raise SystemExit(1)
if __name__=='__main__':main()
