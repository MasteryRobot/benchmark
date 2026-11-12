"""Audit packaged text, USD metadata, mesh headers, archives and image metadata.
Additional private identity terms can be supplied with --terms-file outside this project.
The report records categories/counts only, never matched sensitive values.
"""
from pathlib import Path
import argparse,json,re,zipfile
from pxr import Sdf
from PIL import Image
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--terms-file',type=Path);args=p.parse_args()
terms=json.loads(args.terms_file.read_text()) if args.terms_file else []
rules={
 'repository_address':re.compile(r'(?:https?://|git@)(?:github\.com|gitlab[^/\s]*|gitee\.com|bitbucket\.org)[:/]',re.I),
 'personal_machine_path':re.compile(r'/(?:home|Users)/[A-Za-z0-9_.-]+/|[A-Z]:\\Users\\',re.I),
 'email_address':re.compile(r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.(?:com|cn|org|net|io)\b',re.I),
}
if terms:rules['private_identity']=re.compile('|'.join((r'\b'+re.escape(t)+r'\b') if len(t)<=2 else re.escape(t) for t in terms),re.I)
findings=[];counts={};total=0

def inspect(text,path,kind):
 counts[kind]=counts.get(kind,0)+1
 for category,rx in rules.items():
  if rx.search(text):findings.append({'path':path,'category':category,'representation':kind})
for f in sorted(ROOT.rglob('*')):
 rel=str(f.relative_to(ROOT))
 if f.is_symlink():findings.append({'path':rel,'category':'symlink','representation':'path'});continue
 inspect(rel,rel,'path')
 if f.is_dir():
  if f.name in {'.git','__pycache__'}:findings.append({'path':rel,'category':'history_or_cache','representation':'path'})
  continue
 total+=1
 if f.name=='privacy_report.json':continue
 if f.suffix in {'.usdc','.usd'}:
  layer=Sdf.Layer.FindOrOpen(str(f));assert layer;inspect(layer.ExportToString(),rel,'usd_decoded')
 elif f.suffix.lower()=='.stl':
  with f.open('rb') as stream:head=stream.read(84)
  assert f.stat().st_size==84+50*int.from_bytes(head[80:84],'little')
  assert head[:80].rstrip() in [b'Generic simulation mesh',b'Generic sensor mesh']
  inspect(head[:80].decode(),rel,'stl_header')
 elif f.suffix.lower()=='.png':
  with Image.open(f) as im:inspect(json.dumps(im.info),rel,'png_metadata')
 elif f.suffix=='.npz':
  with zipfile.ZipFile(f) as z:
   for n in z.namelist():
    assert n.endswith('.npy');inspect(n,rel,'array_member')
    with z.open(n) as data:inspect(data.read(128).decode('latin1'),rel,'array_header')
 elif f.suffix in {'.py','.json','.usda','.urdf','.svg','.md','.txt','.html'}:
  text=f.read_text()
  # Embedded screenshots are independently scanned as image assets.
  text=re.sub(r'data:image/png;base64,[A-Za-z0-9+/=]+','embedded-image',text)
  inspect(text,rel,'text')
 else:findings.append({'path':rel,'category':'unreviewed_file_type','representation':'file'})
result={'result':'PASS' if not findings else 'FAIL','files_scanned':total,'representations':counts,'external_identity_terms_checked':len(terms),'findings':findings,'scope':'Identifiers and metadata; visual inspection recorded separately in model_integrity.json.'}
(ROOT/'privacy_report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(bool(findings))
