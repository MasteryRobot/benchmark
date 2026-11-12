"""Isolated Isaac processes avoid retaining state between independent task stages."""
from pathlib import Path
import json,subprocess,os,argparse,sys,tempfile
p=argparse.ArgumentParser();p.add_argument("--only",nargs="*");args=p.parse_args()
root=Path(__file__).resolve().parent;tasks=json.loads((root/'tasks/index.json').read_text());env=dict(os.environ,DISPLAY=':1',OMNI_KIT_ACCEPT_EULA='YES',OPENBLAS_NUM_THREADS='1')
for t in tasks:
 if args.only and t['id'] not in args.only:continue
 folder=root/'tasks'/t['id'];print('START',t['id'],flush=True)
 with (Path(tempfile.gettempdir())/('benchmark_'+t['id']+'.log')).open('w') as f:subprocess.run([sys.executable,str(root/'render_task_scene.py'),t['id']],cwd=root,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=90)
 report=json.loads((folder/'validation.json').read_text());print('DONE',t['id'],report['stable'],report['object_displacement_m'],flush=True)
 if not report['stable']:print('NEEDS_REVIEW',t['id'],flush=True)
