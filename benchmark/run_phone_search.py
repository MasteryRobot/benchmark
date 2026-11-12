"""Execute queued physical trials and stop only after an independently verified success."""
import json
import os
from pathlib import Path
import subprocess
import time
from verify_phone_episode import verify

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'collection_runs/first_success_20261031'
queue=OUT/'queue';queue.mkdir(exist_ok=True)
done=set();idle_since=time.monotonic()
while True:
    pending=[p for p in sorted(queue.glob('*.json')) if p.name not in done]
    if not pending:
        if time.monotonic()-idle_since>900:break
        time.sleep(1);continue
    request=pending[0];done.add(request.name);spec=json.loads(request.read_text())
    dest=OUT/spec['name']
    if dest.exists():continue
    print('START',spec['name'],flush=True)
    with (OUT/(spec['name']+'.log')).open('w') as log:
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='4')
        code=subprocess.call(['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'run_phone_experiment.py'),'--output',str(dest),'--parameters',str(OUT/spec['parameters']),'--gpu','6'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env)
    try:result=verify(dest)
    except Exception as exc:result={'success':False,'error':str(exc)}
    print('RESULT',spec['name'],json.dumps(result),flush=True)
    (OUT/'search_status.json').write_text(json.dumps({'latest_trial':spec['name'],'success':result['success'],'result':result},indent=2))
    if result['success']:break
    idle_since=time.monotonic()
