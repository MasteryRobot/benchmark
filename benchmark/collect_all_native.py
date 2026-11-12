"""Run isolated GPU prerequisite attempts for every registered benchmark task.

Baseline tasks run the experimental grasp policy. Scene-only prototypes run
measured settling/contact diagnostics because no task controller exists.
No diagnostic is counted as task success or exported as successful training data.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import threading
from collection_inventory import inventory

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--gpus', default='4,5,6,7')
    p.add_argument('--tasks', nargs='*')
    args = p.parse_args(); out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    gpus = [int(x) for x in args.gpus.split(',')]
    rows = []
    for suite, path in [('baseline','tasks/index.json'),('sharpa','sharpa/index.json')]:
        for spec in json.loads((ROOT/path).read_text()):
            rows.append({'task':spec['id'], 'suite':suite, 'kind':spec.get('task_type','pose_lift_contact')})
    rows.append({'task':'bimanual_roll_towel','suite':'deformable','kind':'cloth'})
    if args.tasks: rows = [r for r in rows if r['task'] in args.tasks]
    pending = list(rows); lock = threading.Lock()
    report = {'started_utc':datetime.now(timezone.utc).isoformat(), 'status':'RUNNING',
              'registered_tasks':len(rows), 'functional_successes':0, 'complete':False, 'results':[]}
    def save():
        path = out/'batch_report.json'; temp=out/'batch_report.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');temp.replace(path)
    save()
    def worker(gpu):
        while True:
            with lock:
                if not pending: return
                row = pending.pop(0)
            task = row['task']; dest = out/task
            if row['suite']=='deformable':
                command=['bash',str(ROOT/'run_physx_python.sh'),'-m','towel_roll.native_physx',
                         '--output',str(dest),'--tuning',str(ROOT/'towel_roll/gpu_contact_tuning.json')]
            else:
                command=['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'collect_native_state.py'),
                         task,'--gpu',str(gpu),'--output',str(dest),'--duration',
                         '20' if row['suite']=='baseline' else '1','--policy',
                         'baseline_candidate' if row['suite']=='baseline' else 'hold']
            started = datetime.now(timezone.utc).isoformat()
            print(json.dumps({'starting':task,'gpu':gpu,'utc':started}),flush=True)
            try:
                with (out/(task+'.log')).open('x') as log:
                    result=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900)
                code=result.returncode; error=None
            except Exception as exc:
                code=-1;error=str(exc)
            path=dest/'report.json'; data=json.loads(path.read_text()) if path.exists() else {}
            entry={**row,'gpu':gpu,'started_utc':started,'exit_code':code,
                   'report':str(path.relative_to(out)), 'status':data.get('status','PROCESS_ERROR'),
                   'physics_started':data.get('physics_started',False),
                   'recorded_frames':data.get('recorded_physics_frames',data.get('recorded_frames',0)),
                   'functional_success':False,'error':error or data.get('error')}
            with lock:
                report['results'].append(entry);save()
            print(json.dumps(entry,ensure_ascii=False),flush=True)
    with ThreadPoolExecutor(max_workers=len(gpus)) as pool:
        futures=[pool.submit(worker,gpu) for gpu in gpus]
        for future in as_completed(futures): future.result()
    report.update(status='INCOMPLETE_NO_FUNCTIONAL_SUCCESSES',finished_utc=datetime.now(timezone.utc).isoformat())
    save();inventory(out)
    print(json.dumps({'output':str(out),'attempted_tasks':len(report['results']),
        'tasks_with_physics':sum(r['physics_started'] for r in report['results']),
        'functional_successes':0}),flush=True)
    return 2


if __name__=='__main__': raise SystemExit(main())
