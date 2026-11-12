"""Run selected/all new scenes in isolated Isaac Python processes."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from sharpa.contract import ROOT,load_scenarios
from run_sharpa_episode import fingerprint
from build_sharpa_scenes import atomic_write


def main():
    specs=load_scenarios();p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--only',nargs='+',choices=[s['id'] for s in specs])
    p.add_argument('--evaluated-only',action='store_true')
    p.add_argument('--timeout',type=float,default=300)
    p.add_argument('--output',type=Path,default=ROOT/'sharpa/runtime_summary.json')
    args=p.parse_args()
    if args.timeout<=0:p.error('timeout must be positive')
    selected=[s for s in specs if (not args.only or s['id'] in args.only)
              and (not args.evaluated_only or s['task_type']!='scene_only')]
    rows=[]
    for spec in selected:
        started=datetime.now(timezone.utc).isoformat()
        destination=ROOT/'sharpa/scenes'/spec['id']/'runtime_validation.json'
        hashes=fingerprint(spec)
        row=dict(task=spec['id'],runtime_status='ERROR',task_success=False,inputs_sha256=hashes,checked_at_utc=started)
        with tempfile.NamedTemporaryFile(prefix='sharpa_runtime_',suffix='.log',delete=False) as log:
            try:
                proc=subprocess.run([sys.executable,'-B',str(ROOT/'run_sharpa_episode.py'),spec['id']],
                                    cwd=ROOT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1'),
                                    stdout=log,stderr=subprocess.STDOUT,timeout=args.timeout)
                data=json.loads(destination.read_text()) if destination.exists() else {}
                if (data.get('task')==spec['id'] and data.get('checked_at_utc','')>=started
                    and data.get('inputs_sha256')==hashes):
                    row.update({k:data[k] for k in ('runtime_status','task_success')})
                    row['reason']=data.get('reason')
                    expected={'PASS':0,'BLOCKED':2}.get(row['runtime_status'],1)
                    if proc.returncode!=expected:row['runtime_status']='ERROR'
                else:row['reason']='No fresh matching runtime report; old evidence rejected.'
                row['returncode']=proc.returncode
            except subprocess.TimeoutExpired:
                row.update(runtime_status='TIMEOUT',reason='Isolated process exceeded timeout')
            except Exception as error:row['reason']=str(error)
            if row['runtime_status'] in ('ERROR','TIMEOUT'):
                atomic_write(destination,json.dumps(row,indent=2)+'\n')
                print('Diagnostic log:',log.name,flush=True)
        rows.append(row);print(spec['id'],row['runtime_status'],flush=True)
    states={r['runtime_status'] for r in rows}
    status='PASS' if states=={'PASS'} else 'BLOCKED' if states and states<={'PASS','BLOCKED'} else 'FAIL'
    report=dict(runtime_status=status,scenes=len(rows),rows=rows,checked_at_utc=datetime.now(timezone.utc).isoformat(),
                scope='Reset/contact smoke only; no autonomous manipulation success claim.')
    atomic_write(args.output,json.dumps(report,indent=2)+'\n')
    return 0 if status=='PASS' else 2 if status=='BLOCKED' else 1


if __name__=='__main__':sys.exit(main())
