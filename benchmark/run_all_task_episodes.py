"""Run each task in an isolated Isaac process and aggregate fresh runtime reports."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from task_runtime import ROOT, load_tasks, scene_fingerprint


def main():
    tasks = load_tasks()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', nargs='+', choices=[t['id'] for t in tasks])
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--output', type=Path, default=ROOT / 'tasks/runtime_summary.json')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    selected = [t for t in tasks if not args.only or t['id'] in args.only]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1')
    rows = []
    for task in selected:
        print('START', task['id'], flush=True)
        output = ROOT / 'tasks' / task['id'] / 'runtime_validation.json'
        started = datetime.now(timezone.utc).isoformat()
        row = {'task': task['id'], 'checked_at_utc': started,
               'inputs_sha256': scene_fingerprint(task), 'runtime_status': 'ERROR',
               'task_success': False}
        with tempfile.NamedTemporaryFile(prefix='benchmark_runtime_', suffix='.log', delete=False) as log:
            try:
                run = subprocess.run(
                    [sys.executable, '-B', str(ROOT / 'run_task_episode.py'), task['id']],
                    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
                data = json.loads(output.read_text()) if output.is_file() else {}
                fresh = (data.get('task') == task['id'] and data.get('checked_at_utc', '') >= started
                         and data.get('inputs_sha256') == row['inputs_sha256'])
                if fresh:
                    row.update({k: data[k] for k in ('runtime_status', 'task_success')})
                    if 'reason' in data:
                        row['reason'] = data['reason']
                    if run.returncode != 0 and row['runtime_status'] == 'PASS':
                        row['runtime_status'] = 'ERROR'
                else:
                    row['reason'] = 'No fresh report was produced; previous report is not accepted.'
                row['returncode'] = run.returncode
            except subprocess.TimeoutExpired:
                row['runtime_status'] = 'TIMEOUT'
                row['reason'] = 'Task process exceeded the configured timeout.'
            except Exception as error:
                row['error_type'] = type(error).__name__
            if row['runtime_status'] in ('ERROR', 'TIMEOUT'):
                output.write_text(json.dumps(row, indent=2) + '\n')
            if row['runtime_status'] not in ('PASS', 'BLOCKED'):
                print('Diagnostic log:', log.name, flush=True)
        rows.append(row)
        print('DONE', task['id'], row['runtime_status'], flush=True)
    statuses = {r['runtime_status'] for r in rows}
    status = 'PASS' if statuses == {'PASS'} else 'BLOCKED' if statuses <= {'PASS', 'BLOCKED'} else 'FAIL'
    report = {'checked_at_utc': datetime.now(timezone.utc).isoformat(),
              'runtime_status': status, 'tasks': len(rows), 'rows': rows,
              'scope': 'Fresh reset/contact smoke tests; PASS is not a manipulation success rate.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'runtime_status': status, 'tasks': len(rows)}))
    return 0 if status == 'PASS' else 2 if status == 'BLOCKED' else 1


if __name__ == '__main__':
    sys.exit(main())
