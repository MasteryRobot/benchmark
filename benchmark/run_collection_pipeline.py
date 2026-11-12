"""Run all three tasks in isolated processes, then export the same successes twice."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from task_runtime import ROOT
from trajectory_capture import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-python', type=Path, required=True)
    parser.add_argument('--v21-python', type=Path, default=ROOT / '.venvs/lerobot-v21/bin/python')
    parser.add_argument('--v30-python', type=Path, default=ROOT / '.venvs/lerobot-v30/bin/python')
    parser.add_argument('--output', type=Path, default=ROOT / 'datasets')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--successes', type=int, default=1)
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    run = args.output / stamp
    run.mkdir(parents=True, exist_ok=False)
    config = json.loads((ROOT / 'collection_config.json').read_text())
    report = {'status': 'RUNNING', 'collection': [], 'exports': []}
    for task in config['tasks']:
        cmd = [str(args.isaac_python), '-B', str(ROOT / 'collect_representative.py'),
               '--task', task, '--output', str(run / 'collection'),
               '--attempts', str(args.attempts), '--successes', str(args.successes)]
        with (run / f'{task}.log').open('w') as log:
            result = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        report['collection'].append({'task': task, 'exit_code': result.returncode})
        write_json(run / 'pipeline_report.json', report)
    successes = [p for p in (run / 'collection').rglob('episode.json')
                 if json.loads(p.read_text()).get('status') == 'SUCCESS']
    if successes:
        for version, python in [('v2.1', args.v21_python), ('v3.0', args.v30_python)]:
            cmd = [str(python), '-B', str(ROOT / 'export_lerobot.py'), '--raw-root', str(run / 'collection'),
                   '--output', str(run / f'lerobot_{version}'), '--format', version]
            with (run / f'export_{version}.log').open('w') as log:
                result = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            report['exports'].append({'format': version, 'exit_code': result.returncode})
    else:
        report['export_reason'] = 'No successful measured episodes; dataset export intentionally not created'
    complete = bool(successes) and all(x['exit_code'] == 0 for x in report['collection'] + report['exports'])
    report.update(status='SUCCESS' if complete else 'BLOCKED' if all(x['exit_code'] == 2 for x in report['collection']) else 'INCOMPLETE',
                  successful_episodes=len(successes))
    write_json(run / 'pipeline_report.json', report)
    print(json.dumps({'run': str(run), **report}, ensure_ascii=False, indent=2))
    return 0 if complete else 2 if report['status'] == 'BLOCKED' else 1


if __name__ == '__main__':
    sys.exit(main())
