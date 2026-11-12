"""List and launch original and extended scenarios through one entry point."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from task_runtime import load_tasks
from sharpa.contract import ROOT, load_scenarios


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    listing = sub.add_parser('list')
    listing.add_argument('--suite', choices=['all', 'baseline', 'sharpa', 'deformable'], default='all')
    listing.add_argument('--json', action='store_true')
    run = sub.add_parser('run'); run.add_argument('task')
    run.add_argument('--gui', action='store_true')
    run.add_argument('--actions', type=Path); run.add_argument('--output', type=Path)
    run.add_argument('--prepare-only', action='store_true', help='towel surface authoring only, no physics')
    args = p.parse_args(); rows = []
    for suite, values, runner in [('baseline', load_tasks(), 'run_task_episode.py'),
                                  ('sharpa', load_scenarios(), 'run_sharpa_episode.py'),
                                  ('deformable', [{'id': 'bimanual_roll_towel', 'title': '双灵巧手卷毛巾',
                                                   'task_type': 'deformable_cpu_candidate',
                                                   'source_status': 'LOCAL_CPU_EXPLORATION'}],
                                  'run_towel_roll.py')]:
        for spec in values:
            rows.append(dict(id=spec['id'], title=spec['title'], suite=suite, runner=runner,
                             task_evaluator=spec.get('task_type', 'pose_lift_contact'),
                             source_status=spec.get('source_status', 'EXISTING_BASELINE')))
    if args.command == 'list':
        selected = [r for r in rows if args.suite == 'all' or r['suite'] == args.suite]
        if args.json:
            print(json.dumps(selected, ensure_ascii=False, indent=2))
        else:
            for r in selected:
                print(f'{r["suite"]:8} {r["id"]:58} {r["title"]} [{r["task_evaluator"]}]')
            print(f'{len(selected)} scene entries; not an official demo coverage count.')
        return 0
    matches = [r for r in rows if r['id'] == args.task]
    if not matches:
        p.error('Unknown task; use list to inspect available IDs')
    cmd = [sys.executable, '-B', str(ROOT / matches[0]['runner']), args.task]
    if matches[0]['suite'] == 'deformable':
        cmd.extend(['--backend', 'isaac'])
        if args.prepare_only:
            cmd.append('--prepare-only')
    elif args.prepare_only:
        p.error('--prepare-only is available only for the towel scene')
    if args.gui:
        cmd.append('--gui')
    for flag in ('actions', 'output'):
        value = getattr(args, flag)
        if value is not None:
            cmd.extend(['--' + flag, str(value.resolve())])
    return subprocess.run(cmd, cwd=ROOT).returncode


if __name__ == '__main__':
    sys.exit(main())
