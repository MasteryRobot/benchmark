"""Reproduce benchmark gaps using synthetic inputs; never certifies physics."""
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from .evaluator import FunctionalEvaluator

ROOT = Path(__file__).resolve().parents[1]


def observation(angle=0, bodies=('thumb', 'index'), mass=0):
    pose = np.eye(4)
    radians = np.deg2rad(angle)
    c, s = np.cos(radians), np.sin(radians)
    pose[:2, :2] = [[c, -s], [s, c]]
    return {'object_pose_world': pose,
            'palm_pose_world': {'left': np.eye(4), 'right': np.eye(4)},
            'finger_positions_rad': {'left': [angle / 180, 0]},
            'contacts': {'left': [{'body': b, 'force_n': .1,
                                   'point_object_m': [0, -.04, 0]} for b in bodies], 'right': []},
            'lift_above_settled_m': .1, 'external_support_force_n': 0,
            'outcomes': {'delivered_mass_kg': mass}}


def rows(obs, count, dt=.01):
    return [(deepcopy(obs), dt) for _ in range(count)]


def evaluate(spec, samples):
    evaluator = FunctionalEvaluator(spec)
    result = None
    for index, (obs, dt) in enumerate(samples):
        result = evaluator.update(obs, dt)
        if result['done']:
            break
    return {**result, 'samples_consumed': index + 1}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; use a new path to preserve historical evidence')
    spec = json.loads((ROOT / 'functional/example_contract.json').read_text())
    acquired = rows(observation(), 15)
    goal = observation(90, ('thumb', 'middle'))
    new_outcome = observation(90, ('thumb', 'middle'), .01)
    valid = acquired + rows(goal, 25) + rows(new_outcome, 25)
    assisted = observation()
    assisted['contacts']['right'] = assisted['contacts']['left']
    assisted['contacts']['left'] = []
    single_digit = rows(observation(bodies=('index_proximal', 'index_distal')), 15)
    single_digit += rows(observation(90, ('index_middle', 'index_distal')), 25)
    single_digit += rows(observation(90, ('index_middle', 'index_distal'), .01), 25)
    arm_spec = deepcopy(spec)
    arm_spec['track'] = 'free_wrist'
    arm = deepcopy(new_outcome)
    arm['palm_pose_world']['left'] = arm['object_pose_world'].copy()
    patch_spec = deepcopy(spec)
    patch_spec['allow_contact_patch_motion'] = True
    patch = observation(90)
    for contact in patch['contacts']['left']:
        contact['point_object_m'][1] += .01
    patch_outcome = deepcopy(patch)
    patch_outcome['outcomes']['delivered_mass_kg'] = .01
    no_rotation = observation(0, ('thumb', 'middle'))
    patch_pose_loss = deepcopy(no_rotation)
    patch_pose_loss['outcomes']['delivered_mass_kg'] = .01
    dynamic_spec = deepcopy(spec)
    dynamic_spec['dynamic_relative_pose_during_use'] = True
    cases = {
        'synthetic_positive_control': (spec, valid, True),
        'direct_functional_grasp_rejected': (spec, rows(observation(90), 40)
            + rows(observation(90, mass=.01), 25), False),
        'assisted_trace_passes_default_contract': (spec, acquired + rows(assisted, 1)
            + rows(goal, 25) + rows(new_outcome, 25), True),
        'one_digit_multiple_links_pass': (spec, single_digit, True),
        'sparse_three_samples_pass_hold_timers': (spec, [(observation(), .15),
            (goal, .25), (new_outcome, .25)], True),
        'arm_only_relative_motion_control': (arm_spec, acquired + rows(arm, 80), False),
        'stale_outcome_control': (spec, acquired + rows(new_outcome, 80), False),
        'same_bodies_rolling_rejected_by_default': (spec, acquired + rows(patch, 25)
            + rows(patch_outcome, 25), False),
        'contact_patch_option_accepts_same_bodies': (patch_spec, acquired + rows(patch, 25)
            + rows(patch_outcome, 25), True),
        'dynamic_use_option_accepts_goal_pose_loss': (dynamic_spec, acquired + rows(goal, 25)
            + rows(patch_pose_loss, 25), True),
    }
    findings = {}
    for name, (contract, samples, expected) in cases.items():
        result = evaluate(contract, samples)
        findings[name] = {'synthetic_only': True, 'expected_current_predicate_success': expected,
                          'matches_expected_current_behavior': result['predicate_success'] == expected,
                          'contract_overrides': {k: v for k, v in contract.items() if spec.get(k) != v},
                          'result': result}
    tests = subprocess.run([sys.executable, '-B', '-m', 'unittest', 'discover', '-s', 'tests', '-v'],
                           cwd=ROOT, capture_output=True, text=True)
    baseline = json.loads((ROOT / 'tasks/index.json').read_text())
    prototypes = json.loads((ROOT / 'sharpa/index.json').read_text())
    collection = json.loads((ROOT / 'collection_checks/summary.json').read_text())
    listing = subprocess.run([sys.executable, '-B', 'benchmark_cli.py', 'list', '--json'],
                             cwd=ROOT, capture_output=True, text=True, check=True)
    paths = ['functional/audit_gaps.py', 'functional/evaluator.py', 'functional/example_contract.json',
             'functional/evaluate_trace.py', 'isaac_task_env.py', 'representative_policy.py',
             'task_objects.py', 'benchmark_cli.py', 'benchmark_suites.json',
             'tasks/index.json', 'sharpa/index.json', 'collection_checks/summary.json',
             'runtime_checks/physx_gpu_probe_006.json',
             'docs/functional_reference_sources_20261030.json']
    paths += [str(p.relative_to(ROOT)) for p in sorted((ROOT / 'tests').glob('test_*.py'))]
    paths += [str(p.relative_to(ROOT)) for p in sorted((ROOT / 'towel_roll/attempts').glob('gpu_native*/report.json'))]
    paths += [str(p.relative_to(ROOT)) for p in sorted((ROOT / 'collection_runs').rglob('collection_report.json'))]
    paths += [str(p.relative_to(ROOT)) for p in sorted((ROOT / 'collection_runs').rglob('isaac_capture.log'))]
    asset_paths = ['reports/real_assets_20261031/integration.json',
                   'reports/real_assets_20261031/validation.json',
                   'reports/real_assets_20261031/README.md']
    paths += [p for p in asset_paths if (ROOT / p).exists()]
    visual_assets = {}
    if (ROOT / asset_paths[0]).exists():
        assets = json.loads((ROOT / asset_paths[0]).read_text())
        visual_assets = {k: assets[k] for k in ('upgraded_scene_count', 'upgraded_body_count',
                                               'downloaded_scan_assets', 'physics_status')}
    runtime = {}
    for path in paths:
        if (path.endswith('/report.json') or path.endswith('/collection_report.json')
                or path == 'runtime_checks/physx_gpu_probe_006.json'):
            data = json.loads((ROOT / path).read_text())
            keys = ('created_utc', 'checked_at_utc', 'status', 'backend', 'scope', 'physics_started',
                    'task_success', 'finger_contact_verified', 'error', 'simulated_seconds',
                    'started_at_utc', 'finished_at_utc', 'successful_episodes')
            runtime[path] = {k: data[k] for k in keys if k in data}
            if 'preflight' in data:
                preflight = data['preflight']
                runtime[path]['preflight'] = {
                    'status': preflight.get('status'),
                    'packages': preflight.get('packages'),
                    'gpu_available': preflight.get('gpu', {}).get('available')}
            if isinstance(data.get('episodes'), list):
                runtime[path]['recorded_episode_count'] = len(data['episodes'])
    previous = json.loads((ROOT / 'docs/functional_benchmark_audit_20261030.json').read_text())
    hashes = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}
    report = {
        'audit_completed_utc': datetime.now(timezone.utc).isoformat(),
        'evidence_level': 'LOCAL_SOURCE_AND_SYNTHETIC_AUDIT_ONLY',
        'new_physical_trials': 0, 'physical_success_verified': False,
        'inventory': {'registered_entries': len(json.loads(listing.stdout)),
                      'baseline': len(baseline), 'prototypes': len(prototypes),
                      'prototype_coverage': dict(Counter(x['coverage_level'] for x in prototypes)),
                      'recorded_successful_collected_episodes': collection['real_successful_episodes']},
        'synthetic_checks': findings,
        'all_checks_match_expected_current_behavior': all(x['matches_expected_current_behavior'] for x in findings.values()),
        'unit_tests': {'returncode': tests.returncode, 'stdout': tests.stdout, 'stderr': tests.stderr},
        'historical_runtime_evidence': runtime,
        'visual_asset_upgrade': visual_assets,
        'collection_run_reports_with_success': [p for p, r in runtime.items()
            if p.endswith('/collection_report.json') and r.get('successful_episodes', 0) > 0],
        'changes_since_previous_audit': [p for p, h in previous['source_sha256'].items()
            if not (ROOT / p).exists() or hashlib.sha256((ROOT / p).read_bytes()).hexdigest() != h],
        'source_sha256': hashes,
        'interpretation': 'Expected behavior includes exposed gaps. Passing this audit is not benchmark acceptance.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
    print(json.dumps({k: report[k] for k in ('inventory', 'all_checks_match_expected_current_behavior')}, ensure_ascii=False))
    print('unit_test_returncode:', tests.returncode)
    return int(tests.returncode != 0 or not report['all_checks_match_expected_current_behavior'])


if __name__ == '__main__':
    raise SystemExit(main())
