"""Collect real successful episodes at 460x260; unavailable runtime is BLOCKED, never success."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np
from task_runtime import ROOT, load_task, runtime_config
from trajectory_capture import CameraRig, RawEpisode, write_json, sha256


def preflight():
    packages = {name: importlib.util.find_spec(name) is not None for name in ['isaacsim', 'pxr', 'numpy', 'scipy', 'PIL']}
    try:
        process = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'],
                                 capture_output=True, text=True, timeout=15)
        gpu = {'available': process.returncode == 0, 'details': (process.stdout + process.stderr).strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        gpu = {'available': False, 'details': str(error)}
    # Some Isaac packages expose pxr only after SimulationApp loads its extensions.
    ready = all(packages[name] for name in ['isaacsim', 'numpy', 'scipy', 'PIL']) and gpu['available']
    return {'packages': packages, 'gpu': gpu, 'status': 'RUNTIME_CHECK_REQUIRED' if ready else 'BLOCKED',
            'note': 'Import/GPU checks do not establish rendering support or task success.'}


def run(args):
    config = json.loads(args.config.read_text())
    tasks = [args.task] if args.task else config['tasks']
    for task_id in tasks:
        if task_id not in config['tasks']:
            raise ValueError('Task is outside the three representative collection tasks')
        cfg = runtime_config(load_task(task_id))
        if abs(1 / (cfg['physics_dt_s'] * cfg['control_decimation']) - config['fps']) > 1e-9:
            raise ValueError('Capture FPS must match environment control frequency')
    if args.attempts < 1 or args.successes < 1:
        raise ValueError('Attempt and success counts must be positive')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output = args.output / stamp
    output.mkdir(parents=True, exist_ok=False)
    probe = preflight()
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'preflight': probe,
              'resolution': config['resolution'], 'fps': config['fps'], 'tasks': tasks,
              'episodes': [], 'successful_episodes': 0, 'status': 'RUNNING'}
    if probe['status'] == 'BLOCKED' or args.preflight_only:
        report['status'] = probe['status'] if probe['status'] == 'BLOCKED' else 'PREFLIGHT_ONLY'
        write_json(output / 'collection_report.json', report)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return 2 if report['status'] == 'BLOCKED' else 0
    app = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': not args.gui, 'width': config['resolution'][0],
                             'height': config['resolution'][1], 'renderer': 'RayTracedLighting',
                             'limit_cpu_threads': 8})
        from isaac_task_env import IsaacTaskEnv
        from representative_policy import CandidatePolicy
        for task_id in tasks:
            succeeded = 0
            for attempt in range(args.attempts):
                env = rig = recorder = None
                error = None
                row = {'task': task_id, 'attempt': attempt, 'success': False, 'status': 'ERROR'}
                try:
                    env = IsaacTaskEnv(task_id, app, render=False)
                    obs = env.reset()
                    rig = CameraRig(env, config)
                    policy = CandidatePolicy(env, variant=attempt % 3)
                    policy.info['source_sha256'] = sha256(ROOT / 'representative_policy.py')
                    recorder = RawEpisode(output / 'raw' / task_id / f'attempt_{attempt:04d}',
                                          env, config, rig, obs, policy.info)
                    elapsed = 0.
                    while not env.evaluator.done:
                        captured = rig.capture()
                        targets = policy.action(obs, elapsed)
                        before = obs
                        prev = float(env.world.current_time)
                        obs, result = env.step(targets, on_physics_step=recorder.physics_step)
                        dt = float(env.world.current_time) - prev
                        recorder.append(before, [targets[n] for n in env.names], captured, obs, result, dt)
                        elapsed = result['elapsed_s']
                    metadata = recorder.finish()
                    recorder = None
                    row.update(status=metadata['status'], success=metadata['success'],
                               frames=metadata['frames'], final_evaluation=metadata['final_evaluation'])
                    succeeded += int(metadata['success'])
                except Exception as exc:
                    error = f'{type(exc).__name__}: {exc}'
                    row['error'] = error
                    traceback.print_exc()
                finally:
                    if recorder is not None:
                        recorder.finish(error=error or 'Interrupted before completion')
                    if rig is not None:
                        rig.close()
                    if env is not None:
                        env.close()
                    report['episodes'].append(row)
                    report['successful_episodes'] += int(row['success'])
                    write_json(output / 'collection_report.json', report)
                    print(json.dumps(row, ensure_ascii=False), flush=True)
                if succeeded >= args.successes:
                    break
        counts = {t: sum(int(r['success']) for r in report['episodes'] if r['task'] == t) for t in tasks}
        report['status'] = 'SUCCESS' if all(n >= args.successes for n in counts.values()) else 'INCOMPLETE'
    except Exception as exc:
        report.update(status='ERROR', error=f'{type(exc).__name__}: {exc}')
        traceback.print_exc()
    finally:
        if app is not None:
            app.close()
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(output / 'collection_report.json', report)
    return 0 if report['status'] == 'SUCCESS' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'collection_config.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'collection_runs')
    parser.add_argument('--task')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--successes', type=int, default=1)
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
