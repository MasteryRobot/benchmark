"""Offline JSON trace evaluation; never certifies a physical benchmark run."""
import argparse
import hashlib
import json
from pathlib import Path
from .evaluator import FunctionalEvaluator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('contract', type=Path)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads(args.contract.read_text())
    trace = json.loads(args.trace.read_text())
    if trace['task'] != spec['id'] or not trace['samples']:
        parser.error('Trace task mismatch or empty samples')
    evaluator = FunctionalEvaluator(spec)
    results = [evaluator.update(row['observation'], row['dt_s']) for row in trace['samples']]
    report = {'task': spec['id'], 'evidence_level': 'OFFLINE_TRACE_ONLY',
              'physical_success_verified': False, 'evaluation': results[-1],
              'steps': results, 'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in (args.contract, args.trace, Path(__file__),
                                                   Path(__file__).with_name('evaluator.py'))}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({k: report[k] for k in ('evidence_level', 'physical_success_verified', 'evaluation')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
