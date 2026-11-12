"""Aggregate current evidence; historical physics reports cannot produce PASS."""
from datetime import datetime, timezone
import json
import sys
from check_task_runtime import audit
from task_runtime import ROOT


def main():
    offline = audit()
    rows = offline['rows']
    failed = any(r['offline_status'] != 'PASS' or r['physics_status'] in
                 ('ERROR', 'FAIL', 'TIMEOUT') for r in rows)
    complete = all(r['offline_status'] == 'PASS' and r['usd_composition'] == 'PASS'
                   and r['physics_status'] == 'PASS' for r in rows)
    result = 'FAIL' if failed else 'PASS' if complete else 'INCOMPLETE'
    report = {
        'result': result, 'checked_at_utc': datetime.now(timezone.utc).isoformat(),
        'tasks': len(rows), 'rows': rows,
        'scope': 'Current assets, USD composition and fingerprinted reset/contact smoke reports. Historical validation.json is retained for reference only; no autonomous manipulation success claim.',
    }
    (ROOT / 'tasks/runtime_checks.json').write_text(json.dumps(offline, indent=2) + '\n')
    (ROOT / 'tasks/validation_summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'result': result, 'tasks': len(rows)}))
    return 0 if result == 'PASS' else 1 if result == 'FAIL' else 2


if __name__ == '__main__':
    sys.exit(main())
