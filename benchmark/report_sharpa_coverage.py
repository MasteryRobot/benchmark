"""Generate a capability inventory without confusing scene counts with demo coverage."""
from collections import Counter
from datetime import datetime, timezone
import json
from sharpa.contract import ROOT, load_scenarios
from build_sharpa_scenes import atomic_write


def main():
    specs = load_scenarios()
    counts = Counter(s['coverage_level'] for s in specs)
    lines = ['# 场景能力对照表', '',
             '以下为本地原型清单，官方来源均待核实。入口数不代表独立官方 demo 数或成功率。', '',
             f'共 {len(specs)} 个入口：{counts["RIGID_BODY_TASK_APPROXIMATION"]} 个有近似判定，'
             f'{counts["SCENE_ASSETS_ONLY"]} 个只有道具与布局。所有 Isaac 物理实测仍未完成。', '',
             '发牌在演示候选与桌面清单中重复；其它可能的重叠须按官方视频去重。', '',
             '| 场景 ID | 名称 | 支持程度 | 尚缺能力 / 限制 |',
             '| --- | --- | --- | --- |']
    for spec in specs:
        scope = '刚体近似判定（未实测）' if spec['task_type'] != 'scene_only' else '仅场景道具（无任务判定）'
        notes = '; '.join(spec['limitations']) or '局部位姿/接触代理判定；未验证可达性或完整操作。'
        lines.append('| ' + ' | '.join([spec['id'], spec['title'], scope, notes.replace('|', '/')]) + ' |')
    atomic_write(ROOT/'sharpa/coverage.md', chr(10).join(lines)+chr(10))
    source = dict(status='AWAITING_VERIFIABLE_OFFICIAL_SOURCES', checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  requested_scope='As comprehensive as possible across demonstrated Sharpa demos',
                  local_scene_entries=len(specs), official_total=None, verified_official_tasks=[],
                  source_claim_correction='Earlier conversational statements of confirmed official inventory lacked retrievable source evidence.',
                  local_sources=['59 provisional tabletop identifiers', '5 conversation demo candidates', '8 additional unverified demo candidates'],
                  known_duplicate_entries=[['sharpa_card_dealing', 'sharpa_dex_deal_playing_cards']],
                  provenance='All geometry, physical parameters and goal predicates are locally designed; no official asset or policy import.',
                  source_urls='Candidate locators in catalog.py are unverified, not source evidence.')
    atomic_write(ROOT/'sharpa/source_status.json', json.dumps(source,ensure_ascii=False,indent=2)+chr(10))
    checks = json.loads((ROOT/'sharpa/validation_summary.json').read_text())
    runtime = json.loads((ROOT/'sharpa/runtime_summary.json').read_text())
    env = json.loads((ROOT/'sharpa/environment.json').read_text())
    status = dict(status='INCOMPLETE', checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  local_scene_entries=len(specs), baseline_scene_entries=10, capability_counts=dict(counts),
                  offline_status=checks['offline_status'], original_baseline_preserved=checks['original_baseline_preserved'],
                  runtime_status=runtime['runtime_status'], runtime_status_counts=dict(Counter(r['runtime_status'] for r in runtime['rows'])),
                  usd_composition_status_counts=dict(Counter(r['usd_composition'] for r in checks['rows'])),
                  environment_status=env['status'], autonomous_policy='NOT_IMPLEMENTED',
                  official_coverage='UNVERIFIED', official_total=None, previews='CPU_SCHEMATIC_ONLY',
                  blockers=['Official sources/task identities not independently confirmed.',
                            'Isaac Sim, USD Python bindings and functioning NVIDIA driver unavailable.',
                            '47 scene-only tasks still require process-specific physics and task predicates.',
                            'No autonomous demonstration trajectories or policies implemented.'])
    atomic_write(ROOT/'sharpa/completion_status.json',json.dumps(status,ensure_ascii=False,indent=2)+chr(10))
    print(json.dumps(status,ensure_ascii=False,indent=2))


if __name__ == '__main__':main()
