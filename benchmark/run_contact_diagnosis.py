"""Controlled phone diagnostics; preserve original success thresholds."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 best=json.loads((ROOT/'collection_runs/all_tasks_20261031/phone_grasp_search_001/config_004.json').read_text())
 variants=[
  {'name':'velocity_iterations_zero','gpu':4,'physics':{'articulation_velocity_iterations':0,'object_velocity_iterations':0},'substeps':1,'policy':{}},
  {'name':'integration_480hz','gpu':5,'physics':{},'substeps':2,'policy':{}},
  {'name':'force_feedback','gpu':6,'physics':{},'substeps':1,'policy':{'hold_force_ceiling_n':8.,'force_release_gain':.01}}]
 (out/'experiment.json').write_text(json.dumps({'purpose':'Discriminate integration, velocity-iteration, and excess-squeeze causes. No acceptance thresholds changed.','variants':variants},indent=2))
 def worker(v):
  dest=out/v['name'];pc=out/(v['name']+'_policy.json');ph=out/(v['name']+'_physics.json')
  pc.write_text(json.dumps({**best,**v['policy']},indent=2));ph.write_text(json.dumps(v['physics'],indent=2))
  cmd=['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'collect_native_state.py'),'03_pick_phone','--gpu',str(v['gpu']),'--policy','geometric_grasp_probe','--policy-config',str(pc),'--physics-config',str(ph),'--physics-substeps',str(v['substeps']),'--duration','20','--wall-timeout','900','--output',str(dest)]
  with (out/(v['name']+'.log')).open('x') as f:r=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=1000)
  report=json.loads((dest/'report.json').read_text());summary={'name':v['name'],'exit_code':r.returncode,**{k:report.get(k) for k in ['status','recorded_physics_frames','baseline_predicate_success','functional_success','final_evaluation','error']}}
  print(json.dumps(summary),flush=True);return summary
 with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(worker,variants))
 (out/'comparison.json').write_text(json.dumps({'status':'COMPLETE','results':results},indent=2))
 subprocess.run([sys.executable,'-B',str(ROOT/'verify_native_collection.py'),str(out)],check=True,cwd=ROOT)
if __name__=='__main__':main()
