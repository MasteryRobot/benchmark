"""Replay genuine joint commands, never object poses."""
import argparse,json,subprocess,sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
ROOT=Path(__file__).resolve().parent
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--fractions',type=float,nargs='+',default=[0.,.5,.8]);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 source=ROOT/'collection_runs/all_tasks_20261031/phone_grasp_search_001/attempt_004/measured_states.npz'
 if len(a.fractions)>3 or any(not 0<=f<=1 for f in a.fractions):raise ValueError('Use 1-3 fractions within [0,1]')
 variants=[('relax_'+str(round(f*100)),f,gpu) for f,gpu in zip(a.fractions,[4,6,7])]
 (out/'experiment.json').write_text(json.dumps({'source':str(source),'variants':variants,'relax_at_simulation_s':20.,'scope':'Joint commands only. No object state writes, no success threshold changes.'},indent=2))
 def worker(item):
  name,fraction,gpu=item;dest=out/name
  cmd=['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'collect_native_state.py'),'03_pick_phone','--gpu',str(gpu),'--replay-actions',str(source),'--replay-relax-fraction',str(fraction),'--duration','20','--output',str(dest)]
  with (out/(name+'.log')).open('x') as log:r=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900)
  report=json.loads((dest/'report.json').read_text());row={'name':name,'exit_code':r.returncode,**{k:report.get(k) for k in ['status','recorded_physics_frames','baseline_predicate_success','functional_success','final_evaluation','error']}}
  print(json.dumps(row),flush=True);return row
 with ThreadPoolExecutor(max_workers=3) as pool:rows=list(pool.map(worker,variants))
 (out/'comparison.json').write_text(json.dumps({'results':rows},indent=2))
 subprocess.run([sys.executable,'-B',str(ROOT/'verify_native_collection.py'),str(out)],cwd=ROOT,check=True)
if __name__=='__main__':main()
