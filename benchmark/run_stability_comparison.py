"""Run explicitly documented solver variants, never lower success criteria."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy-config',type=Path,required=True)
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    variants=[
        {'solveArticulationContactLast':True},
        {'solveArticulationContactLast':True,'articulation_position_iterations':64,
         'object_position_iterations':64,'articulation_velocity_iterations':0,'object_velocity_iterations':0},
        {'solveArticulationContactLast':True,'finger_stiffness_nm_rad':12.,
         'finger_damping_nm_s_rad':.6,'finger_max_force_nm':3.}
    ]
    def worker(item):
        i,tuning=item;config=out/f'physics_{i:02d}.json';config.write_text(json.dumps(tuning,indent=2))
        dest=out/f'attempt_{i:02d}'
        command=['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'collect_native_state.py'),
            '03_pick_phone','--gpu',str(i+4),'--policy','geometric_grasp_probe',
            '--policy-config',str(a.policy_config.resolve()),'--physics-config',str(config),
            '--duration','20','--output',str(dest)]
        with (out/f'attempt_{i:02d}.log').open('x') as log:
            result=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900)
        report=json.loads((dest/'report.json').read_text())
        summary={'variant':i,'tuning':tuning,'exit_code':result.returncode,
                 'status':report['status'],'baseline_success':report.get('baseline_predicate_success',False),
                 'functional_success':False,'final_evaluation':report.get('final_evaluation'),
                 'error':report.get('error')}
        print(json.dumps(summary),flush=True)
        return summary
    with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(worker,enumerate(variants)))
    (out/'comparison.json').write_text(json.dumps({'status':'COMPLETED_DIAGNOSTIC_COMPARISON',
        'functional_successes':0,'results':results},indent=2))
    subprocess.run([sys.executable,'-B',str(ROOT/'verify_native_collection.py'),str(out)],check=True,cwd=ROOT)

if __name__=='__main__':main()
