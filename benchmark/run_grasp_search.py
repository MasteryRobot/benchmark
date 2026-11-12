"""Bounded real-physics grasp prerequisite search, all failures retained."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import threading
import numpy as np

ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--task',default='03_pick_phone')
    p.add_argument('--gpus',default='4,5,6,7')
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    variants=[]
    for x in [.035,.045,.055]:
        for z in [.11,.13]:
            for opposition in [1.2,1.6]:
                variants.append({'yaw_deg':72.,'palm_grasp_x_m':x,'palm_grasp_y_m':-.02,
                    'palm_grasp_z_m':z,'thumb_opposition_rad':opposition,
                    'post_contact_closure_rad':.18,'minimum_closure_fraction':.5,
                    'close_fraction':.75,'lift_duration_s':6.})
    queue=list(enumerate(variants));results=[];lock=threading.Lock()
    def worker(gpu):
        while True:
            with lock:
                if not queue:return
                index,cfg=queue.pop(0)
            config_path=out/f'config_{index:03d}.json';config_path.write_text(json.dumps(cfg,indent=2))
            dest=out/f'attempt_{index:03d}'
            command=['bash',str(ROOT/'run_physx_python.sh'),str(ROOT/'collect_native_state.py'),args.task,
                '--gpu',str(gpu),'--policy','geometric_grasp_probe','--policy-config',str(config_path),
                '--duration','20','--output',str(dest)]
            print(json.dumps({'starting':index,'gpu':gpu,'config':cfg}),flush=True)
            with (out/f'attempt_{index:03d}.log').open('x') as log:
                proc=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900)
            r=json.loads((dest/'report.json').read_text());row={'index':index,'config':cfg,
                'status':r['status'],'baseline_success':r.get('baseline_predicate_success',False),
                'functional_success':False,'error':r.get('error'),'exit_code':proc.returncode}
            if (dest/'measured_states.npz').exists():
                d=np.load(dest/'measured_states.npz');start=np.array(r.get('settled_object_poses_xyzw',d['objects'][0]))
                valid=d['time_s']>5.
                heights=d['objects'][valid,0,2]-start[0,2]
                row['max_lift_m']=float(heights.max()) if heights.size else 0.
                row['final_lift_m']=float(heights[-1]) if heights.size else 0.
            with lock:
                results.append(row)
                (out/'search_report.json').write_text(json.dumps({'task':args.task,'planned_attempts':len(variants),
                    'finished_attempts':len(results),'functional_successes':0,'results':results},indent=2))
            print(json.dumps(row),flush=True)
    with ThreadPoolExecutor(max_workers=len(args.gpus.split(','))) as pool:
        list(pool.map(worker,[int(x) for x in args.gpus.split(',')]))

if __name__=='__main__':main()
