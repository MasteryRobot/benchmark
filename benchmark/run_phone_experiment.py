"""Run isolated physical phone grasp experiments using the existing recorder."""
import argparse
import json
import hashlib
from pathlib import Path
from types import SimpleNamespace
import collect_phone_native as capture
import representative_policy
from phone_grasp_policy import PhoneGraspPolicy

p=argparse.ArgumentParser()
p.add_argument('--output',type=Path,required=True)
p.add_argument('--parameters',type=Path,required=True)
p.add_argument('--gpu',type=int,default=0)
a=p.parse_args()
parameters=json.loads(a.parameters.read_text())
sources={name:Path(name).read_bytes() for name in ['phone_grasp_policy.py','collect_phone_native.py','task_runtime.py','run_phone_experiment.py']}
class ConfiguredPhonePolicy(PhoneGraspPolicy):
    def __init__(self, env, variant=0):
        super().__init__(env, parameters)
representative_policy.CandidatePolicy=ConfiguredPhonePolicy
args=SimpleNamespace(task='03_pick_phone',output=a.output,gpu=a.gpu,duration=20.,wall_timeout=600.,policy='baseline_candidate',variant=0)
result=capture.run(args)
(a.output/'policy_parameters.json').write_text(json.dumps(parameters,indent=2))
for name,data in sources.items():
    (a.output/('source_'+name)).write_bytes(data)
raise SystemExit(result)
