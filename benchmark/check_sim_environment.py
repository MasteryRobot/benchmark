"""Record prerequisites; availability alone does not establish runtime success."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime,timezone
from build_sharpa_scenes import atomic_write


def main():
    packages={name:importlib.util.find_spec(name) is not None for name in
              ('numpy','scipy','PIL','pxr','trimesh','shapely','isaacsim')}
    gpu=False; reason='nvidia-smi not found'
    if shutil.which('nvidia-smi'):
        try:
            r=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],capture_output=True,text=True,timeout=15)
            gpu=r.returncode==0; reason=(r.stdout+r.stderr).strip()
        except Exception as e: reason=str(e)
    report=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),python_version=sys.version.split()[0],
                packages=packages,nvidia_driver_available=gpu,gpu_probe=reason,
                cpu_authoring_ready=all(packages[n] for n in ('numpy','scipy','PIL')),
                usd_composition_ready=packages['pxr'],isaac_runtime_ready=packages['isaacsim'] and gpu,
                status='READY_FOR_RUNTIME_CHECK' if packages['isaacsim'] and gpu else 'BLOCKED')
    atomic_write(Path(__file__).resolve().parent/'sharpa/environment.json',json.dumps(report,indent=2)+chr(10))
    print(json.dumps(report,indent=2)); return 0 if report['isaac_runtime_ready'] else 2


if __name__=='__main__': sys.exit(main())
