"""GPU preflight and optional surface-scene authoring, never a roll episode."""
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import subprocess
from .config import FOLDER, SPEC


def prepare(output, variant_id='baseline', prepare_only=False, gui=False):
    config = json.loads((FOLDER / 'isaac_config.json').read_text())
    variants = {v['id']: v for v in config['variants']}
    if variant_id not in variants:
        raise ValueError('Unknown towel variant')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {'task': SPEC['id'], 'checked_utc': datetime.now(timezone.utc).isoformat(),
              'variant': variants[variant_id], 'mode': 'surface_authoring' if prepare_only else 'runtime_preflight',
              'status': 'BLOCKED', 'physics_started': False, 'task_success': False, 'blockers': []}
    # Reserve an exclusive report before launching anything; preserve past runs.
    with output.open('x') as stream:
        json.dump(report, stream, indent=2)
    app = None
    try:
        probe = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'],
                               capture_output=True, text=True, timeout=15)
        report['gpu_probe'] = {'returncode': probe.returncode, 'output': (probe.stdout + probe.stderr).strip()}
        if probe.returncode:
            report['blockers'].append('NVIDIA driver is inaccessible in this execution environment')
        if importlib.util.find_spec('isaacsim') is None:
            report['blockers'].append('Isaac Sim is not installed in this Python environment')
        if not prepare_only:
            report['blockers'].append('Articulated cloth-contact controller and measured roll evaluator are not yet integrated')
        if report['blockers']:
            return 2
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': not gui})
        import omni.usd
        from pxr import Sdf
        from .surface import author
        scene = output.with_name(output.stem + '_surface.usda')
        # Reserve the scene too, so an earlier artifact is never replaced.
        with scene.open('x') as stream:
            stream.write('#usda 1.0\n')
        layer = Sdf.Layer.FindOrOpen(str(scene.resolve()))
        layer.subLayerPaths = [str((FOLDER / 'scene_isaac.usda').resolve())]
        layer.Save()
        omni.usd.get_context().open_stage(str(scene.resolve()))
        for _ in range(10): app.update()
        stage = omni.usd.get_context().get_stage()
        report.update(author(stage, variants[variant_id], config['particle_spacing_m']))
        stage.GetRootLayer().Save()
        report.update(status='AUTHORED_NOT_SIMULATED', scene=str(scene.resolve()))
        return 0
    except Exception as exc:
        report['blockers'].append(f'{type(exc).__name__}: {exc}')
        return 2
    finally:
        try:
            if app is not None: app.close()
        finally:
            output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
