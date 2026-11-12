"""Native PhysX GPU startup test; never a benchmark task success."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import traceback

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = {'checked_at_utc': datetime.now(timezone.utc).isoformat(),
              'task_success': False, 'status': 'NOT_RUN'}
    physx = stage = binding = None
    try:
        import numpy as np
        import ovstage
        import ovphysx
        from ovphysx import PhysX, PhysXConfig
        from ovphysx.samples.python_samples.schema_support import physx_schema_paths
        from ovphysx.types import TensorType
        ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
        path = Path(ovphysx.__file__).parent / 'samples/data/links_chain_sample_gpu.usda'
        report['backend'] = 'Standalone NVIDIA PhysX ' + ovphysx.__version__
        report['scope'] = 'GPU articulation smoke only; not Isaac Sim or task success'
        stage = ovstage.Stage('benchmark-gpu-probe')
        ovstage.population.open_usd(stage, str(path), ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physx = PhysX(active_cuda_gpus='0', config=PhysXConfig(
            carbonite_overrides={'/physics/suppressReadback': True}))
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        binding = physx.create_tensor_binding(
            pattern='/World/articulation/articulationLink*', tensor_type=TensorType.ARTICULATION_LINK_POSE)
        report['device'] = str(binding.native_device)
        report['shape'] = list(binding.shape)
        if binding.native_device.device_type.value != 2:
            raise RuntimeError('PhysX binding is not CUDA; refusing CPU fallback')
        before = np.zeros(binding.shape, dtype=np.float32)
        after = np.zeros_like(before)
        binding.read(before)
        for _ in range(120): physx.step_sync(1 / 240)
        binding.read(after)
        if not np.isfinite(after).all(): raise RuntimeError('Nonfinite state')
        report['max_link_position_change_m'] = float(np.max(np.abs(after[..., :3] - before[..., :3])))
        if report['max_link_position_change_m'] < 1e-5: raise RuntimeError('No physical motion')
        report.update(status='PASS', physics_steps=120, simulated_seconds=.5)
    except Exception as exc:
        report.update(status='ERROR', error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
    finally:
        try:
            if binding is not None: binding.destroy()
            if physx is not None:
                if stage is not None and physx.is_stage_attached(stage): physx.wait_task(physx.detach_ovstage())
                physx.destroy()
            if stage is not None: stage.destroy()
        finally:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x') as f: json.dump(report, f, indent=2)
            print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status'] == 'PASS' else 2

if __name__ == '__main__': raise SystemExit(main())
