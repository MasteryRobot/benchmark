Benchmark GPU rebuild and five-camera atlas

Source: /home/ubuntu/zzf/benchmark (updated 2026-10-31)
Independent rebuilt workspace: /home/ubuntu/projects/benchmark_gpu_rebuild
Runtime: /home/ubuntu/miniforge3/envs/isaaclab/bin/python
Isaac Sim: 6.0.1-rc.7; GPU: NVIDIA GeForce RTX 5090.

Rebuilt shared workcell, ten baseline tasks and 72 Sharpa prototype layers,
including the updated YCB textured scan bindings. The existing towel scene
is loaded into GPU rendering as the 83rd task. It is not rebuilt into a new
cloth physics model. All 83 entries are freshly rendered in RTX at 1280x720.

output/pdf/benchmark_gpu_five_camera_views.pdf: cover + linked index + 83 scene pages.
output/views/<task>/: five fresh RGB PNGs, five geometric ray-distance depth NPZs,
Overview.png, and per-scene capture.json. Depth is in meters, not stereo sensor data.
output/manifest.json: input scene and image SHA-256 hashes.
output/gpu_physics_smoke.json: measured 54-DOF, 240-step cuda:0 PhysX smoke check
on the phone scene, GPU dynamics and GPU broadphase enabled.
output_capture.log and output/physics_smoke.log: actual runtime logs.

Reproduce/resume rendering: ./run_gpu_capture.sh
Rebuild scene assets then resume: ./run_gpu_capture.sh --rebuild
Existing capture.json entries are skipped on resume. For a fresh capture,
use a new workspace or move output/views aside first.
Build PDF: /home/ubuntu/miniforge3/envs/isaaclab/bin/python make_gpu_pdf.py
Physics check: /home/ubuntu/miniforge3/envs/isaaclab/bin/python gpu_physics_smoke.py

The atlas captures authored initial states with a paused timeline. It does
not certify task success, all-scene dynamics, grasp contact fidelity or cloth
behavior. Historical validation files copied from the source remain historical;
only files under output/ and output_capture.log are this run's evidence.
YCB source attribution is preserved in assets/external/ycb/ATTRIBUTION.md.

GPU scene entries: output/gpu_scenes/<task_id>.usda. These reference the rebuilt
scene and explicitly enable PhysX GPU dynamics and GPU broadphase. All 83 USD
entries and all required camera paths passed composition/configuration checks;
see output/gpu_entry_validation.json. Open these entries in Isaac Sim for GPU
physics configuration. Configuration validation is separate from dynamics tests.
