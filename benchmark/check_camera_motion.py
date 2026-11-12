"""Verify both wrist cameras follow their own articulated link in PhysX."""
import json, os
from pathlib import Path
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'limit_cpu_threads': 8})
import numpy as np
import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

ROOT = Path(__file__).resolve().parent
specs = json.loads((ROOT / 'cameras.json').read_text())
omni.usd.get_context().open_stage(str(ROOT / 'dealer_table.usdc'))
stage = omni.usd.get_context().get_stage()
for _ in range(10):
    app.update()
world = World(stage_units_in_meters=1, physics_dt=1/120, rendering_dt=1/30)
robot = world.scene.add(SingleArticulation('/World/Robot/DualArm', name='DualArm'))
world.reset()

def frames():
    cache = UsdGeom.XformCache()
    result = {}
    for name, spec in specs.items():
        camera = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(spec['prim_path']))).T
        parent = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(spec['parent_body']))).T
        result[name] = (camera, np.linalg.inv(parent) @ camera)
    return result

report = {'scope': 'Each wrist joint independently perturbed by 0.12 rad in PhysX; USD file unchanged.', 'cases': {}}
initial = robot.get_joint_positions().copy()
for name, joint in [('LeftWrist', 'Joint7_R'), ('RightWrist', 'Joint7_L')]:
    robot.set_joint_positions(initial)
    robot.apply_action(ArticulationAction(joint_positions=initial))
    for _ in range(10):
        world.step(render=True)
    before = frames()
    target = initial.copy()
    index = robot.dof_names.index(joint)
    target[index] += 0.12
    robot.apply_action(ArticulationAction(joint_positions=target))
    for _ in range(60):
        world.step(render=True)
    after = frames()
    change = float(np.max(np.abs(after[name][0] - before[name][0])))
    relative_error = float(np.max(np.abs(after[name][1] - before[name][1])))
    other_change = max(float(np.max(np.abs(after[n][0] - before[n][0]))) for n in specs if n != name)
    joint_change = float(robot.get_joint_positions()[index] - initial[index])
    assert abs(joint_change) > .05, joint_change
    assert change > .01, change
    assert relative_error < 1e-5, relative_error
    assert other_change < 1e-4, other_change
    report['cases'][name] = {'joint': joint, 'joint_motion_rad': joint_change,
        'camera_world_matrix_change': change, 'camera_parent_matrix_error': relative_error,
        'other_camera_max_matrix_change': other_change}
report['result'] = 'PASS'
(ROOT / 'camera_motion_checks.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2), flush=True)
app.close()
