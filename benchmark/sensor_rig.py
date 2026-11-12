"""Five Generic RGBD CAD assemblies, RGB optical frames, no support geometry."""
import json
from pathlib import Path
import numpy as np
import trimesh
from pxr import UsdGeom, Gf, Sdf
from robot_model import Robot, origin
from usd_scene import xform

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'assets/rgbd_camera'
RGB_FOV = (86.0, 55.0)
RESOLUTION = (1920, 1080)

def look_at(eye, target, up=(0, 0, 1)):
    return np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse()).T

def sensor_geometry():
    model = Robot(MODEL / 'robot.urdf')
    frames = model.fk()
    # Official ROS color optical frame (+Z forward, +Y down) -> USD camera.
    optical = frames['camera_color_optical_frame'] @ np.diag([1, -1, -1, 1])
    parts = []
    # Optical/depth frame meshes duplicate IR/RGB lenses; instantiate each physical part once.
    for name in ['camera_link', 'camera_botter_screw_frame', 'camera_color_frame', 'camera_left_ir_frame', 'camera_right_ir_frame']:
        visual = model.links[name].find('visual')
        filename = visual.find('geometry/mesh').get('filename').replace('${mesh_path}', '')
        mesh = trimesh.load(MODEL / filename, process=False)
        mesh.apply_transform(np.linalg.inv(optical) @ frames[name] @ origin(visual.find('origin')))
        color = [.035, .043, .050] if name in ['camera_link', 'camera_botter_screw_frame'] else [.018, .035, .046]
        parts.append((name, mesh, color))
    return parts, optical

def add_sensors(a, robot, frames, root):
    stage = a.s
    records = {}
    basepath = '/World/Robot/DualArm/Links/'
    parts, optical = sensor_geometry()
    def camera(name, parent, parent_world, eye, target, kind, up=(0, 0, 1)):
        world = look_at(eye, target, up)
        rig = parent + '/' + name
        group = UsdGeom.Xform.Define(stage, rig)
        xform(group, np.linalg.inv(parent_world) @ world)
        for part, mesh, color in parts:
            a.mesh(rig + '/SensorBody/' + part, mesh.vertices, mesh.faces, color,
                   normals=mesh.vertex_normals, metal=.25, rough=.35)
        cam = UsdGeom.Camera.Define(stage, rig + '/Camera')
        focal = 18 / np.tan(np.deg2rad(RGB_FOV[0] / 2))
        vertical = 2 * focal * np.tan(np.deg2rad(RGB_FOV[1] / 2))
        cam.CreateFocalLengthAttr(float(focal))
        cam.CreateHorizontalApertureAttr(36)
        cam.CreateVerticalApertureAttr(float(vertical))
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 20))
        cam.CreateFocusDistanceAttr(float(np.linalg.norm(np.array(target) - eye)))
        cam.GetPrim().CreateAttribute('sensor:role', Sdf.ValueTypeNames.String).Set(kind)
        cam.GetPrim().CreateAttribute('sensor:model', Sdf.ValueTypeNames.String).Set('Generic RGBD')
        width, height = RESOLUTION
        fx = width / (2 * np.tan(np.deg2rad(RGB_FOV[0] / 2)))
        fy = height / (2 * np.tan(np.deg2rad(RGB_FOV[1] / 2)))
        records[name] = {
            'model': 'Generic RGBD', 'prim_path': str(cam.GetPath()), 'parent_body': parent,
            'role': kind, 'resolution': [width, height], 'focal_length_mm': float(focal),
            'horizontal_fov_deg': RGB_FOV[0], 'vertical_fov_deg': RGB_FOV[1],
            'K': [[fx, 0, width/2], [0, fy, height/2], [0, 0, 1]],
            'T_world_camera_usd': world.tolist(),
            'T_parent_camera_usd': (np.linalg.inv(parent_world) @ world).tolist(),
            'T_world_camera_opencv': (world @ np.diag([1, -1, -1, 1])).tolist(),
            'T_camera_link_rgb_usd': optical.tolist(),
            'eye_m': list(map(float, eye)), 'look_at_m': list(map(float, target)),
            'optical_convention': 'USD: -Z forward, +Y up; OpenCV: +Z forward, +Y down',
            'body_dimensions_m': [.090, .025, .030],
            'nominal_depth': {'resolution': [1280, 800], 'fov_deg': [91, 66], 'range_m': [.15, 10], 'baseline_m': .05},
            'simulation': 'Nominal RGB pinhole; RGB-aligned ideal geometric depth. No stereo matching, distortion, noise or IR interference model.',
        }
    # +X in the native hand frame is toward the palm, +Z toward the fingers.
    # Place the camera immediately below the wrist/palm root, not over the dorsum.
    for name, chain, side in [('LeftWrist', 'R', 'left'), ('RightWrist', 'L', 'right')]:
        hand = frames[side + '_hand_base']
        eye = (hand @ np.array([.055, 0, -.030, 1]))[:3]
        target = np.array([0, .43, .822])
        camera(name, basepath+'Link7_'+chain, frames['Link7_'+chain], eye, target,
               'palm_side_wrist_detail', up=-hand[:3, 0])
    # Reference image: compact camera just above the central shoulder mount.
    shoulder_mid = (frames['Base_L'][:3,3] + frames['Base_R'][:3,3]) / 2
    camera('Head', basepath+robot.root, frames[robot.root], [0, float(shoulder_mid[1]), 1.43],
           [0, .32, .795], 'global_table')
    # Beside the table edge, aimed inward; no poles, tripods, brackets or feet.
    for name, sign in [('LeftSide', -1), ('RightSide', 1)]:
        parent = '/World/TableSideCameras/' + name
        UsdGeom.Xform.Define(stage, parent)
        camera(name, parent, np.eye(4), [sign*1.10, .52, 1.18],
               [0, -.05, .795], 'global_cross_view')
    (root/'cameras.json').write_text(json.dumps(records, indent=2))
    return records
