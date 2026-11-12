"""Existing robot CAD/FK, bounded arm IK, and towel surface geometry."""
import json
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
from robot_model import Robot, origin, transform
from .config import ROOT, SPEC

PALM_DOWN = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])

@lru_cache(None)
def robot_config():
    config = json.loads((ROOT / 'poses.json').read_text())['robots']['DualArm']
    return Robot(ROOT / config['urdf']), config

class ArmIK:
    def __init__(self, side):
        robot, config = robot_config()
        chain = {'left': 'R', 'right': 'L'}[side]
        self.names = [f'Joint{k}_{chain}' for k in range(1, 8)]
        joints = [next(j for j in robot.joints if j.get('name') == n) for n in self.names]
        zero = robot.fk({}, np.array(config['base_transform']))
        self.base = zero['Base_' + chain]
        self.origins = [origin(j.find('origin')) for j in joints]
        self.axes = [np.fromstring(j.find('axis').get('xyz'), sep=' ') for j in joints]
        self.tip = np.linalg.inv(zero['Link7_' + chain]) @ zero[side + '_hand_base']
        self.lo = np.array([float(j.find('limit').get('lower')) + .005 for j in joints])
        self.hi = np.array([float(j.find('limit').get('upper')) - .005 for j in joints])
        self.q = np.array([config['joint_positions'][n] for n in self.names])

    def fk(self, q):
        T = self.base.copy(); positions = []; axes = []
        for o, axis, angle in zip(self.origins, self.axes, q):
            T = T @ o; positions.append(T[:3, 3].copy()); axes.append(T[:3, :3] @ axis)
            R = np.eye(4); R[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix(); T = T @ R
        return T @ self.tip, np.array(positions), np.array(axes)

    def solve(self, position, rotation=PALM_DOWN, trials=3):
        best = None; rng = np.random.default_rng(7)
        for trial in range(trials):
            q = self.q.copy() if trial == 0 else rng.uniform(self.lo, self.hi)
            for _ in range(150):
                T, p, a = self.fk(q)
                pe = np.asarray(position) - T[:3, 3]
                re = Rotation.from_matrix(rotation @ T[:3, :3].T).as_rotvec()
                if np.linalg.norm(pe) < .00015 and np.linalg.norm(re) < .002: break
                J = np.r_[np.cross(a, T[:3, 3] - p).T, .18 * a.T]
                d = J.T @ np.linalg.solve(J @ J.T + .00012 * np.eye(6), np.r_[pe, .18 * re])
                q = np.clip(q + np.clip(d, -.15, .15), self.lo, self.hi)
            T, p, a = self.fk(q)
            pe = float(np.linalg.norm(position - T[:3, 3]))
            re = float(Rotation.from_matrix(rotation @ T[:3, :3].T).magnitude())
            score = pe + .18 * re
            if best is None or score < best[0]: best = (score, q.copy(), pe, re)
            if pe < .001 and re < .01: break
        self.q = best[1]
        return dict(zip(self.names, map(float, self.q))), {'position_error_m': best[2], 'rotation_error_rad': best[3]}

def initial_pose():
    _, config = robot_config(); q = dict(config['joint_positions']); checks = {}
    for side, x in [('left', -.135), ('right', .135)]:
        update, checks[side] = ArmIK(side).solve(np.array([x, .23, .91]))
        q.update(update)
    return q, checks

def towel_surface(centerline=None):
    t = SPEC['towel']; nx = t['grid_width']; ny = t['grid_length']
    if centerline is None:
        centerline = np.c_[np.linspace(t['near_y_m'], t['near_y_m'] + t['length_m'], ny),
                           np.full(ny, t['table_height_m'] + t['thickness_m'] / 2)]
    xx = np.linspace(-t['width_m'] / 2, t['width_m'] / 2, nx)
    v = np.array([[x, y, z] for y, z in centerline for x in xx])
    faces = []
    for j in range(len(centerline) - 1):
        for i in range(nx - 1):
            a = j * nx + i; faces.extend([[a, a+1, a+nx+1], [a, a+nx+1, a+nx]])
    return v, np.asarray(faces)

@lru_cache(None)
def stl_geometry(filename):
    raw = (ROOT / 'assets/dual_arm' / filename).read_bytes()
    n = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84 + 50 * n: raise ValueError('Expected binary STL')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    tris = np.frombuffer(raw, dtype=dtype, offset=84, count=n)['vertices'].astype(float)
    # Vertex clustering keeps a complete closed-looking CAD surface, not sparse triangles.
    cell = .003 if 'pedestal' in filename else (.0014 if n > 10000 else .0008)
    quantized = np.round(tris.reshape(-1, 3) / cell).astype(np.int32)
    keys, inverse = np.unique(quantized, axis=0, return_inverse=True)
    idx = inverse.reshape(-1, 3)
    idx = idx[(idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])]
    _, unique = np.unique(np.sort(idx, axis=1), axis=0, return_index=True)
    return (keys * cell)[idx[np.sort(unique)]]

def cad_meshes(q):
    robot, config = robot_config(); frames = robot.fk(q, np.array(config['base_transform']))
    for name, link in robot.links.items():
        for vis in link.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None: continue
            tris = stl_geometry(mesh.get('filename')).copy()
            tris *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            T = frames[name] @ origin(vis.find('origin'))
            tris = tris @ T[:3, :3].T + T[:3, 3]
            mat = vis.find('material'); color = [.58, .61, .66]
            if mat is not None:
                c = mat.find('color')
                if c is None:
                    m = robot.xml.find("material[@name='%s']" % mat.get('name', ''))
                    c = m.find('color') if m is not None else None
                if c is not None: color = np.fromstring(c.get('rgba'), sep=' ')[:3]
            yield name, tris, np.asarray(color)
