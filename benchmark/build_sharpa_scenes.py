"""Write portable USDA prototype layers without requiring USD Python bindings.

USDA authoring is possible on CPU; composition and physics remain separate
checks. Reuses the existing shared robot, tabletop, light and five-camera rig.
"""
import argparse
import json
from pathlib import Path
import os
import tempfile
from sharpa.contract import ROOT, validate
from sharpa.recipes import scenarios
from sharpa.visual_assets import binding, visual_lines


def atom(v):
    if isinstance(v, bool):
        return str(v).lower()
    if isinstance(v, str):
        return json.dumps(v, ensure_ascii=False)
    if isinstance(v, (list, tuple)):
        return '(' + ', '.join(atom(x) for x in v) + ')'
    return format(float(v), '.12g')


def atomic_write(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=path.parent, encoding='utf-8', delete=False) as f:
        f.write(value); tmp = f.name
    os.replace(tmp, path)


def shape_lines(part, i, parent, indent='            ', collision=True, hidden=False):
    kind = part['kind']; cls = dict(cube='Cube', sphere='Sphere', cylinder='Cylinder')[kind]
    schemas = ' (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"])' if collision else ''
    lines = [f'{indent}def {cls} "part_{i}"{schemas}', indent + '{']
    def put(typ, key, value):
        lines.append(f'{indent}    {typ} {key} = {atom(value)}')
    put('double3', 'xformOp:translate', part['position'])
    put('double', 'xformOp:rotateZ', part.get('yaw_deg', 0))
    ops = ['xformOp:translate', 'xformOp:rotateZ']
    if kind == 'cube':
        put('double', 'size', 1); put('float3', 'xformOp:scale', part['size']); ops.append('xformOp:scale')
    else:
        put('double', 'radius', part['size'][0])
        if kind == 'cylinder':
            put('double', 'height', part['size'][1]); put('uniform token', 'axis', part.get('axis', 'Z'))
    lines.append(f'{indent}    uniform token[] xformOpOrder = {json.dumps(ops)}')
    lines.append(f'{indent}    color3f[] primvars:displayColor = [{atom(part["color"])}]')
    if hidden:
        put('token', 'visibility', 'invisible')
    if collision:
        put('bool', 'physics:collisionEnabled', True)
        put('float', 'physxCollision:contactOffset', parent.get('contact_offset_m', .0003))
        put('float', 'physxCollision:restOffset', 0)
        lines.append(f'{indent}    rel material:binding:physics = </World/SharpaLooks/{parent["id"]}_Physics>')
        lines.append(f'{indent}    rel material:binding = </World/SharpaLooks/{parent["id"]}_{i}>')
    lines.append(indent + '}')
    return lines


def layer(spec):
    out = ['#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1',
           '    upAxis = "Z"', '    timeCodesPerSecond = 240',
           '    subLayers = [@../../../tasks/workcell.usdc@]', ')',
           'def Xform "World"', '{', '    def Scope "SharpaLooks"', '    {']
    for b in spec['bodies']:
        out += [f'        def Material "{b["id"]}_Physics" (prepend apiSchemas = ["PhysicsMaterialAPI"])',
                '        {', '            float physics:staticFriction = 0.8',
                '            float physics:dynamicFriction = 0.65',
                f'            float physics:restitution = {atom(b.get("restitution", .01))}', '        }']
        for i, part in enumerate(b['parts']):
            name = f'{b["id"]}_{i}'
            out += [f'        def Material "{name}"', '        {',
                    f'            token outputs:surface.connect = </World/SharpaLooks/{name}/Surface.outputs:surface>',
                    '            def Shader "Surface"', '            {',
                    '                uniform token info:id = "UsdPreviewSurface"',
                    f'                color3f inputs:diffuseColor = {atom(part["color"])}',
                    '                float inputs:roughness = 0.5',
                    '                token outputs:surface', '            }', '        }']
    out += ['    }', '    def Xform "Task"', '    {',
            f'        custom string task:id = {atom(spec["id"])}',
            '        custom string task:provenance = "UNVERIFIED_LOCAL_PROTOTYPE"']
    for b in spec['bodies']:
        schemas = ' (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"])' if b['dynamic'] else ''
        out += [f'        def Xform "{b["id"]}"{schemas}', '        {',
                f'            double3 xformOp:translate = {atom(b["position"])}',
                f'            quatd xformOp:orient = {atom(b["orientation_wxyz"])}',
                '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]']
        if b['dynamic']:
            out += [f'            float physics:mass = {atom(b["mass_kg"])}',
                    '            bool physics:rigidBodyEnabled = true',
                    '            bool physxRigidBody:enableCCD = true',
                    '            int physxRigidBody:solverPositionIterationCount = 32',
                    '            int physxRigidBody:solverVelocityIterationCount = 16',
                    f'            float physxRigidBody:angularDamping = {atom(b.get("angular_damping", .5))}']
        visual = binding(b)
        for i, part in enumerate(b['parts']):
            out += shape_lines(part, i, b, hidden=visual is not None)
        out += visual_lines(b)
        camera = spec.get('capture_camera')
        if camera and camera['body'] == b['id']:
            out += ['            def Camera "CaptureCamera"', '            {',
                    f'                double3 xformOp:translate = {atom(camera["position"])}',
                    f'                quatd xformOp:orient = {atom(camera["orientation_wxyz"])}',
                    '                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]',
                    f'                float focalLength = {atom(camera["focal_length_mm"])}',
                    '                float horizontalAperture = 36', '                float verticalAperture = 20.25',
                    '                float2 clippingRange = (0.01, 20)', '            }']
        out += ['        }']
    out += ['        def Xform "Goals"', '        {']
    for target in spec['goals']:
        # Small cross and orientation axes; no collision or rigid-body schema.
        points = [[-.025, 0, 0], [.025, 0, 0], [0, -.025, 0], [0, .025, 0], [0, 0, 0], [0, 0, .035]]
        out += [f'            def BasisCurves "{target["body"]}"', '            {',
                f'                double3 xformOp:translate = {atom(target["position"])}',
                f'                quatd xformOp:orient = {atom(target["orientation_wxyz"])}',
                '                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]',
                '                uniform token type = "linear"', '                uniform token wrap = "nonperiodic"',
                '                int[] curveVertexCounts = [2, 2, 2]',
                '                point3f[] points = [' + ', '.join(atom(p) for p in points) + ']',
                '                float[] widths = [0.001]',
                '                color3f[] primvars:displayColor = [(0.1, 0.9, 0.7)]', '            }']
    out += ['        }', '    }', '    over "PhysicsScene"', '    {',
            '        bool physxScene:enableCCD = true', '    }', '}']
    return '\n'.join(out) + '\n'


def build(root=ROOT):
    root = Path(root)
    if not (root / 'tasks/workcell.usdc').is_file():
        raise FileNotFoundError('Missing existing tasks/workcell.usdc')
    rows = scenarios()
    for spec in rows:
        validate(spec)
        folder = root / 'sharpa/scenes' / spec['id']
        atomic_write(folder / 'scene.usda', layer(spec))
        atomic_write(folder / 'task.json', json.dumps(spec, ensure_ascii=False, indent=2) + '\n')
    atomic_write(root / 'sharpa/index.json', json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=ROOT)
    args = parser.parse_args()
    rows = build(args.output_root)
    print(f'Built {len(rows)} local prototype layers. Official coverage and live simulation NOT verified.')
