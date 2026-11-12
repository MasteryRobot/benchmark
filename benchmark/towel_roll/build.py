"""Build the portable USD rest scene and show its actual CAD mesh preview."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .config import ROOT, FOLDER, SPEC
from .geometry import initial_pose, robot_config, towel_surface
from .raster import render_preview

def tup(v): return '(' + ', '.join(f'{float(x):.9g}' for x in v) + ')'

def build():
    FOLDER.mkdir(exist_ok=True); q, checks = initial_pose()
    vertices, faces = towel_surface()
    lines = ['#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"',
             '    subLayers = [@../tasks/workcell.usdc@]', ')', 'over "World"', '{',
             '    def Xform "TowelRoll"', '    {',
             '        custom string task:id = "bimanual_roll_towel"',
             '        custom string task:physicsScope = "Rest mesh only in USD; CPU reduced-order dynamics in towel_roll/explore.py. No Isaac cloth adapter yet."',
             '        def Mesh "Towel"', '        {',
             '            bool doubleSided = true', '            uniform token subdivisionScheme = "none"',
             '            point3f[] points = [' + ', '.join(tup(v) for v in vertices) + ']',
             '            int[] faceVertexCounts = [' + ', '.join(['3']*len(faces)) + ']',
             '            int[] faceVertexIndices = [' + ', '.join(str(i) for i in faces.ravel()) + ']',
             '            color3f[] primvars:displayColor = [(0.12, 0.63, 0.68)]',
             '        }', '    }', '    over "Robot"', '    {', '        over "DualArm"', '        {', '            over "Links"', '            {']
    robot, config = robot_config(); frames = robot.fk(q, np.array(config['base_transform']))
    for group in sorted(set(robot.groups.values())):
        T = frames[group]; xyzw = Rotation.from_matrix(T[:3,:3]).as_quat()
        lines += [f'                over "{group}"', '                {',
                  '                    double3 xformOp:translate = '+tup(T[:3,3]),
                  '                    quatd xformOp:orient = '+tup([xyzw[3],*xyzw[:3]]),
                  '                }']
    lines += ['            }', '            over "Joints"', '            {']
    for name, val in q.items():
        lines += [f'                over "{name}"', '                {',
                  f'                    float drive:angular:physics:targetPosition = {np.rad2deg(val):.9g}',
                  f'                    float state:angular:physics:position = {np.rad2deg(val):.9g}', '                }']
    lines += ['            }', '        }', '    }', '}']
    (FOLDER/'scene.usda').write_text(chr(10).join(lines)+chr(10))
    (FOLDER/'task.json').write_text(json.dumps(SPEC,ensure_ascii=False,indent=2)+chr(10))
    (FOLDER/'initial_pose.json').write_text(json.dumps({'joint_targets_rad':q,'ik':checks},indent=2)+chr(10))
    print(json.dumps({'ik':checks,'preview':str(render_preview(q))}),flush=True)

if __name__ == '__main__': build()
