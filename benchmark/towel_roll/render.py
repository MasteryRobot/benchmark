"""CPU rasterization of the existing CAD and deformable mesh (not Isaac RGBD)."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from .geometry import cad_meshes, towel_surface
from .config import FOLDER, SPEC

def shaded(triangles, color):
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    light = np.array([-.3, -.5, .81]); light /= np.linalg.norm(light)
    intensity = .50 + .50 * np.abs(normal @ light)
    return np.clip(intensity[:, None] * color, 0, 1)

def draw(ax, q, centerline=None, detail=False):
    z = SPEC['towel']['table_height_m']
    theta = np.linspace(0, np.pi, 100)
    outer = np.c_[1.04*np.cos(theta), -.22 + 1.10*np.sin(theta), np.full(len(theta), z)]
    inner = np.c_[.40*np.cos(theta[::-1]), -.22+.36*np.sin(theta[::-1]), np.full(len(theta), z)]
    # The annular half-table is split into quads to retain the robot opening.
    ring = np.c_[.40*np.cos(theta), -.22+.36*np.sin(theta), np.full(len(theta), z)]
    polys = np.array([[outer[i], outer[i+1], ring[i+1], ring[i]] for i in range(len(theta)-1)])
    all_triangles = [np.concatenate([polys[:,[0,1,2]],polys[:,[0,2,3]]])]
    all_colors = [np.tile(np.array([.15,.23,.27]), (len(all_triangles[0]),1))]
    for name, tris, color in cad_meshes(q):
        if detail and (name.startswith('Base') or name == 'pedestal_link'): continue
        if detail:
            mid=tris.mean(axis=1)
            tris=tris[(abs(mid[:,0])<.42)&(mid[:,1]>.12)&(mid[:,1]<.8)&(mid[:,2]<1.17)]
        all_triangles.append(tris); all_colors.append(shaded(tris,color))
    vertices, faces = towel_surface(centerline); triangles = vertices[faces]
    colors = np.tile(np.array(SPEC['towel']['color']), (len(faces), 1))
    nx = SPEC['towel']['grid_width']; ny = len(vertices)//nx
    for i, f in enumerate(faces):
        x = f[0] % nx; y = f[0] // nx
        if x in (0, 1, nx-3, nx-2) or y in (0, 1, ny-3, ny-2): colors[i] = [.72, .93, .91]
        elif (x+y) % 2: colors[i] *= .94
    all_triangles.append(triangles); all_colors.append(shaded(triangles,colors))
    # One collection sorts individual faces jointly; separate collections occlude the towel.
    ax.add_collection3d(Poly3DCollection(np.concatenate(all_triangles),facecolors=np.concatenate(all_colors),edgecolors='none',linewidths=0,antialiaseds=False,zsort='average'))
    if detail:
        ax.set(xlim=(-.38,.38), ylim=(.13,.78), zlim=(.78,1.17)); ax.set_box_aspect((.76,.65,.39))
        ax.view_init(elev=47, azim=68)
    else:
        ax.set(xlim=(-.9,.9), ylim=(-.28,.90), zlim=(.66,1.9)); ax.set_box_aspect((1.8,1.18,1.24))
        ax.view_init(elev=29, azim=65)
    ax.set_axis_off(); ax.set_proj_type('ortho'); ax.set_facecolor('#edf3f5')

def render_preview(q, path=None, centerline=None, title='BIMANUAL TOWEL ROLL | INITIAL SCENE'):
    path = path or FOLDER/'images/scene_preview.png'; path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(16,8), facecolor='#edf3f5')
    draw(fig.add_axes([0,.08,.54,.81], projection='3d'), q, centerline)
    draw(fig.add_axes([.48,.08,.52,.81], projection='3d'), q, centerline, detail=True)
    fig.text(.035,.94,title,color='#183440',fontsize=22,weight='bold')
    fig.text(.035,.898,'Existing 54-DOF dual-arm / left + right dexterous hands / 36 x 40 cm towel',color='#436473',fontsize=12)
    fig.text(.035,.045,'CPU CAD + mesh preview  |  Not an Isaac sensor image  |  Physical success pending',color='#9a5a14',fontsize=12)
    fig.savefig(path,dpi=110,facecolor=fig.get_facecolor()); plt.close(fig)
    return path
