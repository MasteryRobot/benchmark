"""Author a surface-deformable preparation layer; no controller or success claim.

API reference: Omni Physics 110.1 deformable_bodies.html and
deformable_migration.html. Runtime compatibility must be checked in Isaac.
"""
import math
import numpy as np
from .config import SPEC


def mesh_for_variant(variant, spacing=.0075):
    required = ('center_x_m', 'near_y_m', 'support_height_m', 'width_m', 'length_m', 'thickness_m')
    if any(not math.isfinite(variant[k]) for k in required) or not math.isfinite(spacing):
        raise ValueError('Non-finite towel geometry')
    if (spacing <= 0 or variant['support_height_m'] < 0
            or min(variant[k] for k in ('width_m', 'length_m', 'thickness_m')) <= 0):
        raise ValueError('Invalid towel dimensions')
    nx = math.ceil(variant['width_m'] / spacing) + 1
    ny = math.ceil(variant['length_m'] / spacing) + 1
    if nx * ny > 100000:
        raise ValueError('Requested cloth mesh is too large')
    x = np.linspace(-variant['width_m'] / 2, variant['width_m'] / 2, nx) + variant['center_x_m']
    y = np.linspace(variant['near_y_m'], variant['near_y_m'] + variant['length_m'], ny)
    z = SPEC['towel']['table_height_m'] + variant['support_height_m'] + variant['thickness_m'] / 2
    xx, yy = np.meshgrid(x, y)
    points = np.c_[xx.ravel(), yy.ravel(), np.full(nx * ny, z)]
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            faces.extend(((a, a + 1, a + nx + 1), (a, a + nx + 1, a + nx)))
    return points, np.asarray(faces, dtype=np.int32)


def author(stage, variant, spacing=.0075):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Vt
    try:
        from pxr import PhysxSchema
    except ImportError:
        # Standalone NVIDIA PhysX ships the same USD schema in a codeless wheel.
        from physx_usd_schemas import PhysxSchema
    points, faces = mesh_for_variant(variant, spacing)
    mesh = UsdGeom.Mesh.Get(stage, '/World/TowelRoll/Towel')
    if not mesh:
        raise RuntimeError('Base towel scene is missing its mesh')
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
    prim = mesh.GetPrim()
    # Same schema operations as NVIDIA deformableUtils' nonhierarchical helper;
    # explicit authoring also works without an Omniverse Kit application.
    for schema in ('OmniPhysicsDeformableBodyAPI', 'OmniPhysicsSurfaceDeformableSimAPI'):
        if not prim.ApplyAPI(schema):
            raise RuntimeError('Missing surface deformable schema: ' + schema)
    prim.GetAttribute('omniphysics:restShapePoints').Set(mesh.GetPointsAttr().Get())
    prim.GetAttribute('omniphysics:restTriVtxIndices').Set(
        Vt.Vec3iArray.FromNumpy(faces.astype(np.int32)))
    UsdPhysics.CollisionAPI.Apply(prim)

    def apply(prim, schema, values):
        if not prim.ApplyAPI(schema):
            raise RuntimeError('Missing schema: ' + schema)
        for key, value in values.items():
            attr = prim.GetAttribute(key)
            if not attr or not attr.Set(value):
                raise RuntimeError('Cannot author ' + key)

    mass = variant.get('mass_kg', SPEC['towel']['mass_kg'])
    youngs = variant.get('youngs_modulus_pa', 40000.)
    poisson = variant.get('poissons_ratio', .25)
    friction = variant.get('friction', .65)
    if (not all(math.isfinite(v) for v in (mass, youngs, poisson, friction))
            or mass <= 0 or youngs <= 0 or not 0 <= poisson < .5 or friction < 0):
        raise ValueError('Invalid surface material parameters')
    apply(prim, 'OmniPhysicsDeformableBodyAPI', {'omniphysics:mass': mass})
    apply(prim, 'PhysxSurfaceDeformableBodyAPI', {'physxDeformableBody:selfCollision': True,
          'physxDeformableBody:solverPositionIterationCount': 32})
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateRestOffsetAttr(variant['thickness_m'] / 2)
    collision.CreateContactOffsetAttr(variant['thickness_m'] / 2 + .001)
    material = UsdShade.Material.Define(stage, '/World/TowelRoll/SurfaceMaterial')
    # New FEM material values are explicit design assumptions, not conversions
    # of the archived PBD spring constants. They require physical calibration.
    apply(material.GetPrim(), 'OmniPhysicsBaseMaterialAPI', {'omniphysics:dynamicFriction': friction})
    apply(material.GetPrim(), 'OmniPhysicsDeformableMaterialAPI',
          {'omniphysics:youngsModulus': youngs, 'omniphysics:poissonsRatio': poisson})
    apply(material.GetPrim(), 'OmniPhysicsSurfaceDeformableMaterialAPI',
          {'omniphysics:surfaceThickness': variant['thickness_m'],
           # Zero selects the native E/thickness-derived stiffness. E/(12*(1-v²))
           # omits thickness and was an excessively stiff override, not a modulus.
           'omniphysics:surfaceBendStiffness': variant.get('bend_stiffness_override', 0.0)})
    apply(material.GetPrim(), 'PhysxSurfaceDeformableMaterialAPI',
          {'physxDeformableMaterial:elasticityDamping': variant.get('elasticity_damping', .05),
           'physxDeformableMaterial:bendDamping': variant.get('bend_damping', .05)})
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    if variant['support_height_m']:
        inset = variant.get('support_front_inset_m', -.02)
        support_length = variant['length_m'] + .02 - inset
        if support_length <= 0: raise ValueError('Support inset exceeds towel length')
        pad = UsdGeom.Cube.Define(stage, '/World/TowelRoll/SupportPad')
        pad.CreateSizeAttr(1)
        pad.AddScaleOp().Set(Gf.Vec3f(variant['width_m'] + .04, support_length,
                                    variant['support_height_m']))
        pad.AddTranslateOp().Set(Gf.Vec3d(variant['center_x_m'],
                                        variant['near_y_m'] + inset + support_length / 2,
                                        SPEC['towel']['table_height_m'] + variant['support_height_m'] / 2))
        # Explicit ordering keeps translation in world units, independent of scale.
        xform = UsdGeom.Xformable(pad)
        ops = xform.GetOrderedXformOps()
        xform.SetXformOpOrder([ops[1], ops[0]])
        UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
    return {'vertices': len(points), 'triangles': len(faces),
            'backend': 'PhysX surface deformable', 'material_status': 'UNCALIBRATED',
            'physics_started': False, 'task_success': False}
