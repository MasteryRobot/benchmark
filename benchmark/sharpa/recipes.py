"""Locally designed candidates, NOT an official Sharpa task list or CAD.

Metres, kilograms, radians, Z-up. Props use simple analytic compound colliders.
Dimensions/masses are explicit simulation design estimates. No vendor data is
claimed. Extend SCENARIOS only after checking the supplied release/task list.
"""
from copy import deepcopy
import math

TABLE = .795


def shape(kind, size, xyz=(0, 0, 0), color=(.22, .48, .65), **kw):
    return dict(kind=kind, size=list(size), position=list(xyz), color=list(color), **kw)


def cube(size, xyz=(0, 0, 0), color=(.22, .48, .65)):
    return shape('cube', size, xyz, color)


def cylinder(radius, height, xyz=(0, 0, 0), color=(.6, .62, .64), axis='Z'):
    return shape('cylinder', [radius, height], xyz, color, axis=axis)


def body(name, position, parts, mass=None, **kw):
    return dict(id=name, position=list(position), orientation_wxyz=[1, 0, 0, 0],
                parts=parts, dynamic=mass is not None, mass_kg=mass, **kw)


def goal(name, position, contact='released', **kw):
    result = dict(body=name, position=list(position), orientation_wxyz=[1, 0, 0, 0],
                position_tolerance_m=.012, rotation_tolerance_deg=12,
                contact=contact, require_prior_hand_contact=True)
    result.update(kw)
    return result


def ring(radius, thickness, height, color):
    # Compound boxes preserve the central hole; never use a filled convex hull.
    result = []
    for i in range(24):
        a = i * 2 * math.pi / 24
        p = cube([thickness, 2 * radius * math.tan(math.pi / 24) * 1.03, height],
                 [radius * math.cos(a), radius * math.sin(a), 0], color)
        p['yaw_deg'] = math.degrees(a)
        result.append(p)
    return result


def cards():
    bodies, goals = [], []
    for i in range(5):
        name = f'card_{i+1}'
        bodies.append(body(name, [-.25, .37, TABLE + .0008 + i * .0018],
                           [cube([.063, .088, .0014], color=[.86, .88, .92])], .0018,
                           contact_offset_m=.0001))
        goals.append(goal(name, [-.30 + i * .15, .60, TABLE + .0007]))
    return dict(id='sharpa_card_dealing', title='多张纸牌分发（本地原型）',
                task_type='place_all', bodies=bodies, goals=goals,
                limitations=['Five rigid cards; bending, shuffling and full deck friction are not modeled.'])


def windmill():
    rotor = ring(.0065, .002, .006, [.82, .62, .14])
    for x, y, size in [(.035, 0, [.052, .018, .003]), (-.035, 0, [.052, .018, .003]),
                       (0, .035, [.018, .052, .003]), (0, -.035, [.018, .052, .003])]:
        rotor.append(cube(size, [x, y, 0], [.25, .68, .65]))
    return dict(id='sharpa_windmill_assembly', title='风车转子与套环装配（本地原型）',
                task_type='place_all', bodies=[
                    body('stand', [.18, .45, TABLE], [cube([.10, .10, .012], [0, 0, .006]),
                         cylinder(.01, .030, [0, 0, .027]), cylinder(.004, .025, [0, 0, .0545])]),
                    body('rotor', [-.23, .40, TABLE + .0031], rotor, .024),
                    body('collar', [-.11, .54, TABLE + .0021], ring(.006, .002, .004, [.9, .52, .12]), .005)],
                goals=[goal('rotor', [.18, .45, TABLE + .045], position_tolerance_m=.004),
                       goal('collar', [.18, .45, TABLE + .050], position_tolerance_m=.004)],
                limitations=['Horizontal axle insertion surrogate; no threads, snap-fit or powered windmill.',
                             'Free rigid rotor and collar remain separate after insertion; no automatic welding.'])


def photography():
    return dict(id='sharpa_camera_photography', title='双手持相机拍照（本地原型）',
                task_type='photograph', bodies=[
                    body('camera_body', [.23, .40, TABLE + .0401], [
                        cube([.13, .045, .08], color=[.11, .13, .16]),
                        cylinder(.025, .045, [0, .043, 0], [.19, .21, .25], axis='Y'),
                        cube([.014, .012, .006], [.045, 0, .043], [.75, .20, .16])], .32),
                    body('subject', [0, .74, TABLE], [cube([.12, .018, .18], [0, 0, .09], [.9, .55, .20])])],
                goals=[goal('camera_body', [0, .40, .96], 'both_hands')],
                capture_camera=dict(body='camera_body', position=[0, .067, 0],
                                    orientation_wxyz=[math.sqrt(.5), math.sqrt(.5), 0, 0],
                                    resolution=[640, 360], focal_length_mm=12),
                limitations=['Capture is an explicit action returning rendered RGB; shutter mechanics and autofocus are not modeled.',
                             'Success checks held pose and an actual nonempty captured frame, not image aesthetic quality.'])


def tennis():
    return dict(id='sharpa_table_tennis', title='桌面缩比乒乓球回击（本地原型）',
                task_type='return_ball', bodies=[
                    body('court', [0, .49, TABLE], [cube([.78, .48, .012], [0, 0, .006], [.14, .36, .51])]),
                    body('net', [0, .52, TABLE + .012], [cube([.74, .003, .070], [0, 0, .035], [.68, .71, .74])]),
                    body('paddle', [.20, .35, TABLE + .0211], [
                        cylinder(.060, .018, color=[.68, .12, .15]),
                        cube([.024, .080, .014], [0, -.085, 0], [.58, .35, .18])], .12),
                    body('ball', [0, .64, 1.07], [shape('sphere', [.02], color=[.96, .78, .2])], .0027,
                         launch_velocity_m_s=[0, -.65, -.1], restitution=.8, angular_damping=0)],
                goals=[], return_plane_y=.62, return_min_z=TABLE + .09, return_max_z=1.40,
                return_max_abs_x=.35, racket='paddle', ball='ball',
                limitations=['Scaled single-return drill on the existing workcell; not a regulation table or an opponent.',
                             'No aerodynamic drag or spin-dependent Magnus model. Contact impulses use the physics engine.'])


def baton():
    return dict(id='sharpa_baton_catching', title='运动棒接取（本地原型）',
                task_type='catch', bodies=[body('baton', [0, .47, 1.14],
                    [cylinder(.014, .19, axis='X', color=[.86, .48, .16])], .09,
                    launch_velocity_m_s=[0, -.20, .65])], goals=[], caught_body='baton',
                limitations=['Deterministic launch after reset; no external throwing robot or learned catcher.'])


def scenarios():
    from .dataset_scenes import dataset_scenes
    from .demo_scenes import demo_scenes
    result = [cards(), windmill(), photography(), tennis(), baton()]
    for spec in result:
        spec.update(source_group='conversation_demo_candidates',
                    coverage_level='RIGID_BODY_TASK_APPROXIMATION')
    result += dataset_scenes() + demo_scenes()
    for spec in result:
        spec.setdefault('source_status','UNVERIFIED_CONVERSATION_CANDIDATE')
        spec.setdefault('official_source',None)
        spec.update(schema_version=1, suite='sharpa_prototypes', units='m', up_axis='Z',
                    object_source='Locally authored primitives; dimensions and masses are design estimates, not vendor data.',
                    robot_model='Existing neutral 54-DOF dual arm; not a certified Sharpa robot replica.',
                    runtime=dict(physics_dt_s=1/240, control_decimation=8, settle_steps=1200,
                                 episode_timeout_s=30, success_hold_s=.25, min_contact_force_n=.001,
                                 min_contact_bodies=2, max_linear_speed_m_s=.05, max_angular_speed_rad_s=.25))
        from .visual_assets import binding
        visuals={b['id']:binding(b) for b in spec['bodies'] if binding(b) is not None}
        if visuals:
            spec['visual_assets']=visuals
            spec['object_source']='YCB CC-BY-4.0 textured scans for mapped visuals; remaining props and all collision proxies are local analytic designs. See visual_assets for per-body sources.'
    return deepcopy(result)
