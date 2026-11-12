"""Additional demonstration candidates. Provenance must be confirmed."""
from .recipes import TABLE, body, goal, cube, cylinder, ring
from .dataset_scenes import object_at


def demo_scenes():
    specs = []
    def add(name,title,objects,supports,missing,goals=None,task_type='scene_only'):
        bs=[object_at(f'object_{i+1}',kind,-.27+(i%3)*.13,.35+(i//3)*.15)
            for i,kind in enumerate(objects)]
        bs += [object_at(f'support_{i+1}',kind,.22+(i%2)*.23,.51,False)
               for i,kind in enumerate(supports)]
        spec=dict(id='sharpa_demo_'+name,title=title+'（本地原型）',
                  source_group='public_demo_candidates',source_task=name,
                  bodies=bs,goals=goals or [],task_type=task_type,
                  limitations=[missing],source_status='UNVERIFIED_DEMO_CANDIDATE',
                  coverage_level='SCENE_ASSETS_ONLY' if task_type=='scene_only' else 'RIGID_BODY_TASK_APPROXIMATION')
        specs.append(spec);return spec
    add('pen_spinning','手内转笔',['pen'],[],
        'Pen geometry and two-hand contact observation available; continuous in-hand turn counting and grasp transitions are not implemented.')
    add('bartending','调饮与摇壶',['bottle','shaker','spoon'],['cup','cup'],
        'Rigid vessels and tools only; no fluid, pouring volume or shaker seal.')
    gpu=add('gpu_installation','显卡插装',['gpu'],['board'],
        'Rigid card insertion alignment only; connector resistance, latch and electrical function are not modeled.',
        [goal('object_1',[.22,.51,TABLE+.0744],position_tolerance_m=.004)],'place_all')
    gpu['bodies'][1]['parts'] += [cube([.19,.006,.014],[0,s*.009,.012],[.12,.15,.18]) for s in (-1,1)]
    add('apple_peeling','苹果削皮',['apple','peeler'],['plate'],
        'No peel separation, cutting, deformable skin or volume-loss metric.')
    add('dessert_service','甜品取杯与加料',['cup','spoon','blueberry','blueberry'],['tray','jar'],
        'Local service layout; no ice cream/fluid dispensing, order system or vendor equipment.')
    add('paper_cutting','剪纸',['paper'],['tray'],
        'Two independent scissor blades are shown as tool components; no revolute scissors or paper fracture.')
    scissors=specs[-1]
    blades=[cube([.008,.10,.003],[0,.045,0],[.72,.74,.77])]+ring(.015,.004,.006,[.3,.44,.6])
    for i in range(2):
        scissors['bodies'].append(body(f'scissor_blade_{i+1}',[-.12+i*.05,.39,TABLE+.0032],blades,.025))
    add('needle_threading','穿针引线',['cable'],['board'],
        'Eyelet geometry and cable proxy only; no deformable thread/needle insertion predicate.')
    specs[-1]['bodies'][1]['parts'] += [dict(p,position=[p['position'][0],p['position'][1],.012])
                                      for p in ring(.006,.002,.004,[.7,.72,.75])]
    add('screwdriver_use','螺丝刀拧装',['screwdriver','nail'],['board'],
        'Rigid tools only; thread coupling, torque and insertion depth not implemented.')
    return specs
