"""Local scenes for the provisional task inventory; fidelity is explicit."""
import math
from .catalog import DATASET, dataset_rows
from .recipes import TABLE, body, goal, cards, cylinder, cube
from .props import prop, cup, open_box


def object_at(name, kind, x, y, dynamic=True, z=None):
    parts, mass, offset = prop(kind)
    b = body(name, [x,y,TABLE+offset+.0002 if z is None else z], parts, mass if dynamic else None)
    b['prop_kind'] = kind
    return b


def dataset_scenes():
    result = []
    for row in dataset_rows():
        name = row['source_task']
        if name == 'deal_playing_cards':
            spec = cards(); spec['id'] = 'sharpa_dex_' + name
        else:
            props = [object_at(f'object_{i+1}',kind,-.38+(i%3)*.22,.35+(i//3)*.20)
                     for i,kind in enumerate(row['props'])]
            supports = [object_at(f'support_{i+1}',kind,.18+(i%2)*.21,.49,False)
                        for i,kind in enumerate(row['supports'])]
            spec = dict(id='sharpa_dex_'+name,title=row['title'],task_type='scene_only',
                        bodies=props+supports,goals=[],limitations=[])
            if not row['missing']:
                spec['task_type'] = 'place_all'
                targets = []
                for i,b in enumerate(props):
                    k = b['prop_kind']; offset = prop(k)[2]
                    x,y,z = .20+(i%2)*.06,.49,TABLE+offset+.006
                    targets.append(goal(b['id'],[x,y,z],rotation_tolerance_deg=180 if k in ('ball','pill','toy','fruit','apple','tennis_ball') else 12))
                if name == 'insert_batteries_into_charger':
                    for i,b in enumerate(props):
                        # Cylinders lie in the open charging channels.
                        b['orientation_wxyz']=[math.sqrt(.5),math.sqrt(.5),0,0]
                        b['position'][2]=TABLE+.0072
                        targets[i].update(position=[.18+(-1 if i==0 else 1)*.017,.49,TABLE+.012],
                                          orientation_wxyz=b['orientation_wxyz'],position_tolerance_m=.004)
                elif name == 'empty_dustpan_into_bin':
                    # Place the waste in the pan initially; only waste is scored.
                    for i,b in enumerate(props[1:]):
                        b['position']=[props[0]['position'][0]+(-.022 if i==0 else .022),.35,TABLE+.0227]
                    targets=targets[1:]
                    for i,t in enumerate(targets):
                        t['position']=[.18+(-.025 if i==0 else .025),.49,TABLE+.0225]
                        t['require_prior_hand_contact']=False
                    spec['limitations'].append('Checks deposited rigid waste; does not require a particular pan trajectory.')
                elif name == 'cover_ball_with_cup':
                    props[1]['position']=[.18,.49,TABLE+.0182]
                    targets=[goal('object_1',[.18,.49,TABLE+.085],orientation_wxyz=[0,1,0,0])]
                elif name == 'insert_knife_into_cutlery_bin':
                    targets[0].update(position=[.18,.49,TABLE+.0902],orientation_wxyz=[math.sqrt(.5),math.sqrt(.5),0,0],
                                      position_tolerance_m=.025,rotation_tolerance_deg=35)
                elif name == 'place_cup_on_plate':
                    targets[0]['position']=[.18,.49,TABLE+.003]
                elif name == 'place_knife_and_fork_on_plate':
                    supports[0]['parts']=cup(.11,.012,.003,[.84,.88,.90])
                    for i,t in enumerate(targets):t['position']=[.18+(-.025 if i==0 else .025),.49,TABLE+.012]
                elif name == 'stack_two_plates':
                    targets[0]['position']=[.18,.49,TABLE+.012]
                elif name == 'remove_block_from_box':
                    props[0]['position']=[.18,.49,TABLE+.0227]
                    targets[0]['position']=[-.25,.39,TABLE+.0175]
                elif name == 'reorient_cup_upright':
                    props[0]['orientation_wxyz']=[0,1,0,0]; props[0]['position'][2]=TABLE+.0852
                    targets[0]['position']=[.20,.49,TABLE]
                elif name == 'relocate_tennis_ball':
                    targets[0]['position']=[.20,.49,TABLE+.033]
                elif name == 'place_fruits_in_basket':
                    for i,t in enumerate(targets):t['position']=[.18+(-.045 if i==0 else .045),.49,TABLE+.005+prop(props[i]['prop_kind'])[2]]
                elif name == 'place_pens_in_holder':
                    for i,t in enumerate(targets):t.update(position=[.18+(-.012 if i==0 else .012),.49,TABLE+.078],
                                                           rotation_tolerance_deg=30,position_tolerance_m=.018)
                elif name == 'transfer_pills_between_cups':
                    for i,b in enumerate(props):b['position']=[.18+(-.012 if i==0 else .012),.49,TABLE+.0092]
                    for i,t in enumerate(targets):t['position']=[.39+(-.012 if i==0 else .012),.49,TABLE+.009]
                elif name == 'reorient_and_relocate_carton':
                    props[0]['orientation_wxyz']=[math.sqrt(.5),math.sqrt(.5),0,0];props[0]['position'][2]=TABLE+.0277
                    targets[0]['position']=[.20,.49,TABLE+.065]
                elif name == 'sort_utensils':
                    for support in supports: support['parts']=open_box(.19,.22,.018)
                    supports[1]['position']=[.42,.49,TABLE+.0002]
                    for i,t in enumerate(targets):t['position']=[.18 if i==0 else .40+(i-1)*.04,.49,TABLE+.0122]
                elif name == 'reposition_and_stack_blocks':
                    for i,t in enumerate(targets):t['position']=[.18,.49,TABLE+.0175+i*.035]
                elif name == 'collect_toys_into_basket':
                    for i,t in enumerate(targets):t['position']=[.18+(-.04 if i==0 else .04),.49,TABLE+.030]
                spec['goals']=targets
            elif name == 'assemble_gears_on_base':
                supports[0]['parts'] += [cylinder(.010,.025,[x,0,.0175]) for x in [-.035,.035]]
            elif name == 'play_tic_tac_toe':
                for axis in (0,1):
                    for sign in (-1,1):
                        pos=[0,0,.0055];pos[axis]=sign*.025
                        size=[.15,.001,.001] if axis else [.001,.13,.001]
                        supports[0]['parts'].append(cube(size,pos,[.9,.9,.9]))
        spec.update(source_task=name,source_group='provisional_tabletop_inventory',official_source=None,
                    source_url_candidate=DATASET, source_status='UNVERIFIED_PROVISIONAL_TASK',
                    title=row['title']+'（本地仿真原型）')
        if row['missing'] and name!='deal_playing_cards':
            spec['limitations'] += ['Not implemented: '+row['missing']+'.',
                                    'Physical props and reset/control available; no task-success predicate until missing process is implemented.']
        spec['coverage_level'] = 'RIGID_BODY_TASK_APPROXIMATION' if spec['task_type']!='scene_only' else 'SCENE_ASSETS_ONLY'
        result.append(spec)
    return result
