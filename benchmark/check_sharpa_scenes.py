"""Audit all new scenes; CPU authoring, USD composition and physics are separate."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import re
import sys
import numpy as np
from build_sharpa_scenes import atomic_write, layer
from sharpa.contract import ROOT, load_scenarios, rotation
from run_sharpa_episode import fingerprint


def part_bounds(body):
    R = rotation(body['orientation_wxyz']); result = []
    for p in body['parts']:
        size=p['size'];kind=p['kind']
        if kind=='cube':extent=np.array(size)/2
        elif kind=='sphere':extent=np.repeat(size[0],3)
        else:
            extent=np.repeat(size[0],3);extent['XYZ'.index(p.get('axis','Z'))]=size[1]/2
        a=np.deg2rad(p.get('yaw_deg',0));A=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        center=np.asarray(body['position'])+R@p['position'];extent=np.abs(R@A)@extent
        result.append((center-extent,center+extent))
    return result


def geometry_check(spec):
    bounds={b['id']:part_bounds(b) for b in spec['bodies']}
    overlaps=[];below=[];off_table=[]
    for b in spec['bodies']:
        if min(lo[2] for lo,hi in bounds[b['id']])<.795-1e-6:below.append(b['id'])
        # Conservative rectangle strictly inside the existing curved tabletop.
        if any(lo[0]<-.65 or hi[0]>.65 or lo[1]<.17 or hi[1]>.85 for lo,hi in bounds[b['id']]):
            off_table.append(b['id'])
    for a,b in itertools.combinations(spec['bodies'],2):
        for lo,hi in bounds[a['id']]:
            for lo2,hi2 in bounds[b['id']]:
                depth=np.minimum(hi,hi2)-np.maximum(lo,lo2)
                if np.all(depth>.0006):
                    overlaps.append([a['id'],b['id']]);break
            else:continue
            break
    return dict(status='PASS' if not (overlaps or below or off_table) else 'FAIL',
                overlapping_body_aabbs=overlaps,below_table=below,outside_conservative_table_region=off_table,
                scope='Conservative analytic primitive AABBs; not mesh collision, robot clearance or reachability validation.')


def usd_check(spec, path):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage=Usd.Stage.Open(str(path))
    if stage is None:raise ValueError('USD failed to open')
    if stage.GetCompositionErrors():raise ValueError('USD composition errors')
    if UsdGeom.GetStageUpAxis(stage)!='Z' or not np.isclose(UsdGeom.GetStageMetersPerUnit(stage),1):
        raise ValueError('Incorrect composed units/axis')
    if not stage.GetPrimAtPath('/World/Robot/DualArm'):raise ValueError('Shared robot missing')
    cameras=json.loads((ROOT/'cameras.json').read_text())
    if not all(stage.GetPrimAtPath(c['prim_path']).IsA(UsdGeom.Camera) for c in cameras.values()):
        raise ValueError('Shared RGBD rig missing')
    cache=UsdGeom.XformCache()
    for b in spec['bodies']:
        p=stage.GetPrimAtPath('/World/Task/'+b['id'])
        if not p:raise ValueError('Missing body '+b['id'])
        if bool(p.HasAPI(UsdPhysics.RigidBodyAPI))!=b['dynamic']:raise ValueError('Rigid body mismatch')
        if b['dynamic'] and not np.isclose(UsdPhysics.MassAPI(p).GetMassAttr().Get(),b['mass_kg']):
            raise ValueError('Mass differs from recipe')
        world=np.asarray(cache.GetLocalToWorldTransform(p)).T
        if not np.allclose(world[:3,3],b['position'],atol=1e-8) or not np.allclose(world[:3,:3],rotation(b['orientation_wxyz']),atol=1e-8):
            raise ValueError('Composed pose differs from recipe')
        colliders=[x for x in Usd.PrimRange(p) if x.HasAPI(UsdPhysics.CollisionAPI)]
        if len(colliders)!=len(b['parts']):raise ValueError('Missing compound collision geometry')
    goals=stage.GetPrimAtPath('/World/Task/Goals')
    if any(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(goals)):
        raise ValueError('Goal markers affect physics')
    return 'PASS'


def audit(root=ROOT):
    root=Path(root);specs=load_scenarios(root);rows=[];has_usd=importlib.util.find_spec('pxr') is not None
    baseline=json.loads((root/'sharpa/baseline_hashes.json').read_text())
    preserved=all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in baseline.items())
    for spec in specs:
        row=dict(task=spec['id'],title=spec['title'],offline_status='FAIL',usd_composition='NOT_RUN',
                 physics_status='NOT_RUN',source_status=spec['source_status'],
                 coverage_level=spec['coverage_level'],autonomous_policy='NOT_IMPLEMENTED',
                 task_evaluator='NOT_IMPLEMENTED' if spec['task_type']=='scene_only' else 'LOCAL_APPROXIMATION')
        try:
            folder=root/'sharpa/scenes'/spec['id'];path=folder/'scene.usda';text=path.read_text()
            if not text.startswith('#usda 1.0'):raise ValueError('Missing USDA header')
            if text!=layer(spec):raise ValueError('Scene differs from recipe; rebuild or update recipe')
            for ref in re.findall(r'@([^@]+)@',text):
                target=(folder/ref).resolve()
                if Path(ref).is_absolute() or root.resolve() not in target.parents or not target.is_file():
                    raise ValueError('Invalid or nonportable reference')
            if not (root/'tasks/workcell.usdc').read_bytes().startswith(b'PXR-USDC'):
                raise ValueError('Invalid shared binary layer')
            row['geometry']=geometry_check(spec)
            if row['geometry']['status']!='PASS':raise ValueError('Initial placement failed conservative geometry check')
            row['inputs_sha256']=fingerprint(spec,root)
            if has_usd:row['usd_composition']=usd_check(spec,path)
            live=folder/'runtime_validation.json'
            if live.exists():
                r=json.loads(live.read_text())
                row['physics_status']=r.get('runtime_status','NOT_RUN') if r.get('inputs_sha256')==row['inputs_sha256'] and r.get('task')==spec['id'] else 'STALE'
                if row['physics_status']=='PASS':
                    resets=r.get('resets',[])
                    complete=(r.get('mode')=='reset_contact_smoke' and len(resets)>=2 and r.get('reset_repeatable') is True
                              and r.get('sensor_count')==5 and all(x.get('stable') is True and x.get('seconds',0)>=5 for x in resets)
                              and r.get('evaluation',{}).get('reason')=='running' and r.get('task_success') is False)
                    if not complete:row['physics_status']='INCOMPLETE'
            row['offline_status']='PASS'
        except Exception as error:row['error']=str(error)
        rows.append(row)
    failed=not preserved or any(r['offline_status']!='PASS' or r['physics_status'] in ('ERROR','FAIL','TIMEOUT') for r in rows)
    return dict(result='FAIL' if failed else 'INCOMPLETE',
                checked_at_utc=datetime.now(timezone.utc).isoformat(),original_baseline_preserved=preserved,
                local_scene_count=len(rows),official_demo_total=None,official_verified_scene_count=0,
                official_coverage='UNVERIFIED',offline_status='PASS' if not failed else 'FAIL',
                coverage_counts=dict(Counter(r['coverage_level'] for r in rows)),rows=rows,
                blockers=['No retrievable official demo inventory/source text in this session.',
                          'Live Isaac/NVIDIA physics and USD composition must be executed where available.',
                          'Scene-only cases lack required flexible/fluid/fracture/process models and success predicates.'],
                scope='Local recipe integrity, portable paths and initial AABBs; distinct from physics, reachability, policy success or official coverage.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--offline-only',action='store_true');args=p.parse_args()
    report=audit();atomic_write(ROOT/'sharpa/validation_summary.json',json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('result','local_scene_count','offline_status','coverage_counts','original_baseline_preserved')},ensure_ascii=False))
    for r in report['rows']:
        if r['offline_status']!='PASS':print(r['task'],r.get('error'),r.get('geometry'))
    return (0 if report['offline_status']=='PASS' else 1) if args.offline_only else 1 if report['result']=='FAIL' else 2


if __name__=='__main__':sys.exit(main())
