"""Scan visual bindings; existing body and collision contracts stay explicit."""
import json
from functools import lru_cache
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
MAPPING = {'apple':'013_apple','fruit':'017_orange','tennis_ball':'056_tennis_ball','cup':'065-c_cups','plate':'029_plate','fork':'030_fork','spoon':'031_spoon','knife':'032_knife','hammer':'048_hammer','screwdriver':'043_phillips_screwdriver','carton':'004_sugar_box'}
@lru_cache(None)
def catalog():
    path = ROOT/'assets/realistic/catalog.json'
    return {x['id']:x for x in json.loads(path.read_text())['assets']} if path.exists() else {}
def local_bounds(parts):
    lows=[]; highs=[]
    for p in parts:
        if p['kind']=='cube': extent=np.array(p['size'])/2
        elif p['kind']=='sphere': extent=np.repeat(p['size'][0],3)
        else:
            extent=np.repeat(p['size'][0],3); extent['XYZ'.index(p.get('axis','Z'))]=p['size'][1]/2
        a=np.deg2rad(p.get('yaw_deg',0)); R=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        extent=np.abs(R)@extent; pos=np.array(p['position']); lows.append(pos-extent); highs.append(pos+extent)
    return np.min(lows,axis=0),np.max(highs,axis=0)
def binding(body):
    name=MAPPING.get(body.get('prop_kind'))
    if name is None or name not in catalog(): return None
    asset=catalog()[name]; lo,hi=local_bounds(body['parts']); size=np.array(asset['size_m']); target=hi-lo
    scale=float(np.min(target[:2]/size[:2])); center=(lo+hi)/2
    q=body['orientation_wxyz']; rotation=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
    source=scan_vertices(name)*scale+center
    current_min=float((source@rotation.T)[:,2].min())
    bottoms=[]
    for part in body['parts']:
        plo,phi=local_bounds([part]); corners=np.array([[x,y,z] for x in (plo[0],phi[0]) for y in (plo[1],phi[1]) for z in (plo[2],phi[2])])
        bottoms.append(float((corners@rotation.T)[:,2].min()))
    center += rotation.T@np.array([0.,0.,min(bottoms)-current_min])
    return {'asset_id':name,'usd':asset['usd'],'scale':scale,'translate':center.tolist(),'visual_size_m':(size*scale).tolist(),'collision_proxy_bounds_m':[lo.tolist(),hi.tolist()],'source':asset['url'],'license':asset['license'],'physics_status':'Existing analytic collision proxy retained; mesh/contact fidelity requires validation.'}
@lru_cache(None)
def scan_vertices(name):
    with np.load(ROOT/'assets/realistic'/(name+'.npz')) as data:return data['vertices'].astype(float)
def visual_lines(body,indent='            '):
    b=binding(body)
    if not b: return []
    vec=lambda v:'('+', '.join(format(float(x),'.12g') for x in v)+')'
    return [indent+'def Xform "RealisticVisual" (',indent+'    prepend references = @../../../'+b['usd']+'@</Asset>',indent+')',indent+'{',indent+'    double3 xformOp:translate = '+vec(b['translate']),indent+'    double3 xformOp:scale = '+vec([b['scale']]*3),indent+'    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',indent+'    custom string asset:source = '+json.dumps(b['source']),indent+'    custom string asset:license = "CC-BY-4.0"',indent+'}']
