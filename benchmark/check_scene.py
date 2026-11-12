"""Validate generic articulation, joint anchors, sensor mounts and portable USD."""
from pathlib import Path
import json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from robot_model import Robot
ROOT=Path(__file__).resolve().parent
r=Robot(ROOT/'assets/dual_arm/robot.urdf');report={}
poses=json.loads((ROOT/'poses.json').read_text());report['preset']=poses['preset']
for j in r.revolute:
 q=poses['robots']['DualArm']['joint_positions'][j.get('name')];lim=j.find('limit');assert float(lim.get('lower'))-1e-8<=q<=float(lim.get('upper'))+1e-8
cfg=poses['robots']['DualArm'];frames=r.fk(cfg['joint_positions'],np.array(cfg['base_transform']))
if poses['preset']=='table_ready':
 for side,sign in [('left',-1),('right',1)]:
  frame=frames[side+'_hand_base'];assert np.dot(frame[:3,0],[0,0,-1])>.999
  assert (frame[:3,:3]@np.array([0,sign,0]))[0]*(-sign)>.999
report['palm_down_thumbs_inward']=poses['preset']=='table_ready'
s=Usd.Stage.Open(str(ROOT/'dealer_table.usdc'));joints=[p for p in s.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)];assert len(joints)==54
assert len([p for p in s.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)])==1
cache=UsdGeom.XformCache();gaps=[]
for prim in joints:
 j=UsdPhysics.RevoluteJoint(prim);points=[]
 for k in [0,1]:
  b=s.GetPrimAtPath(getattr(j,f'GetBody{k}Rel')().GetTargets()[0]);mat=cache.GetLocalToWorldTransform(b);p=getattr(j,f'GetLocalPos{k}Attr')().Get();points.append(np.array(mat.Transform(p)))
 gaps.append(float(np.linalg.norm(points[0]-points[1])))
assert max(gaps)<1e-6
report['joint_anchor_max_error_m']=max(gaps);report['revolute_joints']=len(joints)
cameras=json.loads((ROOT/'cameras.json').read_text());assert len(cameras)==5
assert sum(p.IsA(UsdGeom.Camera) for p in s.Traverse())==5
camera_errors=[]
for name,cfg in cameras.items():
 cam=s.GetPrimAtPath(cfg['prim_path']);assert cam.IsA(UsdGeom.Camera)
 assert cfg['model']=='Generic RGBD'
 assert cfg['resolution']==[1920,1080]
 assert cfg['horizontal_fov_deg']==86 and cfg['vertical_fov_deg']==55
 assert cam.GetAttribute('sensor:model').Get()=='Generic RGBD'
 cad=s.GetPrimAtPath(str(cam.GetParent().GetPath())+'/SensorBody')
 assert len(cad.GetChildren())==5
 assert str(cam.GetParent().GetParent().GetPath())==cfg['parent_body']
 actual=np.array(cache.GetLocalToWorldTransform(cam)).T
 error=float(np.max(np.abs(actual-np.array(cfg['T_world_camera_usd']))));assert error<1e-6;camera_errors.append(error)
assert cameras['LeftWrist']['parent_body'].endswith('/Link7_R')
assert cameras['RightWrist']['parent_body'].endswith('/Link7_L')
assert cameras['Head']['parent_body'].endswith('/'+r.root)
report['sensor_cameras']=5;report['camera_extrinsics_max_error']=max(camera_errors)
for name,side in [('LeftWrist','left'),('RightWrist','right')]:
 local=np.linalg.inv(frames[side+'_hand_base'])@np.r_[cameras[name]['eye_m'],1]
 assert local[0]>.04 and abs(local[2])<.06, local
for name in ['LeftSide','RightSide']:
 assert abs(cameras[name]['eye_m'][0])<=1.15
assert cameras['Head']['eye_m'][2]<1.5
assert not any(p.GetName() in ['HeadMast','Post','Collar','LeftWristSupport','RightWristSupport'] or p.GetName().startswith(('Leg','Foot')) for p in s.Traverse() if 'TableSideCameras' in str(p.GetPath()) or '/Robot/' in str(p.GetPath()))
report['camera_model']='Generic RGBD'
report['wrist_mounts']='palm side, wrist root'
report['camera_support_geometry']=False
assert not s.GetRootLayer().GetExternalReferences()
report['usd_self_contained']=True
report['result']='PASS';(ROOT/'geometry_checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
