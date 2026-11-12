from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def transform(xyz=(0,0,0),rpy=(0,0,0)):
 t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz;return t

def origin(node):
 if node is None:return np.eye(4)
 return transform(np.fromstring(node.get('xyz','0 0 0'),sep=' '),np.fromstring(node.get('rpy','0 0 0'),sep=' '))
class Robot:
 def __init__(self,filename):
  self.filename=Path(filename);self.xml=ET.parse(filename).getroot();self.links={n.get('name'):n for n in self.xml.findall('link')}
  self.joints=self.xml.findall('joint');self.revolute=[j for j in self.joints if j.get('type')=='revolute'];self.children={}
  for j in self.joints:self.children.setdefault(j.find('parent').get('link'),[]).append(j)
  self.root=next(n for n in self.links if n not in {j.find('child').get('link') for j in self.joints})
  self.ordered=[]
  def walk(n):
   for j in self.children.get(n,[]):self.ordered.append(j);walk(j.find('child').get('link'))
  walk(self.root)
  self.groups={self.root:self.root}
  for j in self.ordered:
   c=j.find('child').get('link');p=j.find('parent').get('link');self.groups[c]=self.groups[p] if j.get('type')=='fixed' else c
 def fk(self,q=None,base=None):
  q=q or {};frames={self.root:np.eye(4) if base is None else base}
  for j in self.ordered:
   t=origin(j.find('origin'))
   if j.get('type')=='revolute':
    a=np.fromstring(j.find('axis').get('xyz'),sep=' ');r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(a*q.get(j.get('name'),0)).as_matrix();t=t@r
   frames[j.find('child').get('link')]=frames[j.find('parent').get('link')]@t
  return frames
