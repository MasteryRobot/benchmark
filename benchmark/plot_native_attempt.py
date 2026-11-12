"""Plot recorded physics measurements; never synthesize sensor RGB frames."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(folder):
    report=json.loads((folder/'report.json').read_text())
    data=np.load(folder/'measured_states.npz')
    t=data['time_s']-5.
    initial=np.asarray(report['settled_object_poses_xyzw'])
    lift=(data['objects'][:,:,2]-initial[None,:,2])*1000
    forces=np.linalg.norm(data['normal_forces']+data['friction_forces'],axis=-1)
    figure,ax=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    for i,name in enumerate(report['object_paths']):ax[0].plot(t,lift[:,i],label=name.rsplit('/',1)[-1])
    ax[0].axhline(100,color='crimson',linestyle='--',label='Baseline minimum lift: 100 mm')
    ax[0].set_ylabel('Lift above settled (mm)');ax[0].legend(loc='upper left')
    ax[1].plot(t,forces.max(axis=(1,2)),color='darkorange');ax[1].set_ylabel('Max hand contact (N)')
    ax[2].plot(t,np.max(np.abs(data['targets']-data['joints']),axis=1),color='purple')
    ax[2].set_ylabel('Max joint error (rad)');ax[2].set_xlabel('Episode time (s); negative = settling')
    for a in ax:a.grid(alpha=.2);a.axvline(0,color='grey',linestyle=':')
    figure.suptitle(report['task']+' | Recorded GPU physics | Functional success: NOT VERIFIED')
    figure.savefig(folder/'measured_diagnostics.png',dpi=160);plt.close(figure)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    plot(p.parse_args().folder)
