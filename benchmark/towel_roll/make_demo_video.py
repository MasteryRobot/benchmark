"""Render a measured CPU flex attempt as MP4; never imply Isaac/finger success."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib
from .config import FOLDER, SPEC
from .raster import view
from .explore import mujoco, model_xml, NX, NY

FFMPEG = FOLDER.parent / '.venvs/lerobot-v21/lib/python3.10/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
FONT_DIR = Path(matplotlib.get_data_path()) / 'fonts/ttf'


def simulate(out):
    towel = SPEC['towel']
    # flexcomp grids are centered. Correct the historical origin offset for
    # this separate demo, while retaining the original experiment artifacts.
    xml = model_xml().replace(
        f'pos="{-towel["width_m"]/2} {towel["near_y_m"]} {towel["table_height_m"]+towel["thickness_m"]/2}"',
        f'pos="0 {towel["near_y_m"]+towel["length_m"]/2} {towel["table_height_m"]+towel["thickness_m"]/2}"'
    ).replace('selfcollide="none"', 'selfcollide="auto"')
    (out / 'cpu_model.xml').write_text(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial = np.array(data.flexvert_xpos).reshape(NX, NY, 3)
    np.testing.assert_allclose(initial[:, :, 0].min(), -towel['width_m']/2, atol=1e-6)
    np.testing.assert_allclose(initial[:, :, 1].min(), towel['near_y_m'], atol=1e-6)
    vids = [0, (NX - 1) * NY]
    bodies = [model.flex_vertbodyid[v] for v in vids]
    joints = [model.body_jntadr[b] for b in bodies]
    qaddrs = [model.jnt_qposadr[j] for j in joints]
    vaddrs = [model.jnt_dofadr[j] for j in joints]
    prev = initial.reshape(-1, 3)[vids].copy()
    dt = float(model.opt.timestep)
    settle, roll, release = .3, 2., .7
    duration = settle + roll + release
    frames, metrics = [], []
    started = time.monotonic()
    for step in range(round(duration / dt)):
        t = step * dt
        u = np.clip((t - settle) / roll, 0., 1.)
        u = float(u * u * (3 - 2 * u))
        theta = 2 * np.pi * 1.3 * u
        radius = .022 + .003 * u
        active = t < settle + roll
        targets = np.array([
            [x, towel['near_y_m'] + .25 * u - radius * np.sin(theta),
             towel['table_height_m'] + towel['thickness_m']/2 + radius * (1 - np.cos(theta))]
            for x in (-towel['width_m']/2, towel['width_m']/2)])
        if active:
            # Prescribed endpoint boundary conditions, explicitly disclosed as
            # proxy grasps. No robot joint or measured finger contacts are used.
            for i, (qa, va, body) in enumerate(zip(qaddrs, vaddrs, bodies)):
                data.qpos[qa:qa+3] = targets[i] - model.body_pos[body]
                data.qvel[va:va+3] = (targets[i] - prev[i]) / dt
        prev = targets.copy()
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or abs(data.time - (step+1)*dt) > dt/2:
            raise RuntimeError('Simulation instability; refusing a misleading video')
        if step % 10 == 0 or step == round(duration/dt)-1:
            mujoco.mj_forward(model, data)
            xyz = np.array(data.flexvert_xpos).reshape(NX, NY, 3)
            frames.append(xyz)
            metrics.append({'time_s': float(data.time),
                            'phase': 'SETTLE' if t < settle else 'ROLL ATTEMPT' if active else 'RELEASE / OBSERVE',
                            'proxy_active': bool(active),
                            'length_extent_m': float(np.ptp(xyz[:, :, 1])),
                            'height_m': float(xyz[:, :, 2].max() - towel['table_height_m']),
                            'max_speed_m_s': float(np.linalg.norm(data.qvel.reshape(NX, NY, 3), axis=2).max()),
                            'contacts': int(data.ncon)})
        if step % 250 == 0:
            print(f'CPU simulation: {step}/{round(duration/dt)} steps', flush=True)
    np.savez_compressed(out / 'measured_mesh_frames.npz', vertices=np.asarray(frames))
    report = {'backend': 'MuJoCo ' + mujoco.__version__ + ' CPU flex',
              'simulated_seconds': duration, 'wall_seconds': time.monotonic()-started,
              'physics_success_verified': False, 'isaac_run': False,
              'limitations': ['Two prescribed corner constraints replace articulated finger grasps.',
                              'No robot is controlled by this CPU demo.',
                              'Robot inset is a static CAD scene, not a replay.',
                              'Rendered cloth vertices come from this CPU solver, not an ideal rolled shape.',
                              'A displayed roll attempt is not a successful benchmark episode.'],
              'changes_from_historical_cpu_exploration': ['Correct centered grid placement',
                  'Correct initial constrained corner positions', 'Enable flex self collision',
                  'Release both constraints at the end'],
              'warnings': [{'index': i, 'count': int(w.number)} for i, w in enumerate(data.warning) if w.number],
              'final': metrics[-1], 'metrics': metrics,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Simulation complete: ' + json.dumps(report['final']), flush=True)
    return frames, metrics


def cloth_image(xyz):
    faces = []
    colors = []
    for i in range(NX-1):
        for j in range(NY-1):
            color = [.11, .64, .67] if (i+j)%2 else [.08, .56, .61]
            if i in (0, NX-2) or j in (0, NY-2): color = [.69, .92, .87]
            faces.extend([[xyz[i,j], xyz[i+1,j], xyz[i+1,j+1]],
                          [xyz[i,j], xyz[i+1,j+1], xyz[i,j+1]]])
            colors.extend([color, color])
    z = SPEC['towel']['table_height_m']-.002
    corners = [[-.45,.10,z],[.45,.10,z],[.45,.94,z],[-.45,.94,z]]
    faces.extend([[corners[0],corners[1],corners[2]],[corners[0],corners[2],corners[3]]])
    colors.extend([[.18,.25,.29]]*2)
    faces = np.asarray(faces)
    normals = np.cross(faces[:,1]-faces[:,0], faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
    shade = .55+.45*np.abs(normals@np.array([-.3,-.4,.866]))
    rgb = np.clip(np.asarray(colors)*shade[:,None]*255,0,255).astype(np.uint8)
    return view(faces,rgb,[1.0,-.8,1.9],[0,.52,.81],.74,800,420)


def encode(out, frames, metrics):
    fonts = {s: ImageFont.truetype(str(FONT_DIR/'DejaVuSans.ttf'),s) for s in (15,18,22,28,34)}
    inset = Image.open(FOLDER/'images/scene_preview.png').crop((0,120,900,920)).resize((330,293))
    path = out/'towel_roll_demo.mp4'
    command = [str(FFMPEG),'-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
               '-s','1280x720','-r','20','-i','pipe:0','-an','-c:v','libx264','-preset','fast',
               '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart','-n',str(path)]
    process = subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        # Each saved 0.02s CPU sample is shown for 0.10s: five-times slow motion.
        for index,(xyz,metric) in enumerate(zip(frames,metrics)):
            im = Image.new('RGB',(1280,720),'#101d29'); draw = ImageDraw.Draw(im)
            draw.text((28,20),'TOWEL ROLL / CPU ATTEMPT',font=fonts[34],fill='#f2f6f8')
            draw.text((30,67),'Recorded cloth motion  |  36 x 40 cm towel  |  5x slow motion',font=fonts[18],fill='#a7bdca')
            im.paste(cloth_image(xyz),(25,110))
            draw = ImageDraw.Draw(im)
            draw.rounded_rectangle((42,125,360,161),radius=7,fill='#142b35')
            draw.text((55,132),metric['phase'],font=fonts[18],fill='#75e2cb')
            draw.text((855,113),'ROBOT SCENE / STATIC CAD',font=fonts[18],fill='#b6c9d4')
            im.paste(inset,(855,148))
            draw = ImageDraw.Draw(im)
            draw.text((855,462),f'Simulation time  {metric["time_s"]:4.2f} s',font=fonts[22],fill='#e7eff3')
            draw.text((855,500),f'Length extent    {metric["length_extent_m"]*100:4.1f} cm',font=fonts[18],fill='#a7bdca')
            draw.text((855,532),f'Cloth height       {metric["height_m"]*100:4.1f} cm',font=fonts[18],fill='#a7bdca')
            draw.text((855,568),'Physical success: UNVERIFIED',font=fonts[18],fill='#ffc381')
            draw.text((30,548),'CLOTH CENTER STRIP / SIDE VIEW',font=fonts[15],fill='#a7bdca')
            base_y=633
            draw.line((45,base_y,800,base_y),fill='#4e6674',width=2)
            center=xyz[NX//2]
            line=[(45+(p[1]-.20)/.65*755,base_y-(p[2]-SPEC['towel']['table_height_m'])*1000) for p in center]
            draw.line(line,fill='#66dbc9',width=4)
            draw.text((855,601),'Robot is not driven in this demo.',font=fonts[15],fill='#a7bdca')
            draw.rectangle((0,660,1280,720),fill='#342a21')
            draw.text((28,672),'CPU APPROXIMATION  |  TWO CORNER CONSTRAINTS  |  NOT AN ISAAC RUN',font=fonts[18],fill='#ffd3a0')
            draw.text((28,698),'No articulated finger-contact verification. Mesh shown above is the actual CPU attempt.',font=fonts[15],fill='#dcbca0')
            if index in (0,len(frames)//2,len(frames)-1): im.save(out/f'frame_{index:03d}.png')
            repeats = 20 if index in (0,len(frames)-1) else 2
            for _ in range(repeats): process.stdin.write(im.tobytes())
            if index%30==0: print(f'Video render: {index}/{len(frames)} frames',flush=True)
        process.stdin.close()
        error=process.stderr.read().decode()
        if process.wait(): raise RuntimeError(error)
    finally:
        if process.poll() is None: process.kill(); process.wait()
    print(str(path),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    frames,metrics=simulate(args.output)
    encode(args.output,frames,metrics)


if __name__=='__main__': main()
