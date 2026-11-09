"""Kinematic arm-clearance study and MuJoCo playback; never opens hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from simulation.arm_closure import ArmClosure
from simulation.paths import MODEL_DIR
from simulation.spawn_fit import reset_to_spawn

DRIVES = ('b2_arm_d1', 'b3_arm_d2', 'b4_arm_d3', 'b5_arm_d4', 'b6_arm_d5', 'b7_arm_d6')
BODY_LABELS = {'body': 'Chassis', 'b2_arm_d1': 'D1 shoulder', 'b3_arm_d2': 'D2 upper arm',
               'l3_arm_linkage_d3': 'D3 crank', 'b4_1_arm_rod_d3': 'D3 rod',
               'b4_2_arm_d3': 'Forearm', 'l4_arm_linkage_d4': 'D4 wrist',
               'l5_arm_linkage_d5': 'D5 wrist', 'l6_arm_linkage_d6': 'D6 tool',
               'steel_wall': 'Steel wall', 'preview_floor': 'Floor'}


def descendant_bodies(model, root):
    result = set()
    for body in range(model.nbody):
        parent = body
        while parent:
            if parent == root:
                result.add(body)
                break
            parent = int(model.body_parentid[parent])
    return result


class ArmSweepStudy:
    """Sample one joint at a time, keeping all independent coordinates fixed.

    Query margins are enlarged only on this in-memory model. MuJoCo's existing
    contact masks, parent/weld filters and explicit body exclusions are retained.
    Thus results are conditional on these filters and the convex collision meshes.
    Sampling is not a continuous collision certificate or a hardware calibration.
    """

    def __init__(self, scene=MODEL_DIR/'scene.xml', *, clearance_mm=2., step_deg=1.,
                 extent_deg=180., refinement_deg=.1):
        for name, value in (('clearance', clearance_mm), ('step', step_deg),
                            ('extent', extent_deg), ('refinement', refinement_deg)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not refinement_deg <= step_deg <= extent_deg <= 360:
            raise ValueError('Require refinement <= step <= extent <= 360 degrees')
        self.scene = Path(scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.clearance = clearance_mm/1000
        self.step = step_deg
        self.extent = extent_deg
        self.refinement = refinement_deg
        if self.model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
            raise ValueError('Contacts disabled in this scene; use the full scene.xml')
        reset_to_spawn(self.model, self.data)
        self.home = self.data.qpos.copy()
        self.addresses = [int(self.model.joint(name).qposadr[0]) for name in DRIVES]
        self.arm_bodies = descendant_bodies(self.model, self.model.body(DRIVES[0]).id)
        self.arm_geom = np.isin(self.model.geom_bodyid, list(self.arm_bodies))
        # Inactive proximity contacts; no physics stepping or contact-force motion.
        self.query_margin = max(.01, self.clearance*2)
        self.original_margin = self.model.geom_margin.copy()
        self.original_gap = self.model.geom_gap.copy()
        self.model.geom_margin[:] = np.maximum(self.model.geom_margin, self.query_margin)
        self.model.geom_gap[:] = self.model.geom_margin
        self.original_rgba = self.model.geom_rgba.copy()
        self.original_matid = self.model.geom_matid.copy()
        self.reset()
        self.baseline = self.inspect()
        self.results = []

    def reset(self, qpos=None):
        self.data.qpos[:] = self.home if qpos is None else qpos
        self.data.qvel[:] = 0
        self.data.time = 0
        mujoco.mj_forward(self.model, self.data)
        self.closure = ArmClosure(self.model, self.data)

    def pair(self, contact):
        geoms = [int(g) for g in contact.geom]
        bodies = [int(self.model.geom_bodyid[g]) for g in geoms]
        names = [self.model.body(b).name if b else self.model.geom(g).name
                 for b, g in zip(bodies, geoms)]
        return dict(distance_mm=float(contact.dist)*1000, geom_ids=geoms,
                    geom_names=[self.model.geom(g).name for g in geoms], body_ids=bodies,
                    body_names=names, labels=[BODY_LABELS.get(n,n) for n in names],
                    point=np.asarray(contact.pos).tolist())

    def inspect(self):
        candidates = [c for c in self.data.contact
                      if self.arm_geom[int(c.geom[0])] or self.arm_geom[int(c.geom[1])]]
        nearest = self.pair(min(candidates, key=lambda c:c.dist)) if candidates else None
        distance = nearest['distance_mm']/1000 if nearest else None
        gap = float(np.linalg.norm(self.data.site_xpos[self.closure.sites[0]]-
                                   self.data.site_xpos[self.closure.sites[1]]))
        closed = gap < 1e-6
        clear = closed and (distance is None or distance >= self.clearance)
        reason = ('closure' if not closed else 'clear' if clear else
                  'collision' if distance < 0 else 'clearance')
        return dict(clear=clear, reason=reason, nearest=nearest, closure_gap_m=gap,
                    clearance_lower_bound_mm=self.query_margin*1000 if nearest is None else None)

    def pose(self, joint, angle_deg):
        if not 1 <= joint <= 6 or not math.isfinite(angle_deg):
            raise ValueError('Finite angle and joint 1..6 required')
        address = self.addresses[joint-1]
        self.data.qpos[address] = math.radians(angle_deg)
        if not self.closure.apply(self.data):
            return dict(clear=False, reason='closure', nearest=None, closure_gap_m=None,
                        requested_deg=angle_deg, retained_last_pose=True)
        result = self.inspect()
        result['requested_deg'] = angle_deg
        return result

    def _direction(self, joint, sign):
        self.reset()
        center = math.degrees(self.home[self.addresses[joint-1]])
        joint_id = self.model.joint(DRIVES[joint-1]).id
        end = center + sign*self.extent
        end_reason = 'search_end'
        if self.model.jnt_limited[joint_id]:
            authored = math.degrees(self.model.jnt_range[joint_id, 0 if sign < 0 else 1])
            if sign*(end-authored) > 0:
                end, end_reason = authored, 'authored_joint_limit'
        distance = sign*(end-center)
        if distance < 0:
            raise ValueError('Reference pose is outside an authored joint range')
        last_angle, last_pose = center, self.home.copy()
        tested = 0
        for offset in np.arange(self.step, distance+self.step, self.step):
            angle = center + sign*min(offset, distance)
            sample = self.pose(joint, angle)
            tested += 1
            if sample['clear']:
                last_angle, last_pose = angle, self.data.qpos.copy()
                continue
            blocked_angle, blocked = angle, sample
            # Always restart refinement from a known closed, clear branch.
            while abs(blocked_angle-last_angle) > self.refinement:
                middle = (last_angle+blocked_angle)/2
                self.reset(last_pose)
                sample = self.pose(joint, middle)
                tested += 1
                if sample['clear']:
                    last_angle, last_pose = middle, self.data.qpos.copy()
                else:
                    blocked_angle, blocked = middle, sample
            return dict(angle_deg=last_angle, delta_deg=last_angle-center,
                        reason=blocked['reason'], first_blocked_deg=blocked_angle,
                        boundary=blocked, samples=tested)
        return dict(angle_deg=last_angle, delta_deg=last_angle-center,
                    reason=end_reason, first_blocked_deg=None, boundary=None, samples=tested)

    def scan(self, progress=None):
        if not self.baseline['clear']:
            raise ValueError('Reference pose fails closure/clearance; inspect it before deriving ranges: '+
                             json.dumps(self.baseline))
        self.results = []
        for joint, name in enumerate(DRIVES,1):
            result = dict(joint=joint, name=name,
                          reference_deg=math.degrees(self.home[self.addresses[joint-1]]),
                          lower=self._direction(joint,-1), upper=self._direction(joint,1))
            self.results.append(result)
            if progress: progress(result)
        self.reset()
        return self.report()

    def report(self):
        exclusions = []
        for signature in self.model.exclude_signature:
            a,b = int(signature)>>16, int(signature)&65535
            exclusions.append([self.model.body(a).name,self.model.body(b).name])
        source_hashes = {}
        for path in (self.scene, self.scene.parent/'robot.xml', self.scene.parent/'initial_pose.json',
                     self.scene.parent/'omitted_surfaces.json'):
            if path.is_file(): source_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        return dict(schema_version=1, scope='sampled single-joint slices around the saved pose',
                    hardware_limits=False, model_coordinates=True, mujoco_version=mujoco.__version__,
                    scene=str(self.scene), source_file_sha256=source_hashes,
                    clearance_mm=self.clearance*1000, step_deg=self.step,
                    search_extent_deg=self.extent, refinement_deg=self.refinement,
                    reference_qpos=self.home.tolist(), baseline=self.baseline, joints=self.results,
                    audit=dict(contact_detection_enabled=True, explicit_exclusions=exclusions,
                               collision_geoms=int(np.count_nonzero(self.model.geom_contype |
                                                                   self.model.geom_conaffinity)),
                               omitted_surfaces=json.loads((self.scene.parent/'omitted_surfaces.json').read_text())
                               if (self.scene.parent/'omitted_surfaces.json').is_file() else []),
                    limitations=['Other five motor coordinates and chassis are fixed at the reference pose.',
                                 'Convex hull geometry, contact masks and parent/weld exclusions remain in effect.',
                                 'Discrete sampling/refinement is not continuous collision certification.',
                                 'Search end means no boundary found within the search, not a mechanical limit.',
                                 'Cables, assembly tolerances, gravity, stopping distance and hardware zero mapping are unvalidated.'])


def print_result(result):
    ends = []
    for side in ('lower','upper'):
        value = result[side]
        pair = (value.get('boundary') or {}).get('nearest')
        ends.append(f"{value['angle_deg']:+.2f} deg ({value['reason']}"+
                    ('; '+' / '.join(pair['labels']) if pair else '')+')')
    print(f"D{result['joint']}: "+' to '.join(ends),flush=True)


def motion_angle(result, phase):
    """Eight-second default cycle: out, dwell, home, out, dwell, home."""
    points = ((0.,result['reference_deg']),(.20,result['lower']['angle_deg']),
              (.27,result['lower']['angle_deg']),(.47,result['reference_deg']),
              (.67,result['upper']['angle_deg']),(.74,result['upper']['angle_deg']),
              (.94,result['reference_deg']),(1.,result['reference_deg']))
    for (a,x),(b,y) in zip(points,points[1:]):
        if a <= phase <= b:
            t=(phase-a)/(b-a)
            return x+(y-x)*(.5-.5*math.cos(math.pi*t))
    return result['reference_deg']


def bounds_text(result):
    return f"{result['lower']['angle_deg']:+.1f} .. {result['upper']['angle_deg']:+.1f} deg"


def annotate_model(study, joint, sample, result):
    m=study.model
    m.geom_rgba[:] = study.original_rgba
    m.geom_matid[:] = study.original_matid
    active=m.jnt_bodyid[m.joint(DRIVES[joint-1]).id]
    ids=np.flatnonzero(m.geom_bodyid==active)
    m.geom_rgba[ids]=[.15,.73,.65,1.]
    m.geom_matid[ids]=-1
    angle=math.degrees(study.data.qpos[study.addresses[joint-1]])
    boundary=None
    for side in ('lower','upper'):
        if abs(angle-result[side]['angle_deg'])<.8:
            boundary=(result[side].get('boundary') or {}).get('nearest')
    for g,b in zip((boundary or {}).get('geom_ids',[]),(boundary or {}).get('body_ids',[])):
        ids=np.flatnonzero(m.geom_bodyid==b) if b else [g]
        m.geom_rgba[ids]=[.98,.45,.12,1.]
        m.geom_matid[ids]=-1
    return boundary


def camera_for(study):
    camera=mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    study.reset()
    camera.lookat[:]=study.data.xpos[study.model.body('body').id]+[0,-.05,-.10]
    camera.distance=1.5
    camera.azimuth=-60
    camera.elevation=-15
    return camera


def view(study, *, seconds=None, cycle_seconds=8.):
    from mujoco import viewer
    state=dict(joint=1, phase=0., paused=False, collisions=False, reset=False)
    def key(code):
        if 49<=code<=54:
            state.update(joint=code-48,phase=0.,reset=True)
        elif code==32: state['paused']=not state['paused']
        elif code in (82,114): state.update(phase=0.,reset=True)
        elif code in (67,99): state['collisions']=not state['collisions']
    camera=camera_for(study)
    with viewer.launch_passive(study.model,study.data,key_callback=key,
                               show_left_ui=False,show_right_ui=False) as window:
        with window.lock():
            window.cam.lookat[:]=camera.lookat
            window.cam.distance=camera.distance
            window.cam.azimuth=camera.azimuth
            window.cam.elevation=camera.elevation
        print('MuJoCo viewer ready: Space pause; 1-6 select; R restart; C collision hulls.',flush=True)
        start=last=time.monotonic()
        while window.is_running():
            now=time.monotonic();elapsed=min(.05,now-last);last=now
            if seconds is not None and now-start>=seconds:break
            with window.lock():
                if state.pop('reset',False):study.reset()
                if not state['paused']:
                    state['phase']+=elapsed/cycle_seconds
                    if state['phase']>=1:
                        state['phase']=0.;state['joint']=state['joint']%6+1;study.reset()
                joint=state['joint'];result=study.results[joint-1]
                angle=motion_angle(result,state['phase'])
                sample=study.pose(joint,angle)
                if not sample['clear']:
                    state['paused']=True
                pair=annotate_model(study,joint,sample,result)
                window.opt.geomgroup[2]=not state['collisions']
                window.opt.geomgroup[3]=state['collisions']
                window.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=state['collisions']
                nearest=sample.get('nearest')
                clearance=f"{nearest['distance_mm']:.2f} mm" if nearest else 'No near eligible pair'
                labels=['FORGE / ARM RANGE STUDY',f'DOF {joint}', 'Candidate interval', 'Nearest clearance',
                        'Playback', 'Boundary pair']
                values=['SIMULATION ONLY',f'{angle:+.1f} deg',bounds_text(result),clearance,
                        'PAUSED' if state['paused'] else 'AUTO',
                        ' / '.join(pair['labels']) if pair else '-']
                assert study.data.time==0
            window.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               '\n'.join(labels),'\n'.join(values)),
                              (mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               'Space: pause | 1-6: select | R: restart | C: collision hulls\n'
                               'Mouse: orbit / zoom. Orange: limiting pair. Other joints stay at the saved pose.\n'
                               'Sampled model clearances; NOT validated hardware limits.','')])
            # Full sync also transfers the changing joint/boundary colors.
            window.sync()
            time.sleep(max(0.,1/60-(time.monotonic()-now)))


def render_video(study, destination, *, cycle_seconds=8., fps=24):
    import imageio.v2 as imageio
    from PIL import Image,ImageDraw,ImageFont
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(font_path,17)
    small=ImageFont.truetype(font_path,14)
    big=ImageFont.truetype(font_path,26)
    camera=camera_for(study)
    option=mujoco.MjvOption();option.geomgroup[3]=0
    study.model.vis.global_.offwidth=960;study.model.vis.global_.offheight=720
    stills=[]
    with mujoco.Renderer(study.model,height=720,width=960) as renderer, \
         imageio.get_writer(str(destination),fps=fps,codec='libx264',quality=8,
                            macro_block_size=16,ffmpeg_params=['-movflags','+faststart']) as writer:
        for joint,result in enumerate(study.results,1):
            study.reset()
            count=max(2,round(cycle_seconds*fps))
            for frame in range(count):
                phase=frame/(count-1);angle=motion_angle(result,phase)
                sample=study.pose(joint,angle)
                if not sample['clear']:
                    raise RuntimeError(f'Playback leaves sampled clear interval: D{joint}, {angle}, {sample}')
                pair=annotate_model(study,joint,sample,result)
                renderer.update_scene(study.data,camera,scene_option=option)
                canvas=Image.new('RGB',(1280,720),'#10232e')
                canvas.paste(Image.fromarray(renderer.render()),(0,0))
                draw=ImageDraw.Draw(canvas)
                draw.rectangle((0,0,960,60),fill='#10232e')
                draw.text((24,13),'FORGE  /  ARM CLEARANCE STUDY',font=big,fill='#ecf4f6')
                x=982
                draw.text((x,24),'SIMULATION ONLY',font=font,fill='#52cbbb')
                draw.text((x,70),f'DOF {joint}',font=big,fill='white')
                draw.text((x,110),f'{angle:+.1f} deg',font=big,fill='white')
                draw.text((x,160),'Sampled candidate interval',font=small,fill='#adc2cd')
                draw.text((x,184),bounds_text(result),font=font,fill='white')
                draw.text((x,222),f'Clearance threshold: {study.clearance*1000:g} mm',font=small,fill='#adc2cd')
                nearest=sample.get('nearest')
                draw.text((x,248),'Nearest: '+(f"{nearest['distance_mm']:.2f} mm" if nearest else 'outside query band'),font=small,fill='white')
                for offset,side in enumerate(('lower','upper')):
                    end=result[side];at=294+offset*94
                    draw.text((x,at),side.upper()+': '+end['reason'].replace('_',' '),font=small,fill='#ffb36d')
                    limiting=(end.get('boundary') or {}).get('nearest')
                    if limiting:
                        for line,name in enumerate(limiting['labels']):
                            draw.text((x,at+23+line*21),name,font=small,fill='white')
                for offset,other in enumerate(study.results):
                    draw.text((x,496+offset*24),f"D{other['joint']}  {bounds_text(other)}",font=small,
                              fill='#52cbbb' if other['joint']==joint else '#adc2cd')
                draw.rectangle((0,674,960,720),fill='#10232e')
                draw.text((24,687),'One joint at a time  |  D3 linkage closed  |  Orange = limiting parts',font=font,fill='white')
                draw.text((x,670),'NOT hardware limits',font=small,fill='#ffb36d')
                writer.append_data(np.asarray(canvas))
                if frame==round(.22*(count-1)):
                    stills.append(canvas.copy())
                    canvas.save(destination.with_name(f'{destination.stem}-d{joint}.png'))
            print(f'Video: D{joint} rendered',flush=True)
    sheet=Image.new('RGB',(1280,1080),'#10232e')
    for i,frame in enumerate(stills):sheet.paste(frame.resize((640,360)),((i%2)*640,(i//2)*360))
    sheet.save(destination.with_suffix('.png'))
    study.reset()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,default=MODEL_DIR/'scene.xml')
    parser.add_argument('--clearance-mm',type=float,default=2.)
    parser.add_argument('--step-deg',type=float,default=1.)
    parser.add_argument('--extent-deg',type=float,default=180.)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--headless',action='store_true',help='scan/report without opening a window')
    parser.add_argument('--video',type=Path,help='optional MP4 export; install requirements-preview.txt')
    parser.add_argument('--seconds',type=float,help='close native viewer after this duration')
    parser.add_argument('--cycle-seconds',type=float,default=8.)
    args=parser.parse_args(argv)
    if not math.isfinite(args.cycle_seconds) or args.cycle_seconds<=0:
        parser.error('--cycle-seconds must be positive and finite')
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds<=0):
        parser.error('--seconds must be positive and finite')
    study=ArmSweepStudy(args.scene,clearance_mm=args.clearance_mm,
                        step_deg=args.step_deg,extent_deg=args.extent_deg)
    report=study.scan(print_result)
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('Report:',args.report.resolve(),flush=True)
    if args.video:render_video(study,args.video,cycle_seconds=args.cycle_seconds)
    if not args.headless:view(study,seconds=args.seconds,cycle_seconds=args.cycle_seconds)


if __name__=='__main__':main()
