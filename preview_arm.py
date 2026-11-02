"""Six-drive arm linkage preview; not calibrated hardware dynamics."""
from pathlib import Path
import argparse, time, os
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from forge_sim.spawn_fit import apply_initial_arm_pose
ROOT=Path(__file__).resolve().parent
DRIVES=['b2_arm_d1','b3_arm_d2','b4_arm_d3','b5_arm_d4','b6_arm_d5','b7_arm_d6']
PASSIVE=['b4_1_arm_rod_d3','b4_2_arm_d3']
def build():
    tree=ET.parse(ROOT/'model/robot.xml'); root=tree.getroot()
    option=root.find('option')
    if option is None: option=ET.SubElement(root,'option')
    option.set('gravity','0 0 0'); option.set('timestep','0.001'); option.set('integrator','implicitfast')
    option.set('iterations','100')
    flag=option.find('flag')
    if flag is None: flag=ET.SubElement(option,'flag')
    flag.set('contact','disable')
    joints={j.get('name'):j for j in root.findall('.//worldbody//joint')}
    missing=set(DRIVES+PASSIVE)-set(joints)
    if missing: raise ValueError(f'Missing arm joints: {missing}')
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag=='freejoint' or (child.tag=='joint' and child in joints.values() and child.get('name') not in DRIVES+PASSIVE):
                parent.remove(child)
    for name in DRIVES+PASSIVE:
        joints[name].set('damping','0.1')
        joints[name].set('armature','0.01')
    old=root.find('actuator')
    if old is not None: root.remove(old)
    actuators=ET.SubElement(root,'actuator')
    for name in DRIVES:
        j=joints[name]
        limits=j.get('range','-0.5 0.5')
        ET.SubElement(actuators,'position',name=name+'_target',joint=name,kp='20',kv='2',ctrllimited='true',ctrlrange=limits)
    path=ROOT/'model/arm_preview.xml'
    ET.indent(tree); tree.write(path,encoding='utf-8',xml_declaration=True)
    return path
def check(m):
    d=mujoco.MjData(m); apply_initial_arm_pose(m,d); mujoco.mj_forward(m,d)
    if m.neq!=1: raise ValueError(f'Expected one ball closure, got {m.neq}')
    s1,s2=int(m.eq_obj1id[0]),int(m.eq_obj2id[0])
    def gap(): return float(np.linalg.norm(d.site_xpos[s1]-d.site_xpos[s2]))
    initial=gap(); worst=initial
    for target in (0.05,-0.05,0.0):
        d.ctrl[2]=target
        for _ in range(1000):
            mujoco.mj_step(m,d); worst=max(worst,gap())
            if not np.isfinite(d.qpos).all(): raise ValueError('Non-finite pose')
    result={'initial_closure_gap_m':initial,'max_closure_gap_m':worst,'final_motor3_rad':float(d.qpos[m.jnt_qposadr[m.joint(DRIVES[2]).id]]),'passive_bearing_rad':float(d.qpos[m.jnt_qposadr[m.joint(PASSIVE[1]).id]])}
    print(result,flush=True)
    if worst>0.001: raise ValueError('Loop closure exceeds 1 mm during DOF3 preview sweep')
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--check',action='store_true'); args=parser.parse_args()
    m=mujoco.MjModel.from_xml_path(str(build()))
    if args.check: check(m); return
    from mujoco import viewer as mj_viewer
    d=mujoco.MjData(m)
    apply_initial_arm_pose(m,d)
    paused=[True]
    def key(code):
        if code==32: paused[0]=not paused[0]
        elif code in (82,114):
            mujoco.mj_resetData(m,d); apply_initial_arm_pose(m,d); mujoco.mj_forward(m,d); paused[0]=True
    mujoco.mj_forward(m,d)
    with mj_viewer.launch_passive(m,d,key_callback=key) as viewer:
        viewer.cam.lookat[:]=m.stat.center
        viewer.cam.distance=m.stat.extent*1.5
        while viewer.is_running():
            start=time.monotonic()
            if not paused[0]: mujoco.mj_step(m,d)
            viewer.sync()
            time.sleep(max(0,m.opt.timestep-(time.monotonic()-start)))
if __name__=='__main__': main()
