"""Integrate the new CAD assembly with the established FORGE simulation scene."""
from pathlib import Path
import json, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import mujoco
import numpy as np
from simulation.merge_mjcf import merge_model
from simulation.body_colors import apply_body_colors
from simulation.arm_geometry import normalize_d3_rod_centers
from simulation.place_on_floor import place_on_floor
from simulation.velocity_control import install_velocity_actuators
from simulation.paths import MODEL_DIR, source_workspace
DRIVES=['b2_arm_d1','b3_arm_d2','b4_arm_d3','b5_arm_d4','b6_arm_d5','b7_arm_d6']
def prepare(folder):
    tree=ET.parse(folder/'robot.xml'); root=tree.getroot()
    mapping={'b1_main_frame':'body','b2_suspension_left_1':'suspension_1','b2_suspension_right_2':'suspension_2','b2_suspension_front_3':'suspension_3','b3_freedrive_5':'freedrive_1','b3_freedrive_6':'freedrive_2'}
    mapping.update({f'b3_wheel_{i}':f'wheel_{i}' for i in range(1,5)})
    mapping.update({f'b4_wheel_{i}':f'wheel_{i}' for i in (5,6)})
    mapping.update({f'l2_wheel_linkage_{i}':f'linkage_wheel_{i}' for i in range(1,5)})
    mapping.update({f'l3_wheel_linkage_{i}':f'linkage_wheel_{i}' for i in (5,6)})
    for el in root.iter():
        for attr in ('name','body1','body2','joint','joint1','joint2'):
            if el.get(attr) in mapping: el.set(attr,mapping[el.get(attr)])
    normalize_d3_rod_centers(root)
    tree.write(folder/'robot.xml',encoding='utf-8',xml_declaration=True)
    merge_model(folder,10000,preserve_collision=True)
    place_on_floor(folder)
    tree=ET.parse(folder/'robot.xml'); root=tree.getroot()
    # Rebase motor coordinates while preserving the aligned physical pose.
    pose_path=MODEL_DIR/'initial_pose.json'
    offsets=json.loads(pose_path.read_text()).get('joint_zero_offsets_rad',{}) if pose_path.exists() else {}
    for name,angle in offsets.items():
        for body in root.findall('.//worldbody//body'):
            joint=body.find(f"joint[@name='{name}']")
            if joint is None:continue
            if not np.allclose(np.fromstring(joint.get('pos','0 0 0'),sep=' '),0):
                raise ValueError('Zero rebasing currently requires joint at body origin')
            quat=np.fromstring(body.get('quat','1 0 0 0'),sep=' ')
            axis=np.fromstring(joint.get('axis','0 0 1'),sep=' ')
            axis/=np.linalg.norm(axis)
            rotation=np.r_[np.cos(angle/2),axis*np.sin(angle/2)]
            out=np.empty(4);mujoco.mju_mulQuat(out,quat,rotation)
            body.set('quat',' '.join(f'{x:.17g}' for x in out))
            if joint.get('range'):
                limits=np.fromstring(joint.get('range'),sep=' ')-angle
                joint.set('range',' '.join(map(str,limits)))
    tree.write(folder/'robot.xml',encoding='utf-8',xml_declaration=True)
    install_velocity_actuators(folder)
    tree=ET.parse(folder/'robot.xml'); root=tree.getroot()
    actuators=root.find('actuator')
    for name in DRIVES:
        j=root.find(f".//worldbody//joint[@name='{name}']")
        if j is None: raise ValueError('Missing drive '+name)
        j.set('damping','0.1');j.set('armature','0.01')
        ET.SubElement(actuators,'position',name=name+'_target',joint=name,kp='50',kv='5',ctrllimited='true',ctrlrange=j.get('range','-1.570796 1.570796'))
    apply_body_colors(root)
    contact=ET.SubElement(root,'contact')
    for a,b in [('b2_arm_d1','l3_arm_linkage_d3'),('b4_1_arm_rod_d3','b4_2_arm_d3')]:
        ET.SubElement(contact,'exclude',body1=a,body2=b)
    ET.indent(tree);tree.write(folder/'robot.xml',encoding='utf-8',xml_declaration=True)
    from simulation.spawn_fit import fit_spawn
    fit_spawn(folder/'scene.xml')
    (folder/'simulation_name_map.json').write_text(json.dumps(mapping,indent=2))
    m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    assert m.njnt==20 and m.neq==1 and m.nu==10,(m.njnt,m.neq,m.nu)
def main():
    import argparse
    argparse.ArgumentParser(description=__doc__).parse_args()
    ROOT=source_workspace()
    model=MODEL_DIR
    with tempfile.TemporaryDirectory(prefix='.export-integration-',dir=model) as temp:
        folder=Path(temp)
        shutil.copy2(model/'cad_robot.xml',folder/'robot.xml')
        shutil.copy2(model/'scene_template.xml',folder/'scene.xml')
        shutil.copytree(model/'assets',folder/'assets')
        prepare(folder)
        backup=ROOT/'build/simulation/backups'/('integration-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
        backup.mkdir(parents=True)
        moved=[];installed=[]
        try:
            for name in ('robot.xml','scene.xml','assets','simulation_name_map.json'):
                if (model/name).exists():shutil.move(str(model/name),str(backup/name));moved.append(name)
                shutil.move(str(folder/name),str(model/name));installed.append(name)
        except BaseException:
            for name in reversed(installed):shutil.move(str(model/name),str(folder/name))
            for name in reversed(moved):shutil.move(str(backup/name),str(model/name))
            raise
    print('Integrated full robot; 20 joints, 10 actuators, one ball closure.')
if __name__=='__main__':main()
