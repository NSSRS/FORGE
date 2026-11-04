"""Export Onshape CAD into the shared robot_description model."""
from pathlib import Path
import json, os, subprocess, sys, tempfile, shutil
from datetime import datetime
from dotenv import load_dotenv
from simulation.paths import MODEL_DIR, source_workspace
from simulation.onshape_compat import prepare_meshes
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
from stl import mesh as stl_mesh

def main():
    import argparse
    argparse.ArgumentParser(description=__doc__).parse_args()
    ROOT=source_workspace()
    load_dotenv(ROOT/'src/simulation/.env')
    model=MODEL_DIR
    config=json.loads((model/'config.json').read_text())
    config['joint_properties']={'*':{'actuated':False}}
    with tempfile.TemporaryDirectory(prefix='.export-',dir=model) as folder:
        stage=Path(folder)
        (stage/'config.json').write_text(json.dumps(config))
        subprocess.run([sys.executable,'-u',str(Path(__file__).with_name('onshape_compat.py')),str(stage)],check=True,cwd=ROOT)
        raw=ROOT/'build/simulation/backups'/('raw-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
        raw.mkdir(parents=True)
        shutil.copy2(stage/'robot.xml',raw/'robot.xml')
        shutil.copytree(stage/'assets',raw/'assets')
        prepare_meshes(stage/'robot.xml')
        tree=ET.parse(stage/'robot.xml'); xml=tree.getroot(); omitted=[]
        for asset in list(xml.findall('asset/mesh')):
            file=stage/'assets'/asset.get('file')
            if file.suffix.lower()!='.stl': continue
            pts=stl_mesh.Mesh.from_file(str(file)).vectors.reshape(-1,3).astype(float)
            if np.linalg.matrix_rank(pts-pts.mean(axis=0),tol=1e-9)<3:
                name=asset.get('name',file.stem); omitted.append(name)
                xml.find('asset').remove(asset)
                for parent in list(xml.iter()):
                    for g in list(parent):
                        if g.tag=='geom' and g.get('mesh')==name: parent.remove(g)
        tree.write(stage/'robot.xml',encoding='utf-8',xml_declaration=True)
        (stage/'omitted_surfaces.json').write_text(json.dumps(omitted,ensure_ascii=False,indent=2))
        print('Omitted zero-thickness preview meshes:',len(omitted),flush=True)
        shutil.copy2(stage/'robot.xml',stage/'cad_robot.xml')
        shutil.copy2(model/'scene_template.xml',stage/'scene.xml')
        from simulation.integrate_sim import prepare
        prepare(stage)
        m=mujoco.MjModel.from_xml_path(str(stage/'scene.xml'))
        d=mujoco.MjData(m); mujoco.mj_forward(m,d)
        print('Validated:',m.nbody,'bodies,',m.njnt,'joints,',m.neq,'equalities',flush=True)
        backup=ROOT/'build/simulation/backups'/datetime.now().strftime('%Y%m%d-%H%M%S')
        backup.mkdir(parents=True)
        moved, installed = [], []
        try:
            for name in ('robot.xml','scene.xml','assets','cad_robot.xml','simulation_name_map.json','omitted_surfaces.json'):
                if (model/name).exists():
                    shutil.move(str(model/name),str(backup/name)); moved.append(name)
                if (stage/name).exists():
                    shutil.move(str(stage/name),str(model/name)); installed.append(name)
        except BaseException:
            for name in reversed(installed): shutil.move(str(model/name),str(stage/name))
            for name in reversed(moved): shutil.move(str(backup/name),str(model/name))
            raise
        print('Export installed:',model,flush=True)
if __name__=='__main__': main()
