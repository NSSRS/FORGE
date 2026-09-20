"""Generate repeatable steel terrain presets from the current robot and scene.

Lengths are metres, angles degrees. These are test surfaces, not ship standards.
Curvature runs along the robot's forward direction; separate convex segments
preserve curved collision geometry. Bumps approximate local plate deformation.
"""
from pathlib import Path
import argparse
import copy
import math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

MODEL = Path(__file__).resolve().parent / "model"

def fmt(values):
    return " ".join(f"{v:.10g}" for v in values)


def generate(preset="curved", length=10.0, angle=60.0, width=4.0,
             bump_diameter=.10, bump_height=.005, tilt=0.0, inward=False, bump_spacing=.8, bump_track_spacing=.737):
    values = [length, angle, width, bump_diameter, bump_height, tilt, bump_spacing, bump_track_spacing]
    if not all(math.isfinite(x) for x in values):
        raise ValueError("Terrain parameters must be finite")
    if preset not in ("curved", "bumpy"):
        raise ValueError("Use curved or bumpy; flat uses the original scene.xml")
    if not (2 <= length <= 30 and 0 < angle <= 90 and 1.5 <= width <= 10):
        raise ValueError("Use length 2..30 m, angle (0,90] deg, width 1.5..10 m")
    if not (.02 <= bump_diameter <= .5 and 0 < bump_height <= .05 and 0 <= tilt <= 90):
        raise ValueError("Use bump diameter .02.. .5 m, height (0,.05] m, tilt 0..90 deg")
    if not .15 <= bump_spacing <= 5.:
        raise ValueError("Use bump spacing .15..5 m")
    if not .1 <= bump_track_spacing <= width - bump_diameter:
        raise ValueError("Bump lateral spacing must be >= 0.1 m and fit within plate width")
    radius = length / math.radians(angle)
    sign = 1 if inward else -1
    def point(s, x=0.0):
        t=s/radius
        return np.array([x, sign*radius*(1-math.cos(t)), radius*math.sin(t)])
    def normal(s):
        t=s/radius
        return np.array([0.,math.cos(t),-sign*math.sin(t)])
    # Work in a tangent frame: origin at center of the arc, steel normal +Y.
    scene=ET.parse(MODEL/"scene.xml").getroot()
    scene.set("model", "forge_"+preset)
    for inc in list(scene.findall("include")): scene.remove(inc)
    world=scene.find("worldbody")
    for g in list(world.findall("geom")):
        if g.get("name")=="steel_wall": world.remove(g)
    asset=scene.find("asset")
    if asset is None: asset=ET.SubElement(scene,"asset")
    robot=ET.parse(MODEL/"robot.xml").getroot()
    for child in robot:
        if child.tag=="worldbody": continue
        if child.tag=="asset":
            for elem in child: asset.append(copy.deepcopy(elem))
        else: scene.append(copy.deepcopy(child))
    # Place the whole test assembly above the ground, including underside tests.
    q=np.array([math.cos(math.radians(tilt)/2),-math.sin(math.radians(tilt)/2),0,0])
    rot=np.zeros(9);mujoco.mju_quat2Mat(rot,q);rot=rot.reshape(3,3)
    samples=np.array([rot@point(s) for s in np.linspace(-length/2,length/2,201)])
    origin=np.array([0.,-.79,max(1.5, .8-float(samples[:,2].min()))])
    frame=ET.SubElement(world,"frame",name="terrain_frame",pos=fmt(origin),quat=fmt(q))
    # Preserve the original tangent-facing wheel geometry, relocating its center.
    source=mujoco.MjModel.from_xml_path(str(MODEL/"scene.xml"))
    state=mujoco.MjData(source);mujoco.mj_forward(source,state)
    wheel_z=[state.xpos[source.body(f"linkage_wheel_{i}").id,2] for i in range(1,7)]
    center_z=(min(wheel_z)+max(wheel_z))/2
    for original in robot.find("worldbody"):
        elem=copy.deepcopy(original)
        if elem.tag!="frame":
            raise ValueError("Terrain generation expects the current framed robot")
        pos=np.fromstring(elem.get("pos","0 0 0"),sep=" ")
        pos-=np.array([0.,-.79,center_z])
        # Inward concavity needs clearance at the wheelbase ends.
        if inward:
            pos[1]+=radius*(1-math.cos(.45/radius))
        elem.set("pos",fmt(pos))
        frame.append(elem)
    names=[]
    def mesh_geom(name, vertices, color):
        ET.SubElement(asset,"mesh",name=name,vertex=fmt(np.asarray(vertices).ravel()))
        ET.SubElement(frame,"geom",name=name,type="mesh",mesh=name,
                      rgba=color,contype="1",conaffinity="1",friction="1 .005 .0001")
        names.append(name)
    # Adjacent slabs share boundary vertices; nominal normal thickness 20 mm.
    count=math.ceil(length/.10)
    edges=np.linspace(-length/2,length/2,count+1)
    for i,(a,b) in enumerate(zip(edges[:-1],edges[1:])):
        vertices=[point(s,x)-depth*normal(s)
                  for depth in (0.,.020) for s in (a,b) for x in (-width/2,width/2)]
        mesh_geom(f"steel_panel_{i:03d}",vertices,".45 .52 .60 1")
    bump_count=0
    if preset=="bumpy":
        # Two wheel-track rows, ahead of the spawn; leave its wheel footprint clear.
        for j,s in enumerate(np.arange(.8,length/2-.25,bump_spacing)):
            for k,x in enumerate((-bump_track_spacing/2,bump_track_spacing/2)):
                s0=s+(.12 if (j+k)%2 else 0)
                vertices=[]
                for rr in np.linspace(0,bump_diameter/2,7):
                    h=bump_height*(1-(rr/(bump_diameter/2))**2)
                    for phi in np.linspace(0,2*math.pi,32,endpoint=False):
                        ds=rr*math.sin(phi);xx=x+rr*math.cos(phi)
                        vertices.append(point(s0+ds,xx)+h*normal(s0+ds))
                for phi in np.linspace(0,2*math.pi,32,endpoint=False):
                    ds=bump_diameter/2*math.sin(phi)
                    vertices.append(point(s0+ds,x+bump_diameter/2*math.cos(phi))-.001*normal(s0+ds))
                mesh_geom(f"steel_bump_{bump_count:03d}",vertices,".57 .61 .65 1")
                bump_count+=1
    scene.find("custom/text[@name='magnetic_surface']").set("data"," ".join(names))
    custom=scene.find("custom")
    ET.SubElement(custom,"numeric",name="terrain_camera",data=fmt([*origin,270.,-12.+tilt]))
    ET.SubElement(custom,"text",name="terrain_description",data=
                  f"{preset}: L={length:g}m angle={angle:g}deg R={radius:.2f}m tilt={tilt:g}deg bumps={bump_count}")
    path=MODEL/f"scene_{preset}.xml"
    ET.indent(scene)
    xml=ET.tostring(scene,encoding="unicode")
    # Validate before replacing a previous generated preset.
    temporary=MODEL/f".terrain-{preset}.xml"
    try:
        temporary.write_text(xml)
        from spawn_fit import fit_spawn
        fit_spawn(temporary)
        mujoco.MjModel.from_xml_path(str(temporary))
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Terrain: {path.name}; radius={radius:.3f} m; {count} panels; {bump_count} bumps")
    return path

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("preset",choices=["curved","bumpy"])
    p.add_argument("--length",type=float,default=10)
    p.add_argument("--angle",type=float,default=60)
    p.add_argument("--width",type=float,default=4)
    p.add_argument("--bump-diameter",type=float,default=.10)
    p.add_argument("--bump-height",type=float,default=.005)
    p.add_argument("--bump-spacing",type=float,default=.8)
    p.add_argument("--bump-track-spacing",type=float,default=.737)
    p.add_argument("--tilt",type=float,default=0)
    p.add_argument("--inward",action="store_true")
    a=p.parse_args();generate(**vars(a))
