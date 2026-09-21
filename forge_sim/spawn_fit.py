"""Fit a terrain spawn using rigid pose and passive joints, without changing forces."""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import least_squares

PASSIVE = ("suspension_1", "suspension_2", "suspension_3", "freedrive_1", "freedrive_2")
# Search bounds only, not claims about real mechanism travel.
PASSIVE_SEARCH_DEG = 30.0
TARGET_GAP = .0005


def reset_to_spawn(model, data):
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "terrain_spawn")
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


def fit_spawn(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    surfaces = [i for i in range(model.ngeom)
                if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or "").startswith("steel_")]
    from forge_sim.magnetic import MagneticAttraction
    wheels = [g for _, _, g in MagneticAttraction(model).pairs]
    robot = list(np.flatnonzero((model.geom_bodyid != 0) &
                 ((model.geom_contype != 0) | (model.geom_conaffinity != 0))))
    robot = list(dict.fromkeys(robot + wheels))
    floor = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"preview_floor")
    targets = surfaces + ([floor] if floor>=0 else [])
    base = model.joint("floating_base").id
    base_dof = model.jnt_dofadr[base]
    rotation = data.xmat[model.body("body").id].reshape(3,3).copy()
    joint_ids = [model.joint(n).id for n in PASSIVE]
    dofs = [model.jnt_dofadr[j] for j in joint_ids]
    scales = np.array([.025,.08,.025,*([np.deg2rad(20)]*3),*([np.deg2rad(PASSIVE_SEARCH_DEG)]*5)])
    lo,hi = -np.ones(11),np.ones(11)
    for n,j in enumerate(joint_ids,6):
        if model.jnt_limited[j]:
            initial=model.qpos0[model.jnt_qposadr[j]]
            lo[n]=max(lo[n],(model.jnt_range[j,0]-initial)/scales[n])
            hi[n]=min(hi[n],(model.jnt_range[j,1]-initial)/scales[n])
    initial = model.qpos0.copy()
    centers=[];halves=[]
    for g in surfaces:
        rot=data.geom_xmat[g].reshape(3,3)
        centers.append(data.geom_xpos[g]+rot@model.geom_aabb[g,:3])
        halves.append(np.abs(rot)@model.geom_aabb[g,3:])
    centers,halves=np.array(centers),np.array(halves)

    def evaluate(x):
        delta=x*scales
        velocity=np.zeros(model.nv)
        velocity[base_dof:base_dof+3]=rotation@delta[:3]
        velocity[base_dof+3:base_dof+6]=delta[3:6]
        velocity[dofs]=delta[6:]
        data.qpos[:]=initial
        mujoco.mj_integratePos(model,data.qpos,velocity,1.)
        mujoco.mj_forward(model,data)
        distances={}
        floor_dist=[]
        for g in robot:
            lower=np.linalg.norm(np.maximum(np.abs(centers-data.geom_xpos[g])-halves,0),axis=1)
            nearby=np.flatnonzero(lower<model.geom_rbound[g]+.15)
            distances[g]=min((mujoco.mj_geomDistance(model,data,g,surfaces[k],.15,None)
                              for k in nearby),default=.15)
            floor_dist.append(mujoco.mj_geomDistance(model,data,g,floor,.15,None) if floor>=0 else .15)
        wheel_gaps=np.array([distances[g] for g in wheels])
        # Evaluate engine-eligible self contacts without inventing exclusions.
        self_pen=max([max(0.,-c.dist) for c in data.contact
                      if model.geom_bodyid[c.geom1] and model.geom_bodyid[c.geom2]] or [0.])
        residual=np.r_[wheel_gaps-TARGET_GAP,
                       8*np.minimum(np.array(list(distances.values()))-.0001,0),
                       8*np.minimum(floor_dist,0),8*self_pen,
                       .00002*x]
        return residual,wheel_gaps,min(min(distances.values()),min(floor_dist)),-self_pen

    before=evaluate(np.zeros(11))[1]
    solution=least_squares(lambda x:evaluate(x)[0],np.zeros(11),bounds=(lo,hi),
                           diff_step=.001, max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    _,gaps,min_surface,min_self=evaluate(solution.x)
    # Reject an unsafe/inadequate fit instead of saving a misleading spawn.
    if min_surface < -.00005 or min_self < -.00005 or max(gaps) >= .01:
        raise ValueError(f"Cannot fit terrain safely: wheel gaps mm={np.round(gaps*1000,2).tolist()}, "
                         f"surface/self min mm={min_surface*1000:.3f}/{min_self*1000:.3f}")
    qpos=data.qpos.copy()
    tree=ET.parse(path);root=tree.getroot()
    keys=root.find("keyframe")
    if keys is None:keys=ET.SubElement(root,"keyframe")
    for key in list(keys):
        if key.get("name")=="terrain_spawn":keys.remove(key)
    ET.SubElement(keys,"key",name="terrain_spawn",qpos=" ".join(f"{v:.12g}" for v in qpos))
    ET.indent(tree);tree.write(path,encoding="utf-8",xml_declaration=True)
    print("Spawn fit gaps mm:",np.round(before*1000,2).tolist(),"->",np.round(gaps*1000,2).tolist())
    print("Passive joint offsets deg:",dict(zip(PASSIVE,np.round(np.degrees(solution.x[6:]*scales[6:]),2))))
    return gaps
