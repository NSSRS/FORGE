"""Project static joint edits onto the D3 rod closure without stepping physics."""
import mujoco
import numpy as np
from scipy.optimize import least_squares

class ArmClosure:
    def __init__(self,model,data):
        self.model=model
        self.rod=model.joint('b4_1_arm_rod_d3').id
        self.bearing=model.joint('b4_2_arm_d3').id
        self.sites=[model.site('closing_b4_2_arm_rod_d3_'+str(i)).id for i in (1,2)]
        self.dofs=[*range(model.jnt_dofadr[self.rod],model.jnt_dofadr[self.rod]+3),model.jnt_dofadr[self.bearing]]
        self.qadr=int(model.jnt_qposadr[self.rod]);self.badr=int(model.jnt_qposadr[self.bearing])
        self.work=mujoco.MjData(model)
        self.last=data.qpos.copy()
    def apply(self,data):
        m=self.model
        if np.array_equal(data.qpos,self.last):
            # A new model/site or an unclosed initial pose can share the same qpos.
            mujoco.mj_kinematics(m,data)
            gap=data.site_xpos[self.sites[0]]-data.site_xpos[self.sites[1]]
            if np.linalg.norm(gap)<1e-6:return True
        candidate=data.qpos.copy()
        # Rod and bearing are dependent coordinates, never independent commands.
        candidate[self.qadr:self.qadr+4]=self.last[self.qadr:self.qadr+4]
        candidate[self.badr]=self.last[self.badr]
        velocity=np.zeros(m.nv)
        def evaluate(x):
            self.work.qpos[:]=candidate
            velocity[self.dofs]=x
            mujoco.mj_integratePos(m,self.work.qpos,velocity,1)
            mujoco.mj_kinematics(m,self.work)
            gap=self.work.site_xpos[self.sites[0]]-self.work.site_xpos[self.sites[1]]
            return np.r_[gap,1e-8*x]
        low=np.full(4,-np.pi);high=np.full(4,np.pi)
        if m.jnt_limited[self.bearing]:
            low[3]=m.jnt_range[self.bearing,0]-candidate[self.badr]
            high[3]=m.jnt_range[self.bearing,1]-candidate[self.badr]
        result=least_squares(evaluate,np.clip(np.zeros(4),low,high),bounds=(low,high),
                             max_nfev=100,ftol=1e-12,xtol=1e-12,gtol=1e-12)
        residual=evaluate(result.x)
        ok=np.linalg.norm(residual[:3])<1e-6
        data.qpos[:]=self.work.qpos if ok else self.last
        self.last=data.qpos.copy()
        mujoco.mj_forward(m,data)
        return ok
