"""Static robot posing with native Joint sliders; no physics stepping."""
from pathlib import Path
import argparse
import os
import time
import platform
import threading

ROOT=Path(__file__).resolve().parent
def load_scene(path):
    import mujoco
    from forge_sim.spawn_fit import reset_to_spawn
    model=mujoco.MjModel.from_xml_path(str(path))
    data=mujoco.MjData(model)
    reset_to_spawn(model,data)
    return model,data
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,default=ROOT/'model/scene.xml')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--renderer',choices=('auto','gpu','software'),default='auto',
                        help='auto selects NVIDIA D3D12 under WSL unless software rendering was explicitly requested')
    parser.add_argument('--render-threads',type=int,help='Mesa software rendering threads; software mode only')
    args=parser.parse_args()
    if args.render_threads is not None and args.render_threads<1:
        parser.error('--render-threads must be positive')
    software=args.renderer=='software' or (args.renderer=='auto' and os.environ.get('LIBGL_ALWAYS_SOFTWARE')=='1')
    if args.render_threads is not None and not software:
        parser.error('--render-threads requires --renderer software')
    if software:
        os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
        os.environ['GALLIUM_DRIVER']='llvmpipe'
        os.environ['LP_NUM_THREADS']=str(args.render_threads or 1)
        print('Rendering: Mesa software; threads='+os.environ['LP_NUM_THREADS'],flush=True)
    elif args.renderer=='gpu' or 'microsoft' in platform.release().lower():
        os.environ.pop('LIBGL_ALWAYS_SOFTWARE',None)
        os.environ['GALLIUM_DRIVER']='d3d12'
        os.environ.setdefault('MESA_D3D12_DEFAULT_ADAPTER_NAME','NVIDIA')
        print('Rendering: WSL D3D12; requested adapter='+os.environ['MESA_D3D12_DEFAULT_ADAPTER_NAME'],flush=True)
    import mujoco
    from mujoco import viewer
    import numpy as np
    from forge_sim.spawn_fit import reset_to_spawn
    model,data=load_scene(args.scene.resolve())
    # Start on the opposite side; retain the existing elevation and framing.
    model.vis.global_.azimuth=(model.vis.global_.azimuth+180.0)%360.0
    if args.check:
        key=model.key('terrain_spawn').id
        np.testing.assert_allclose(data.qpos,model.key_qpos[key],atol=1e-12)
        print('PASS: native viewer initial qpos matches preview_sim terrain_spawn exactly.')
        return
    # Use the managed native UI so its Pause button enables Joint editing,
    # but replace only this process's physics loop with forward kinematics.
    # No installed MuJoCo files are changed. Pinned to MuJoCo 3.13.0.
    def static_loop(native,loader):
        result=viewer._reload(native,loader)
        if result is None:
            native.exit()
            return
        m,d=result
        print("Loaded "+str(args.scene.resolve())+"; initial DOF1 = "+str(round(float(np.degrees(d.qpos[m.jnt_qposadr[m.joint("b2_arm_d1").id]])),6))+" degrees",flush=True)
        from forge_sim.arm_closure import ArmClosure
        closure=ArmClosure(m,d)
        start=time.monotonic()
        print('STATIC VIEWER: press Space once (Simulation > Pause) to enable Joint sliders. No physics runs in either UI mode.',flush=True)
        while not native.exitrequest:
            with native.lock():
                if not closure.apply(d):
                    print("D3 pose unreachable: retained the last closed pose.",flush=True)
                mujoco.mj_forward(m,d)
                assert d.time==0, 'Static viewer advanced simulation time'
            if args.seconds and time.monotonic()-start>=args.seconds:
                print('Visible-session final D1 degrees:',np.degrees(d.qpos[m.jnt_qposadr[m.joint('b2_arm_d1').id]]),flush=True)
                native.exit()
                break
            time.sleep(1/60)
        print('Static viewer closed; simulation time remained zero.',flush=True)
    original=viewer._physics_loop
    viewer._physics_loop=static_loop
    try:
        viewer.launch(model=model,data=data)
    finally:
        viewer._physics_loop=original
if __name__=='__main__':main()
