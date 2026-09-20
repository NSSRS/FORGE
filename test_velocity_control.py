import tempfile
import unittest
from pathlib import Path
import mujoco
import numpy as np
from velocity_control import install_velocity_actuators, set_wheel_rpm

class VelocityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "model/scene.xml"))

    def test_saturates_output_torque_in_both_directions(self):
        m=self.model
        for sign in (-1,1):
            d=mujoco.MjData(m)
            d.ctrl[:]=sign*1000
            mujoco.mj_forward(m,d)
            np.testing.assert_allclose(d.actuator_force,sign*12,atol=1e-10)
            for i in range(m.nu):
                joint=int(m.actuator_trnid[i,0])
                self.assertAlmostEqual(d.qfrc_actuator[m.jnt_dofadr[joint]],sign*12)

    def test_rpm_conversion_clamping_and_reset(self):
        m=self.model;d=mujoco.MjData(m)
        set_wheel_rpm(m,d,[100,-100,30,0])
        np.testing.assert_allclose(d.ctrl,np.array([75,-75,30,0])*np.pi/30)
        mujoco.mj_resetData(m,d)
        np.testing.assert_allclose(d.ctrl,0)
        with self.assertRaises(ValueError):set_wheel_rpm(m,d,[float('nan')]*4)

    def test_only_four_driven_wheels(self):
        m=self.model
        self.assertEqual(m.nu,4)
        names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,int(x[0])) for x in m.actuator_trnid]
        self.assertEqual(names,['wheel_1','wheel_2','wheel_3','wheel_4'])
        np.testing.assert_allclose(m.actuator_gear[:,0],1)

    def test_export_installer_is_repeatable(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            joints=''.join(f'<body><joint name="wheel_{i}"/><geom size=".1"/></body>' for i in range(1,7))
            (folder/'robot.xml').write_text('<mujoco><worldbody>'+joints+'</worldbody></mujoco>')
            install_velocity_actuators(folder)
            first=(folder/'robot.xml').read_bytes()
            install_velocity_actuators(folder)
            self.assertEqual((folder/'robot.xml').read_bytes(),first)
            m=mujoco.MjModel.from_xml_path(str(folder/'robot.xml'))
            self.assertEqual(m.nu,4)

if __name__=='__main__':unittest.main()
