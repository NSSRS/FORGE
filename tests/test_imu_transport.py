import source_paths  # source-checkout package imports
import struct
import unittest
from imu.protocol import crc, read_sample, decode

class TransportTests(unittest.TestCase):
    def test_known_crc(self):
        self.assertEqual(crc(bytes.fromhex('01030000000a')), 0xcdc5)

    def test_physical_frame_and_scaling(self):
        raw=bytes.fromhex('50031e0056001307fffffe0000000107660194f4c7021e0000f66bffff300c00017ab3')
        class Port:
            def reset_input_buffer(self): pass
            def write(self, data): self.request=data
            def read(self, size): return raw
        p=Port()
        regs=read_sample(p)
        a,g,m,r=decode(regs)
        self.assertAlmostEqual(a[2], 9.801861596679687)
        self.assertLess(g[0],0)
        self.assertLess(r[1],0)
        # Captured frame: X/roll=0.542, Y/pitch=-2.453, Z/yaw=77.836 degrees.
        import math
        for actual, expected in zip(r, (0.542, -2.453, 77.836)):
            self.assertAlmostEqual(math.degrees(actual), expected)
        self.assertEqual(p.request[1],3)

    def test_corrupt_and_short_rejected(self):
        class Port:
            def reset_input_buffer(self): pass
            def write(self, data): pass
            def read(self, size): return bytes(35)
        with self.assertRaises(ValueError): read_sample(Port())
        with self.assertRaises(ValueError): decode([0])

if __name__=='__main__': unittest.main()
