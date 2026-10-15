"""Hardware-free ESP32 stream, calibration, and command-loss regression tests."""
import source_paths  # source-checkout package imports
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/motor_control'))
from teleop.radio_input import Axis, RadioCommand, parse_channels
from motor_control.differential import DifferentialDrive


def row(forward=1000, turn=1000, age=0, other=1000):
    return ','.join(map(str, [age, forward, turn] + [other] * 14))


class RadioTests(unittest.TestCase):
    def make_radio(self):
        return RadioCommand(Axis(1, 200, 1000, 1800),
                            Axis(2, 200, 1000, 1800, sign=-1), .08, .28)

    def arm(self, radio, start=1):
        for t in (0, .1, .2, .31):
            radio.receive(row(), start + t)
        self.assertTrue(radio.armed)

    def test_parser_rejects_diagnostics_corruption_and_out_of_range(self):
        for line in ('CRSF diagnostic: age_ms,ch1,...,ch16 (raw ticks)',
                     'NO_RECENT_CHANNEL_FRAMES', '', row() + ',1',
                     row().replace('1000', '-1', 1), row(age=-1), row(age=2**32),
                     row(forward=2048), row(forward=1.5), row().replace('1000', 'NaN', 1)):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_channels(line)
        age, values = parse_channels(row(age=10) + '\r\n')
        self.assertEqual(age, .01)
        self.assertEqual(values, (1000,) * 16)

    def test_asymmetric_calibration_deadzone_sign_and_clamp(self):
        axis = Axis(1, 100, 900, 1900, sign=-1, deadzone=.1)
        for raw, expected in ((100, 1), (900, 0), (980, 0), (1450, -.5),
                              (1900, -1), (2047, -1), (0, 1), (460, .5)):
            self.assertAlmostEqual(axis.value((raw,) * 16), expected)

    def test_invalid_calibration_fails_closed(self):
        for args in ((0, 200, 1000, 1800), (17, 200, 1000, 1800),
                     (1, -1, -1, -1), (1, 1000, 1000, 1800),
                     (1, 200, 1000, 2048), (1, 200, 1000, 1800, 0)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Axis(*args)
        with self.assertRaises(ValueError):
            RadioCommand(Axis(1, 0, 1000, 2047), Axis(1, 0, 1000, 2047), .08, .28)
        for value in (float('nan'), float('inf'), 0, -.1, .21):
            with self.assertRaises(ValueError):
                RadioCommand(Axis(1, 0, 1000, 2047), Axis(2, 0, 1000, 2047), .08, .28, value)

    def test_startup_deflected_stick_cannot_move(self):
        radio = self.make_radio()
        for i in range(20):
            radio.receive(row(1800), i * .1)
            self.assertEqual(radio.output(i * .1), (0, 0))
        self.arm(radio, start=3)
        radio.receive(row(1800), 3.4)
        self.assertEqual(radio.output(3.4), (.08, 0))

    def test_one_stick_combines_axes_and_ignores_other_channels(self):
        radio = self.make_radio()
        self.arm(radio)
        radio.receive(row(1800, 200, other=0), 1.4)
        self.assertEqual(radio.output(1.4), (.08, .28))
        targets = DifferentialDrive(max_velocity_rpm=20).mix(*radio.output(1.4))
        self.assertAlmostEqual(max(map(abs, targets)), 20)
        radio.receive(row(1800, 200, other=2047), 1.5)
        self.assertEqual(radio.output(1.5), (.08, .28))
        radio.receive(row(), 1.6)
        self.assertEqual(radio.output(1.6), (0, 0))

    def test_partial_deflection_sets_proportional_speed(self):
        radio = self.make_radio()
        self.arm(radio)
        # 0.05 deadzone plus half the remaining 0.95 span.
        radio.receive(row(1420, 580), 1.4)
        self.assertAlmostEqual(radio.output(1.4)[0], .04)
        self.assertAlmostEqual(radio.output(1.4)[1], .14)

    def test_source_age_reduces_lease_and_loss_requires_neutral(self):
        radio = self.make_radio()
        self.arm(radio)
        radio.receive(row(1800, age=100), 1.4)
        self.assertEqual(radio.output(1.44), (.08, 0))
        self.assertEqual(radio.output(1.451), (0, 0))
        radio.receive(row(1800), 1.46)
        self.assertEqual(radio.output(1.46), (0, 0))
        self.arm(radio, start=2)
        radio.receive(row(200), 2.4)
        self.assertEqual(radio.output(2.4), (-.08, 0))

    def test_bad_or_stale_row_immediately_clears_active_target(self):
        for line in ('NO_RECENT_CHANNEL_FRAMES', 'garbage', row(1800, age=150),
                     row(1800, age=501)):
            radio = self.make_radio()
            self.arm(radio)
            radio.receive(row(1800), 1.4)
            self.assertEqual(radio.output(1.4), (.08, 0))
            radio.receive(line, 1.41)
            self.assertEqual(radio.output(1.41), (0, 0))
            self.assertFalse(radio.armed)

    def test_sample_gap_cannot_be_hidden_by_receive(self):
        radio = self.make_radio()
        self.arm(radio)
        radio.receive(row(1800), 1.4)
        radio.receive(row(1800), 1.7)
        self.assertEqual(radio.output(1.7), (0, 0))
        # Widely separated neutral samples cannot acquire the 0.3s dwell.
        for i in range(10):
            radio.receive(row(), 2 + i * .2)
        self.assertFalse(radio.armed)
