"""Explicit restart coordinates without wrapping feedback or physical travel."""

import source_paths
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from controls.controller.calibration import select_a1_encoder_branch
from controls.controller.weld_plan import initial_from_model, move_path, Settings
from controls.hardware.weld_app import main
from motors.arm_calibration import CalibrationMap

ROOT = Path(__file__).resolve().parents[1]


class EncoderBranchTests(unittest.TestCase):
    def setUp(self):
        self.mapping = CalibrationMap(json.loads(
            (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
        ))
        self.pose = json.loads(
            (ROOT / "src/robot_description/config/arm_standby_compact.json").read_text()
        )

    def test_reported_pose_translates_targets_and_preserves_model_and_travel(self):
        original = deepcopy(self.mapping.document)
        measured = 163.1074
        with self.assertRaisesRegex(ValueError, "A1.*163.1074"):
            self.mapping.model(1, measured)
        session = select_a1_encoder_branch(self.mapping, 1)
        self.assertAlmostEqual(session.model(1, measured), self.mapping.model(1, measured - 360))
        before = initial_from_model(self.pose, self.mapping)
        after = initial_from_model(self.pose, session)
        self.assertAlmostEqual(after[0], 185.30840435359923)
        np.testing.assert_allclose(np.array(after) - before, [360, 0, 0, 0, 0, 0])
        np.testing.assert_allclose(np.array(session.raw_limits(1)) - self.mapping.raw_limits(1), 360)
        start = np.array(after)
        start[0] = measured
        path = move_path(start, after, Settings())
        path.check_raw_bounds([session.raw_limits(i) for i in range(1, 7)])
        self.assertLess(np.ptp(path.positions[:, 0]), 23)
        self.assertTrue(np.all(path.positions[:, 0] > 160))
        for q in path.positions[:, 0]:
            self.assertAlmostEqual(session.raw(1, session.model(1, q)), q)
        for joint in range(2, 7):
            self.assertEqual(session.joints[str(joint)], self.mapping.joints[str(joint)])
            self.assertEqual(session.raw_limits(joint), self.mapping.raw_limits(joint))
        self.assertEqual(self.mapping.document, original)
        self.assertNotEqual(session.fingerprint, self.mapping.fingerprint)
        self.assertEqual(session.document["encoder_branch_selection"]["source_mapping_fingerprint"], self.mapping.fingerprint)
        self.assertFalse(session.document["production_qualified"])

    def test_session_branch_stays_fixed_and_a4_never_rebases(self):
        self.assertIs(select_a1_encoder_branch(self.mapping), self.mapping)
        for turns in (-1, 1):
            session = select_a1_encoder_branch(self.mapping, turns)
            for value in (-196.8926, -174.6916):
                shifted = value + 360 * turns
                self.assertAlmostEqual(session.model(1, shifted), self.mapping.model(1, value))
                with self.assertRaises(ValueError):
                    session.model(1, value)
            lo, hi = session.raw_limits(1)
            for value in (lo - .01, hi + .01, float("nan")):
                with self.assertRaises(ValueError):
                    session.model(1, value)
            with self.assertRaises(ValueError):
                session.model(4, self.mapping.joints["4"]["model_intercept_deg"] + 360)
            with self.assertRaisesRegex(ValueError, "already translated"):
                select_a1_encoder_branch(session, turns)
        for turns in (True, 1.0, .5, 2, -2):
            with self.assertRaises(ValueError):
                select_a1_encoder_branch(self.mapping, turns)

    def test_cli_checks_source_pair_then_uses_session_branch_for_saved_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "mapping.json").write_text(json.dumps(self.mapping.document))
            (folder / "standby.json").write_text(json.dumps(self.pose))
            raw = initial_from_model(self.pose, self.mapping)
            raw[0] = 163.1074
            (folder / "start.json").write_text(json.dumps(dict(
                passed=True, feedback=[dict(joint=i, enabled=False, status=0, raw_position_deg=q)
                                       for i, q in enumerate(raw, 1)]
            )))
            args = ["--plan-only", "--arm", "--a1-encoder-turns", "1",
                    "--mapping", str(folder / "mapping.json"),
                    "--initial-pose", str(folder / "standby.json"),
                    "--start-report", str(folder / "start.json"),
                    "--output", str(folder / "result.json")]
            result = dict(initial_raw_deg=[], preparation_timing={}, weld_start_check={},
                          effective_hz=1.0, feed_m_s=.006, optimization={})
            with patch("controls.hardware.weld_app.build_plan", return_value=result) as build, \
                    patch("controls.hardware.weld_ports.ArmOwner") as owner, \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(main(args), 0)
                request = build.call_args.args[0]
                self.assertEqual(request["current_raw_deg"], raw)
                self.assertAlmostEqual(request["initial_raw_deg"][0], 185.30840435359923)
                session = CalibrationMap(request["mapping"])
                self.assertAlmostEqual(session.model(1, raw[0]), 22.20100435359924)
                saved = json.loads((folder / "result.json").read_text())
                self.assertEqual(saved["encoder_branch_selection"]["turns"], 1)
                owner.assert_not_called()
                # Selecting a branch cannot hide a mismatched source standby.
                (folder / "result.json").unlink()
                self.pose["mapping_fingerprint"] = "wrong-source-map"
                (folder / "standby.json").write_text(json.dumps(self.pose))
                build.reset_mock()
                self.assertEqual(main(args), 2)
                build.assert_not_called()
                owner.assert_not_called()


if __name__ == "__main__":
    unittest.main()
