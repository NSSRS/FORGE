"""Cable limits do not define model zero; captures stay separate from production."""
import source_paths
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from motors.arm_endpoint import settled_endpoint, A5Capture
from test_arm_pose_mirror import mapping, report, RAW


def samples(a5=None):
    result = []
    for i in range(7):
        sample = report()
        sample.update(source="disabled-refresh", historical=False, fault=False,
                      host_received_monotonic_s=1+i*.1)
        for row in sample["feedback"]:
            row.update(feedback_valid=True, velocity_rad_s=0)
        if a5 is not None: sample["feedback"][4]["raw_position_deg"] = a5
        result.append(sample)
    return result


REFERENCE = dict(kind="a5_straight_axes", model_deg=93.17253,
                 physical_alignment_verified=False)


class EndpointTests(unittest.TestCase):
    def test_cable_ends_do_not_fit_offset_and_straight_changes_only_a5(self):
        original = mapping();before = deepcopy(original.document)
        with tempfile.TemporaryDirectory() as folder:
            recorder = A5Capture(folder, original, dict(identity_verified=True))
            for kind, angle in (("CW", -95), ("CCW", 70)):
                result = recorder.record(kind, samples(angle), 1.61)
                self.assertIsNone(result["mapping"])
            self.assertEqual(result["bounds"], [-93, 68])
            result = recorder.record("straight", samples(-3), 1.61, reference=REFERENCE)
            saved = json.loads(result["path"].with_name("mapping.json").read_text())
            self.assertAlmostEqual(saved["joints"]["5"]["model_intercept_deg"], 90.17253)
            self.assertEqual(saved["joints"]["5"]["raw_branch_deg"], [-93, 68])
            for key in "12346":self.assertEqual(saved["joints"][key], before["joints"][key])
            self.assertEqual(original.document, before)
            self.assertFalse(saved["production_qualified"])
            self.assertFalse(saved["initial_pose_target_authorized"])
            evidence = json.loads(result["path"].read_text())
            self.assertFalse(evidence["motor_targets_sent"])
            self.assertIsNone(evidence["captures"]["CW"]["model_reference"])
            # A newly restricted cable endpoint must not reuse the previous opposite end.
            result = recorder.record("CW", samples(-90), 1.61)
            self.assertIsNone(result["mapping"])
            self.assertNotIn("CCW", result["captured"])
            result = recorder.record("CCW", samples(65), 1.61)
            self.assertEqual(result["mapping"]["joints"]["5"]["raw_branch_deg"], [-88, 63])
            self.assertEqual(result["mapping"]["joints"]["5"]["model_intercept_deg"], 90.17253)

    def test_swapped_ends_and_straight_outside_bounds_never_export_a_mapping(self):
        for cw, ccw, straight in ((50,-50,0),(-10,10,20),(-200,200,0),(-1,1,0)):
            with self.subTest(cw=cw, ccw=ccw), tempfile.TemporaryDirectory() as folder:
                recorder = A5Capture(folder, mapping(), dict(identity_verified=True))
                recorder.record("CW", samples(cw), 1.61)
                recorder.record("CCW", samples(ccw), 1.61)
                result = recorder.record("straight", samples(straight), 1.61, reference=REFERENCE)
                self.assertIsNone(result["mapping"])
                self.assertTrue(result["validation_error"])
                self.assertFalse(list(Path(folder).glob("*/mapping.json")))

    def test_stale_moving_enabled_faulted_historical_missing_and_unsettled_rejected(self):
        changes = [lambda s:s[-1].update(historical=True),
                   lambda s:s[-1].update(fault=True),
                   lambda s:s[-1].update(source="passive-cache"),
                   lambda s:s[-1]["feedback"][4].update(enabled=True),
                   lambda s:s[-1]["feedback"][4].update(status=8),
                   lambda s:s[-1]["feedback"][4].update(velocity_rad_s=.1),
                   lambda s:s[-1]["feedback"][4].update(age_ms=81),
                   lambda s:s[-1]["feedback"][4].update(raw_position_deg=RAW[4]+1),
                   lambda s:s[-1]["feedback"].pop(),
                   lambda s:s.__delitem__(slice(0,2)), lambda s:s[-1].update(host_received_monotonic_s=1.1)]
        for change in changes:
            group = samples();change(group)
            with self.subTest(change=change), self.assertRaises(ValueError):settled_endpoint(group,1.61)
        with self.assertRaises(ValueError):settled_endpoint(samples(), 2)

    def test_unverified_route_changed_gain_and_old_mechanical_fit_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "routing"):
                A5Capture(folder, mapping(), {})
            m = mapping();m.joints["5"]["model_per_raw"] = 1
            with self.assertRaisesRegex(ValueError, "gain -1"):
                A5Capture(folder,m,dict(identity_verified=True))
            m = mapping();m.document["a5_cw_alignment"] = {}
            m.document["a5_cw_alignment"]["raw_cw_deg"] = -92
            with self.assertRaisesRegex(ValueError, "Obsolete"):
                A5Capture(folder,m,dict(identity_verified=True))
            self.assertEqual(list(Path(folder).iterdir()), [])


if __name__ == "__main__":unittest.main()
