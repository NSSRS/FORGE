"""Offline evidence handling must not turn stale or interrupted runs into success."""
import source_paths
import json
import contextlib
import io
import math
from pathlib import Path
import tempfile
import unittest

from controls.hardware.hold_analysis import compare, main, print_summary, summarize


def sample(t, state="ready", *, error=.1, velocity=2, torque=-3, **overrides):
    rows = {}
    for joint in map(str, range(1, 7)):
        rows[joint] = dict(at_monotonic_s=t, age_ms=0, position_rad=0,
                           target_rad=math.radians(error), velocity_rad_s=math.radians(velocity),
                           torque_feedback_nm=torque, requested_speed_rad_s=.1,
                           enabled=True, enable_confirmed=True, status=1, mos_c=30, rotor_c=31)
        rows[joint].update(overrides)
    return dict(kind="sample", time_s=t, state=state, events=[],
                arm=dict(holding=True, endpoint_hold=state == "ready", last_samples=rows))


def records():
    start = sample(1, "verifying_hold")
    start["events"] = [dict(event="recovery_requested", time_s=1)]
    ready = sample(3)
    ready["events"] = [dict(event="stage_arrived", state="ready", time_s=3)]
    return [dict(kind="session", schema="forge.weld_motion_journal", simulated=False,
                 settings=dict(max_speed_deg_s=5), tracker=dict(version=3), mapping=dict(version=1)),
            dict(kind="accepted_plan", plan=dict(home={"goal": 1}, scene_fingerprint="scene")),
            start, sample(2, "initial"), ready, sample(13)]


class HoldAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "session.jsonl"

    def write(self, rows, tail=""):
        self.path.write_text("".join(json.dumps(row)+"\n" for row in rows)+tail)
        return summarize(self.path)

    def test_timing_signed_torque_speed_and_hold_error_are_distinct(self):
        result = self.write(records())
        self.assertEqual(result["attempts"][0]["time_to_ready_s"], 2)
        hold = result["segments"][-1]
        joint = hold["joints"]["1"]
        self.assertEqual(joint["torque_feedback_nm"]["mean"], -3)
        self.assertEqual(joint["torque_feedback_nm"]["rms"], 3)
        self.assertAlmostEqual(joint["velocity_deg_s"]["peak_abs"], 2)
        self.assertAlmostEqual(joint["command_error_deg"]["peak_abs"], .1)
        self.assertEqual(joint["max_sample_gap_s"], 10)
        self.assertFalse(result["hardware_qualified"])
        self.assertIsNone(result["disable_acknowledged"])

    def test_duplicate_cache_does_not_extend_hold_and_stale_age_includes_host_delay(self):
        rows = records()
        duplicate = sample(100, at_monotonic_s=13)
        stale = sample(101, at_monotonic_s=100.9)
        result = self.write(rows+[duplicate, stale, sample(102, enabled=False), sample(103, status=8)])
        hold = result["segments"][-1]
        self.assertEqual(hold["duplicates"], 6)
        self.assertEqual(hold["rejected"], 18)
        self.assertEqual(hold["minimum_axis_span_s"], 10)
        self.assertEqual(hold["joints"]["1"]["samples"], 2)

    def test_interrupted_attempt_and_later_success_are_not_combined(self):
        rows = records()[:4]
        interrupted = sample(2.5, "paused")
        interrupted["events"] = [dict(event="pause", time_s=2.5)]
        second = records()[2:]
        for row in second:
            row["time_s"] += 10
            for event in row["events"]:
                event["time_s"] += 10
            for item in row["arm"]["last_samples"].values():
                item["at_monotonic_s"] += 10
        result = self.write(rows+[interrupted]+second)
        self.assertEqual([a["outcome"] for a in result["attempts"]], ["interrupted", "ready"])
        self.assertIsNone(result["attempts"][0]["time_to_ready_s"])
        self.assertEqual(result["attempts"][1]["time_to_ready_s"], 2)

    def test_partial_live_tail_and_closed_shutdown_failure_stay_explicit(self):
        result = self.write(records(), '{"kind":')
        self.assertTrue(result["incomplete_final_line"])
        self.assertFalse(result["session_closed"])
        result = self.write(records()+[dict(kind="session_end", fault="missing feedback", dropped_records=4,
                              arm_session=dict(disable_acknowledged=False, mode_restoration_verified=False))])
        self.assertFalse(result["disable_acknowledged"])
        self.assertIn("missing feedback", result["faults"])
        self.assertEqual(result["dropped_records"], 4)

    def test_malformed_complete_record_and_nonmonotonic_clock_are_rejected(self):
        self.path.write_text('{bad}\n')
        with self.assertRaisesRegex(ValueError, "line 1"):
            summarize(self.path)
        with self.assertRaisesRegex(ValueError, "monotonic"):
            self.write(records()+[sample(1)])

    def test_missing_axes_and_error_over_tolerance_cannot_show_all_axes_in_tolerance(self):
        rows = records()
        rows[-1] = sample(13, error=.4)
        rows[-2]["arm"]["last_samples"].pop("6")
        rows[-1]["arm"]["last_samples"].pop("6")
        hold = self.write(rows)["segments"][-1]
        self.assertFalse(hold["all_axes_within_025_deg"])
        self.assertEqual(hold["minimum_axis_span_s"], 0)
        self.assertIsNone(hold["joints"]["6"]["torque_feedback_nm"])

    def test_comparison_detects_path_setting_scene_changes(self):
        before = self.write(records())
        rows = records()
        rows[0]["settings"]["max_speed_deg_s"] = 100
        rows[1]["plan"].update(home={"goal": 2}, scene_fingerprint="other")
        after = self.write(rows)
        checks = compare(before, after)["checks"]
        self.assertFalse(checks["settings"])
        self.assertFalse(checks["identical_planned_paths"])
        self.assertFalse(checks["same_scene"])

    def test_invalid_mass_is_rejected(self):
        with self.assertRaises(SystemExit):
            main([str(self.path), "--ee-mass-kg", "nan"])

    def test_temperatures_keep_observed_changes_and_missing_values_explicit(self):
        rows = records()
        rows[-1] = sample(13, mos_c=35, rotor_c=39)
        result = self.write(rows)
        joint = result['segments'][-1]['joints']['2']
        self.assertEqual(joint['mos_c']['maximum'], 35)
        self.assertEqual(joint['temperature_change_c']['rotor_c'],
                         dict(first_c=31, last_c=39, change_c=8, observed_span_s=10))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            print_summary(result)
        self.assertIn('Peak MOS  Peak motor', output.getvalue())
        self.assertIn('+8 degC over 10.00 s', output.getvalue())
        rows[-2] = sample(3, mos_c=None, rotor_c=None)
        joint = self.write(rows)['segments'][-1]['joints']['2']
        self.assertIsNone(joint['temperature_change_c']['mos_c']['change_c'])
        rows[-1] = sample(13, mos_c=None, rotor_c=None)
        joint = self.write(rows)['segments'][-1]['joints']['2']
        self.assertIsNone(joint['mos_c'])
        self.assertIsNone(joint['temperature_change_c']['rotor_c'])

    def test_loaded_configuration_is_preserved_and_compared(self):
        rows = records()
        profile = dict(load_context={'welding_gun_attached': True},
                       torque_policy={'a2_trip_nm': 13, 'overload_duration_cutoff_s': None})
        rows.insert(1, dict(kind='arm_configuration', profile=profile))
        first = self.write(rows)
        self.assertEqual(first['arm_configuration'], profile)
        self.assertTrue(compare(first, first)['checks']['same_arm_configuration'])
        profile['torque_policy']['a2_trip_nm'] = 12
        second = self.write(rows)
        self.assertFalse(compare(first, second)['checks']['same_arm_configuration'])
        legacy = self.write(records())
        self.assertIsNone(compare(legacy, first)['checks']['same_arm_configuration'])

    def test_endpoint_retreat_timing_is_separate_from_normal_preparation(self):
        rows=records()
        rows[3]['state']='endpoint_recovery'
        rows[3]['events']=[dict(event='endpoint_recovery_completed',time_s=2,duration_s=.8)]
        result=self.write(rows)
        attempt=result['attempts'][0]
        self.assertEqual(attempt['time_to_ready_s'],2)
        self.assertEqual(attempt['endpoint_recovery_s'],.8)
        self.assertEqual(attempt['time_to_ready_after_endpoint_s'],1)
        self.assertEqual(attempt['normal_preparation_to_ready_s'],1)
        self.assertEqual(result['segments'][1]['state'],'endpoint_recovery')
        rows=records()
        rows[3]['events']=[dict(event='hold_verified',state='initial',time_s=1.25)]
        ordinary=self.write(rows)['attempts'][0]
        self.assertEqual(ordinary['normal_preparation_to_ready_s'],1.75)
        self.assertIsNone(ordinary['endpoint_recovery_s'])


if __name__ == "__main__":
    unittest.main()
