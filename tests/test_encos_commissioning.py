"""Bounded commissioning exercises the real MotorBus simulator with a fake clock."""
import source_paths
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from motors import MotorBus, MotorError
from motors.commissioning import BenchPlan, run


class Clock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class CommissioningTests(unittest.TestCase):

    def exercise(self, plan, mutate=None, *, query_only=False):
        clock = Clock()
        opened = []

        def factory(configs, **kwargs):
            self.assertTrue(kwargs['simulate'])
            bus = MotorBus(configs, **kwargs)
            bus._backend.clock = clock
            bus._backend.last = clock()
            if mutate:
                mutate(bus)
            opened.append(bus)
            return bus
        with redirect_stdout(io.StringIO()):
            result = run(simulate=True, plan=plan, bus_factory=factory, clock=clock,
                         sleep=clock.sleep, query_only=query_only)
        return result, opened[0]

    def test_motor4_retains_physical_slot_and_both_direction_stops(self):
        result, bus = self.exercise(BenchPlan.fourth(only=True))
        self.assertEqual(result['motor_ids'], [4])
        self.assertEqual(result['motor_slots'], [4])
        self.assertEqual([s['target_rpm'] for s in result['stages']], [5, -5])
        self.assertTrue(all((s['below_half_rpm'] for s in result['stages'])))
        self.assertTrue(result['owner_closed'])
        self.assertTrue(bus._closed)
        for s in result['samples']:
            self.assertEqual([m['motor_id'] for m in s['telemetry']['motors']], [4])
            self.assertEqual([m['slot'] for m in s['telemetry']['motors']], [4])

    def test_four_motors_refresh_all_commands_in_split_slots(self):
        result, _ = self.exercise(BenchPlan.fourth())
        self.assertEqual(result['outcome'], 'passed')
        self.assertEqual(result['motor_slots'], [0, 1, 3, 4])
        for stage in result['stages']:
            self.assertEqual(set(stage['travel_deg']), {'1', '2', '3', '4'})
            for value in stage['travel_deg'].values():
                self.assertGreater(value * stage['target_rpm'], 0)
                self.assertLess(abs(value), 35)

    def test_idle_observation_reports_each_motor_without_motion(self):
        result, bus = self.exercise(BenchPlan.fourth(), query_only=True)
        self.assertEqual(result['schema'], 'forge.encos_idle_feedback')
        self.assertFalse(bus._active)
        self.assertTrue(bus._closed)
        self.assertFalse(result['can_freshness_verified'])
        self.assertEqual(set(result['stages'][0]['observed_updates']), {1, 2, 3, 4})
        self.assertTrue(all(n > 0 for n in result['stages'][0]['observed_updates'].values()))

    def test_idle_observation_fails_if_one_position_stops_advancing(self):
        def freeze(bus):
            snapshot = bus.snapshot
            def retained():
                sample = snapshot()
                sample['motors'][2]['position_observed_s'] = 0.0
                return sample
            bus.snapshot = retained
        with self.assertRaisesRegex(MotorError, 'Motor 3 idle position'):
            self.exercise(BenchPlan.fourth(), freeze, query_only=True)

    def test_missing_fourth_aborts_before_any_motion_and_records_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            report = Path(folder) / 'failure.json'
            opened = []

            def factory(configs, **kwargs):
                bus = MotorBus(configs, **kwargs)
                opened.append(bus)

                def missing(**kwargs):
                    raise MotorError('missing motor IDs: [4]')
                bus.wait_ready = missing
                return bus
            with redirect_stdout(io.StringIO()), self.assertRaisesRegex(MotorError, 'missing motor'):
                run(simulate=True, plan=BenchPlan.fourth(), report_path=report, bus_factory=factory)
            saved = json.loads(report.read_text())
            self.assertEqual(saved['outcome'], 'failed')
            self.assertTrue(saved['owner_closed'])
            self.assertEqual(opened[0]._active, set())
            self.assertEqual(saved['stages'], [])

    def test_fault_is_not_reset_and_owner_closes(self):
        opened = []
        faults_at_close = []

        def fault(bus):
            opened.append(bus)
            original = bus.velocity
            close = bus.close

            def closing():
                faults_at_close.append(bus._backend.fault)
                close()
            bus.close = closing

            def command(motor_id, rpm):
                original(motor_id, rpm)
                bus._backend.fault = 3
            bus.velocity = command
        with self.assertRaisesRegex(MotorError, 'Latched motor fault'):
            self.exercise(BenchPlan.fourth(), fault)
        self.assertTrue(opened[0]._closed)
        self.assertEqual(faults_at_close, [3])

    def test_report_collision_rejects_before_opening_owner(self):
        with tempfile.TemporaryDirectory() as folder:
            report = Path(folder) / 'existing.json'
            report.write_text('original')
            with self.assertRaises(FileExistsError):
                run(simulate=True, report_path=report, bus_factory=lambda *a, **k: self.fail('opened'))
            self.assertEqual(report.read_text(), 'original')


if __name__ == '__main__':
    unittest.main()
