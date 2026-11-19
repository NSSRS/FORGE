"""Supported A2/A3 TMAX=40 and A4-A6 TMAX=12.5 setup; never enables motors."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

from .arm import ArmBus, CAP_A2_RANGE_SETUP, CAP_LOAD_RANGE_SETUP, CONTROL_REGISTERS, decode_register
from .arm_a2_range_setup import save_report
from .arm_bench import BenchSession, preflight, read_settings, validate_plan

TARGETS = {2: 40.0, 3: 40.0, 4: 12.5, 5: 12.5, 6: 12.5}


def capture_settings(session):
    """Preserve all discovery registers and raw controls of each affected drive."""
    discovery = [read_settings(session.bus, m['bus'], m['can_id'], m['feedback_id'],
                               session.stop) for m in session.motors]
    controls = {}
    for joint in TARGETS:
        controls[joint] = {}
        for register in CONTROL_REGISTERS:
            session.stop()
            reply = session.bus.read_register(2, joint, register)
            if reply is None or reply[0] != joint + 16:
                raise RuntimeError(f'A{joint} control register {register}: missing/wrong reply')
            controls[joint][register] = reply[1]
    return dict(discovery=discovery, control_raw=controls)


def configuration_snapshot(settings):
    # VBus is live read-only telemetry. Keep it in evidence, not equality checks.
    return {**settings, 'control_raw': {
        joint: {reg: raw for reg, raw in values.items() if reg != 60}
        for joint, values in settings['control_raw'].items()
    }}


def configure_load_ranges(bus, baseline, report, checkpoint):
    required = CAP_A2_RANGE_SETUP | CAP_LOAD_RANGE_SETUP
    if bus.capabilities & required != required:
        raise RuntimeError('Flash expanded load-range firmware (capability 0x1000000) first')
    info = report['base_info'] = bus.base_info()
    if info['configured_mask'] or info['id_session_locked'] or info.get('a2_range_setup_flags'):
        raise RuntimeError('Start after a motor-supply and MC02 power cycle; setup will not interrupt an existing owner')
    if any(s.enabled or s.fault or s.stopping for s in (bus.state(i) for i in range(6))):
        raise RuntimeError('MC02 is enabled, stopping or faulted; no automatic stop/reset')
    plan = deepcopy(baseline)
    validate_plan(plan, all_motors=True)
    if (plan.get('active_joints') != list(range(1, 7)) or
            [m['torque_trip_nm'] for m in plan['motors']] != [20, 12, 12, 3.5, 3.5, 3.5]):
        raise ValueError('Setup requires the unchanged six-axis baseline and rated-reference trips')
    actual = report['tmax_before_nm'] = {}
    for joint, target in TARGETS.items():
        reply = bus.read_register(2, joint, 23)
        if reply is None or reply[0] != joint + 16:
            raise RuntimeError(f'A{joint} TMAX: missing/wrong reply; no writes sent')
        actual[joint] = decode_register(23, reply[1])
        if actual[joint] not in (28.0 if joint < 4 else 10.0, target):
            raise RuntimeError(f'A{joint} reports unexpected TMAX={actual[joint]:g}; no writes sent')
        plan['motors'][joint-1]['ranges']['tmax'] = actual[joint]
    motors = plan['motors']
    observed = preflight(bus, motors)
    session = BenchSession(bus, motors, observed, report)
    steps = report['steps'] = []
    try:
        report['disabled_before'] = {str(i+1): asdict(s) for i, s in session.probe_disabled().items()}
        before = report['settings_before'] = capture_settings(session)
        expected = deepcopy(before)
        checkpoint()
        for joint, target in TARGETS.items():
            step = dict(joint=joint, target_tmax_nm=target, write_attempted=False,
                        save_attempted=False, save_acknowledged=False)
            steps.append(step)
            if actual[joint] == target:
                step['already_at_target'] = True
                continue
            step['write_attempted'] = True
            checkpoint()  # Evidence must be durable before changing the drive.
            session.stop()  # Disk sync may take longer than feedback freshness.
            operation = (bus.setup_a2_torque_range if joint == 2 else
                         lambda *, save=False: bus.setup_load_torque_range(joint, save=save))
            operation()
            step['volatile_readback_verified'] = True
            expected['discovery'][joint-1][23] = target
            after = step['settings_after_write'] = capture_settings(session)
            if configuration_snapshot(after) != configuration_snapshot(expected):
                raise RuntimeError(f'A setting other than A{joint} TMAX changed; no save sent, power-cycle before use')
            step['save_attempted'] = True
            checkpoint()
            session.stop()
            operation(save=True)  # Never retry an uncertain save/write.
            step['save_acknowledged'] = True
            after = step['settings_after_save'] = capture_settings(session)
            if configuration_snapshot(after) != configuration_snapshot(expected):
                raise RuntimeError(f'A{joint} post-save settings mismatch; do not start motion')
            checkpoint()
        report.update(passed=True, tmax_after_nm=TARGETS.copy(), power_cycle_required=True,
                      persistence_verified=False, base_info_after=bus.base_info())
    except BaseException as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        try:
            report['disabled_after'] = {str(i+1): asdict(s) for i, s in session.stop().items()}
            report['disable_acknowledged'] = True
        except (OSError, RuntimeError) as exc:
            report.update(passed=False, disable_acknowledged=False, cleanup_error=str(exc))
            if 'error' not in report:
                raise
        checkpoint()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--supported', action='store_true', required=True,
                        help='all links and gun are mechanically supported for torque loss')
    parser.add_argument('--write', action='store_true', required=True,
                        help='write and save only missing fixed TMAX settings on A2-A6')
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args(argv)
    report = dict(passed=False, operation='A2/A3 TMAX=40, A4-A6 TMAX=12.5',
                  motor_enabled_by_setup=False, persistence_verified=False)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as stream:
        json.dump(report, stream)
    checkpoint = lambda: save_report(args.report, report)
    checkpoint()
    baseline = Path(__file__).resolve().parents[1] / 'robot_description/config/arm_rig_a1_a6.json'
    try:
        with ArmBus(args.port) as bus:
            configure_load_ranges(bus, json.loads(baseline.read_text()), report, checkpoint)
    except (OSError, RuntimeError, ValueError) as exc:
        report.update(passed=False, error=str(exc))
        checkpoint()
        print(f'LOAD RANGE SETUP STOPPED: {exc}\nEvidence: {args.report}')
        return 1
    checkpoint()
    print('A2/A3 TMAX reads 40; A4-A6 reads 12.5. Motors remain disabled; no motion commanded.')
    print('Power-cycle BOTH the motor supply and MC02, then run the expanded load-profile readback check.')
    print(f'Evidence: {args.report}')
    return 0
