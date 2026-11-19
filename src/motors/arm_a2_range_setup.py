"""One-time supported A2 TMAX=40 setup through MC02; never enables motors."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
import os
from pathlib import Path
import tempfile

from .arm import ArmBus, CAP_A2_RANGE_SETUP, CONTROL_REGISTERS, decode_register
from .arm_bench import BenchSession, preflight, read_settings, validate_plan


def save_report(path, report):
    """Durable evidence before the first write, and at each completed phase."""
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temp = Path(stream.name)
        try:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temp.unlink(missing_ok=True)
            raise
    try:
        os.replace(temp, path)
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        temp.unlink(missing_ok=True)


def capture_settings(session):
    """All routing/ranges/modes, plus raw A2 controls and live bus voltage."""
    observed = [read_settings(session.bus, m['bus'], m['can_id'], m['feedback_id'],
                              session.stop) for m in session.motors]
    control = {}
    for register in CONTROL_REGISTERS:
        session.stop()
        reply = session.bus.read_register(2, 2, register)
        if reply is None or reply[0] != 18:
            raise RuntimeError(f'A2 control register {register}: missing/wrong reply; setup stopped')
        control[register] = reply[1]
    return dict(discovery=observed, a2_control_raw=control)


def configuration_snapshot(settings):
    """Compare configuration exactly; retain live VBus in the original evidence."""
    # Damiao register 60 is read-only bus voltage, not a saved parameter.
    return {**settings, 'a2_control_raw': {
        register: raw for register, raw in settings['a2_control_raw'].items()
        if register != 60
    }}


def configure_a2_range(bus, baseline, report, checkpoint):
    """Requires fresh MCU, complete rig and a durable checkpoint callback."""
    if not bus.capabilities & CAP_A2_RANGE_SETUP:
        raise RuntimeError('Flash A2 range-setup firmware (capability 0x400000) first')
    info = report['base_info'] = bus.base_info()
    if info['configured_mask'] or info['id_session_locked'] or info.get('a2_range_setup_flags'):
        raise RuntimeError('Start after a motor-supply and MC02 power cycle; setup will not interrupt an existing owner')
    states = [bus.state(i) for i in range(6)]
    if any(s.enabled or s.fault or s.stopping for s in states):
        raise RuntimeError('MC02 is enabled, stopping or faulted; no automatic stop/reset')
    plan = deepcopy(baseline)
    validate_plan(plan, all_motors=True)
    if (plan.get('active_joints') != list(range(1, 7)) or
            [m['torque_trip_nm'] for m in plan['motors']] != [20, 12, 12, 3.5, 3.5, 3.5]):
        raise ValueError('Setup requires the unchanged six-axis baseline profile and rated-reference trips')
    reply = bus.read_register(2, 2, 23)
    if reply is None or reply[0] != 18:
        raise RuntimeError('A2 TMAX: missing/wrong reply; no writes sent')
    actual = decode_register(23, reply[1])
    report['a2_tmax_before_nm'] = actual
    if actual not in (28.0, 40.0):
        raise RuntimeError(f'A2 reports unexpected TMAX={actual:g}; only the known 28-to-40 setup is supported')
    plan['motors'][1]['ranges']['tmax'] = actual
    motors = plan['motors']
    observed = preflight(bus, motors)
    session = BenchSession(bus, motors, observed, report)
    try:
        initial = session.probe_disabled()
        report['disabled_before'] = {str(i+1): asdict(s) for i, s in initial.items()}
        before = report['settings_before'] = capture_settings(session)
        checkpoint()
        if actual == 40:
            report.update(passed=True, already_at_40=True, motor_parameter_write_attempted=False,
                          power_cycle_required=True, persistence_verified=False)
            return
        session.stop()
        report['motor_parameter_write_attempted'] = True
        checkpoint()  # Do not change any parameter if recording evidence fails.
        session.stop()  # Disk sync must not make the disabled feedback stale.
        bus.setup_a2_torque_range()
        report['volatile_readback_verified'] = True
        after = capture_settings(session)
        expected = deepcopy(before)
        expected['discovery'][1][23] = 40.0
        expected_config = configuration_snapshot(expected)
        if configuration_snapshot(after) != expected_config:
            report['settings_after_write'] = after
            raise RuntimeError('A setting other than A2 TMAX changed; no save sent, power-cycle before further use')
        report['settings_after_write'] = after
        report['save_attempted'] = True
        checkpoint()
        session.stop()
        bus.setup_a2_torque_range(save=True)
        report['save_acknowledged'] = True
        final = report['settings_after_save'] = capture_settings(session)
        if configuration_snapshot(final) != expected_config:
            raise RuntimeError('Post-save settings mismatch; retain evidence and do not start motion')
        report.update(passed=True, power_cycle_required=True, persistence_verified=False,
                      a2_tmax_after_nm=40.0, motion_locked_until_mcu_restart=True)
    except BaseException as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        # No mode/current/gain changes to restore; only disable acknowledgements.
        try:
            final_states = session.stop()
            report['disabled_after'] = {str(i+1): asdict(s) for i, s in final_states.items()}
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
                        help='explicitly write A2 TMAX=40 and save motor parameters')
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args(argv)
    report = dict(passed=False, operation='A2 TMAX 28 to 40', motor_enabled_by_setup=False,
                  motor_parameter_write_attempted=False, save_attempted=False,
                  save_acknowledged=False, persistence_verified=False)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as stream:
        json.dump(report, stream)
    checkpoint = lambda: save_report(args.report, report)
    checkpoint()
    baseline = Path(__file__).resolve().parents[1] / 'robot_description/config/arm_rig_a1_a6.json'
    try:
        with ArmBus(args.port) as bus:
            configure_a2_range(bus, json.loads(baseline.read_text()), report, checkpoint)
    except (OSError, RuntimeError, ValueError) as exc:
        report.update(passed=False, error=str(exc))
        checkpoint()
        print(f'A2 SETUP STOPPED: {exc}\nEvidence: {args.report}')
        return 1
    checkpoint()
    print('A2 TMAX reads 40. Motors remain disabled. No motion was commanded.')
    print('Power-cycle BOTH the motor supply and MC02, then run the 30 Nm profile readback check.')
    print(f'Evidence: {args.report}')
    return 0
