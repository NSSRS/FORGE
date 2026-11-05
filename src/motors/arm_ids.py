"""Assign DOF-matched IDs to one isolated, disabled Damiao motor. Never enables."""
import argparse
import json
from pathlib import Path
import struct
import time

from .arm import ArmBus
from .arm_bench import BenchSession

REGS = (7, 8, 9, 10, 21, 22, 23, 31)


def read_settings(bus, channel, mid, rx, stop=None):
    result = {}
    for reg in REGS:
        if stop:
            stop()
        reply = bus.read_register(channel, mid, reg)
        if reply is None or reply[0] != rx:
            raise RuntimeError(f'ID/readback mismatch at register {reg}; no blind retry')
        result[reg] = reply[1] if reg < 21 else struct.unpack('<f', struct.pack('<I', reply[1]))[0]
    if result[7] != rx or result[8] != mid or result[10] not in (1, 2):
        raise RuntimeError('Unexpected IDs or mode; no ID writes allowed')
    return result


def assign(bus, *, dof, channel, current_id, current_feedback, report, verify=False):
    if not verify and not bus.capabilities & 1024:
        raise RuntimeError('Flash ID-commissioning firmware (HELLO capability 0x400) first')
    for slot in range(6):
        state = bus.state(slot)
        if state.enabled or state.stopping or state.fault or state.registers:
            raise RuntimeError('Reset MC02 before this isolated commissioning session')
    # Detect other responding IDs. Identical factory IDs cannot be reliably
    # distinguished by CAN discovery: physical isolation is also mandatory.
    found = []
    for mid in range(16):
        reply = bus.read_register(channel, mid, 8)
        if reply is not None:
            if reply[1] != mid:
                raise RuntimeError('Inconsistent discovery reply; inspect bus isolation')
            found.append((mid, reply[0]))
    expected = (dof, 16+dof) if verify else (current_id, current_feedback)
    if found != [expected]:
        raise RuntimeError(f'Expected one isolated motor {expected}; discovered {found}')
    mid, rx = expected
    before = read_settings(bus, channel, mid, rx)
    report['before'] = before
    if verify:
        report['power_cycle_verified'] = True
        return
    model = '8009P' if dof==1 else '4340P' if dof<=3 else '4310P'
    motor = dict(model=model, bus=channel, can_id=mid, feedback_id=rx,
                 speed_trip_rad_s=.1, torque_trip_nm=.1)
    session = BenchSession(bus, [motor], [before], report)
    session.configured = [0]
    try:
        pmax = min(12.5, before[21])
        bus.prepare_routed_bench(0, motor, -pmax, pmax, before[10])
        bus.probe(0)
        session.poll(0, lambda s: s.registers & 15 == 15, 'range and original-mode probe')
        session.stop()
        bus.id_operation(0)
        report['motion_locked_until_mc02_reset'] = True
        for reg, value in ((7, 16+dof), (8, dof)):
            if (rx if reg==7 else mid) == value:
                continue
            session.stop()
            old_mid, old_rx = mid, rx
            # Record intent before USB ACK: writes can succeed even if ACK is lost.
            report.setdefault('attempted_writes', []).append({'register': reg, 'value': value})
            bus.id_operation(reg, value)
            if reg==7:
                rx=value
            else:
                mid=value
            ack = bus._diagnostic_result(channel, old_mid, reg)
            if ack is None or ack[1] != value or ack[0] not in (old_rx, rx):
                raise RuntimeError('ID write acknowledgement missing/mismatched; inspect reported attempted IDs before retrying')
            session.stop()
            if bus.read_register(channel, mid, reg) != (rx, value):
                raise RuntimeError('ID write readback failed; no save issued')
        after = read_settings(bus, channel, mid, rx, session.stop)
        report['after'] = after
        if any(after[r] != before[r] for r in REGS if r not in (7, 8)):
            raise RuntimeError('Non-ID setting changed; refusing parameter save')
        session.stop()
        report['save_attempted'] = True
        bus.id_operation(0xaa)
        ack = bus._diagnostic_result(channel, mid, 1)
        if ack is None or ack[0] != rx:
            raise RuntimeError('Save acknowledgement missing; persistence unknown; do not blindly repeat save')
        time.sleep(.05)  # Manual: flash write up to 30 ms.
        report['save_acknowledged'] = True
        session.stop()
        report['saved_readback'] = read_settings(bus, channel, mid, rx, session.stop)
        report['ids_written'] = True
        print(f'DOF{dof}: command ID {mid}, feedback ID 0x{rx:02X}; save acknowledged. Power-cycle verification still required.', flush=True)
    finally:
        try:
            session.stop()
            report['disable_acknowledged'] = True
        except (OSError, RuntimeError, ValueError) as exc:
            report['cleanup_error'] = str(exc)
            raise RuntimeError('Disable acknowledgement missing: turn motor power OFF') from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dof', type=int, choices=range(1,7), required=True)
    parser.add_argument('--bus', type=int, choices=(1,2,3))
    parser.add_argument('--current-id', type=int, default=1)
    parser.add_argument('--current-feedback', type=int, default=0)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--isolated', action='store_true', help='Only this physical motor is on the selected CAN bus')
    parser.add_argument('--fresh-power-cycle', action='store_true', help='Motor power cycled since its last test/write; no volatile parameter changes since then')
    action=parser.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--verify', action='store_true', help='Read-only verification after motor power cycle and MCU reset')
    parser.add_argument('--report', type=Path)
    args=parser.parse_args(argv)
    channel=args.bus if args.bus is not None else (1 if args.dof==1 else 2)
    if (args.dof==1)!=(channel==1):
        parser.error('DOF1 must be on CAN1; DOF2..6 must be on CAN2 or CAN3')
    if not 1<=args.current_id<=15 or not (args.current_feedback==0 or 16<=args.current_feedback<2047):
        parser.error('Invalid current IDs')
    print(f'DOF{args.dof}, CAN{channel}: target command={args.dof}, feedback=0x{16+args.dof:02X}. No motor enable commands.', flush=True)
    if not args.execute and not args.verify:
        print('Offline plan only. --execute requires --isolated and --fresh-power-cycle. Save persists ALL current motor parameters.')
        return 0
    if not args.isolated or not args.fresh_power_cycle or args.report is None:
        parser.error('Execution/verification requires --isolated --fresh-power-cycle --report PATH')
    report=dict(dof=args.dof, bus=channel, passed=False, ids_written=False,
                save_acknowledged=False, power_cycle_verified=False)
    result=2
    try:
        with ArmBus(args.port) as bus:
            assign(bus,dof=args.dof,channel=channel,current_id=args.current_id,
                   current_feedback=args.current_feedback,report=report,verify=args.verify)
        report['passed']=True
        result=0
    except (OSError, RuntimeError, ValueError) as exc:
        report['error']=str(exc)
    except KeyboardInterrupt:
        report['error']='Interrupted; inspect attempted writes before retrying'
        result=130
    finally:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return result
