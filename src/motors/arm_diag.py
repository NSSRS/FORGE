"""Read motor registers and capture raw CAN reply evidence. Never enables or writes."""
import argparse
import json
from pathlib import Path
from .arm import ArmBus


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/ttyACM0')
    p.add_argument('--bus', type=int, choices=(1,2,3), default=1)
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--report', type=Path, required=True)
    args=p.parse_args(argv)
    report=dict(bus=args.bus, command_id=args.id, register_reads=[])
    try:
        with ArmBus(args.port) as bus:
            report['capabilities']=hex(bus.capabilities)
            for slot in range(6):
                state=bus.state(slot)
                if state.enabled or state.stopping or state.registers:
                    raise RuntimeError('Reset MC02 before this read-only capture')
            for reg in (7,8,9,10,21,22,23,31):
                before=bus.can_rx_trace(args.bus)
                reply=bus.read_register(args.bus,args.id,reg)
                after=bus.can_rx_trace(args.bus)
                row=dict(register=reg,reply=reply,rx_count_delta=(after['count']-before['count']) & 0xffffffff,last_rx=after)
                report['register_reads'].append(row)
                print(json.dumps(row),flush=True)
        return 0
    except (OSError,RuntimeError,ValueError) as exc:
        report['error']=str(exc)
        print(str(exc),flush=True)
        return 2
    finally:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n')
