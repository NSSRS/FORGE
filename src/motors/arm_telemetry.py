"""Passive Damiao base telemetry and classic-CAN capacity estimates.

Use the existing ArmBus owner for live monitoring; never create a competing
controller or refresh movement commands from the telemetry path.
"""
import argparse
import json
import math
from pathlib import Path
import time
from .arm import ArmBus


def can_budget(motors, update_hz=500, bitrate=1_000_000):
    """Eight-byte standard command+feedback, including 3-bit intermission.

    111..135 bits/frame accounts for bit stuffing, not errors, retransmissions,
    diagnostic traffic, other nodes, arbitration delays or scheduler jitter.
    This is a planning estimate, not measured bus utilization.
    """
    if type(motors) is not int or not 0 <= motors <= 6:
        raise ValueError('motor count must be 0..6')
    if any(type(x) not in (int,float) or not math.isfinite(x) or x <= 0
           for x in (update_hz,bitrate)):
        raise ValueError('positive finite update rate and bitrate required')
    frames=motors*update_hz*2
    return dict(motors=motors,update_hz=update_hz,bitrate=bitrate,
                frames_per_second=frames,estimated_load_min=frames*111/bitrate,
                estimated_load_max=frames*135/bitrate)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',help='Explicit idle-controller USB port; omit for offline capacity estimate')
    p.add_argument('--samples',type=int,default=10)
    p.add_argument('--interval',type=float,default=.1)
    p.add_argument('--report',type=Path)
    args=p.parse_args(argv)
    if not 1<=args.samples<=10000 or not math.isfinite(args.interval) or args.interval<.02:
        p.error('samples 1..10000, interval at least 0.02 seconds')
    report={'capacity_estimates':[can_budget(n) for n in (1,3,5)],'samples':[]}
    try:
        if args.port:
            with ArmBus(args.port) as bus:
                report['base_info']=bus.base_info()
                if report['base_info']['api_version']!=1:
                    raise RuntimeError('Unsupported base telemetry API version')
                for i in range(args.samples):
                    start=time.monotonic()
                    row={'host_monotonic_s':start,'snapshot':bus.snapshot(),
                         'buses':{str(b):bus.bus_health(b) for b in (1,2,3)}}
                    report['samples'].append(row)
                    print(json.dumps(row),flush=True)
                    if i+1<args.samples:time.sleep(max(0,args.interval-(time.monotonic()-start)))
        else: print(json.dumps(report['capacity_estimates'],indent=2))
        return 0
    except (OSError,RuntimeError,ValueError) as exc:
        report['error']=str(exc); print(str(exc),flush=True); return 2
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
