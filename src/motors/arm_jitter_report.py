"""Reproduce the saved November 12 group-hold jitter analysis; files only.

This preserves the trial-specific observations and hypotheses from that session.
It is not a generic diagnosis of another run. The report hash prevents applying
those historical annotations to unrelated evidence. Current status is in
ARM_SESSION_HANDOFF.md; no hardware transport is imported or opened here.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

SOURCE_SHA256 = '1504c7ff456494bcdfc64d1cc8a5ae03bc9b937bd2c05d535d357eb260cb53ed'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path, help='original group-hold-20261112-165434/holding.json')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--plan', type=Path,
                        default=Path(__file__).resolve().parents[1]/'robot_description/config/arm_rig_a1_a6.json')
    args = parser.parse_args(argv)
    path = args.report.resolve()
    if hashlib.sha256(path.read_bytes()).hexdigest() != SOURCE_SHA256:
        parser.error('This historical analysis requires the original 2026-11-12 group-hold report; hash mismatch.')
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    d=json.loads(path.read_text())
    segments=[]
    for e in d['events']:
        if e['event']!='jog':continue
        start=e['at_monotonic_s']
        end=next((x['at_monotonic_s'] for x in d['events'] if x['at_monotonic_s']>start and x['event'] in ('jog','hold','stop','select')),math.inf)
        rows=[r for r in d['motion_samples'] if r['joint']==e['joint'] and r['phase']=='jog' and start<=r['at_monotonic_s']<end]
        if len(rows)<15:continue
        segments.append((e,rows))

    stats={}
    models=json.loads(args.plan.read_text())['motors']
    for joint in range(1,7):
        e,rows=next((e,r) for e,r in segments if e['joint']==joint)
        t=[r['at_monotonic_s']-e['at_monotonic_s'] for r in rows]
        v=[e['direction']*math.degrees(r['velocity_rad_s']) for r in rows]
        steady=[r for r in rows if r['at_monotonic_s']-e['at_monotonic_s']>=1.]
        vsteady=[e['direction']*math.degrees(r['velocity_rad_s']) for r in steady]
        gap=[r['sample_gap_ms'] for r in rows if r['sample_gap_ms'] is not None]
        avg=e['direction']*math.degrees((steady[-1]['position_rad']-steady[0]['position_rad'])/(steady[-1]['at_monotonic_s']-steady[0]['at_monotonic_s']))
        stats[str(joint)]={'model':models[joint-1]['model'],
            'segment_start_monotonic_s':e['at_monotonic_s'],'direction':e['direction'],
            'logged_samples':len(rows),'position_derived_mean_deg_s_after_first_second':avg,
            'reported_speed_min_max_deg_s_after_first_second':[min(vsteady),max(vsteady)],
            'reported_speed_std_deg_s_after_first_second':statistics.pstdev(vsteady),
            'median_abs_tracking_error_deg_after_first_second':statistics.median(abs(math.degrees(r['tracking_error_rad'])) for r in steady),
            'host_sample_gap_ms_min_median_max':[min(gap),statistics.median(gap),max(gap)],
            'maximum_feedback_age_ms':max(r['age_ms'] for r in rows),
            'lead_limited_samples':sum(r['lead_limited'] for r in rows)}
    fault=d['recoveries'][0]['failure_before']['fault_snapshot']
    evidence={'source_report':str(path),'hardware_commands_sent':False,
        'operator_observation':'A1/A4/A5/A6 jitter only while moving; holding steady. A2/A3 fine.',
        'representative_segments':stats,
        'fault':{'joint':fault['slot']+1,'reported_velocity_deg_s':math.degrees(fault['velocity_rad_s']),
            'target_error_deg':math.degrees(fault['target_rad']-fault['position_rad']),
            'command_age_ms':fault['command_age_ms'],'feedback_age_ms':fault['feedback_age_ms'],
            'selected_moving_joint':2,'faulted_joint_was_holding':True,'raw_snapshot':fault},
        'retention':{'dropped_samples':d['motion_samples_dropped'],'retained_samples':len(d['motion_samples']),
            'retained_span_s':d['motion_samples'][-1]['at_monotonic_s']-d['motion_samples'][0]['at_monotonic_s']},
        'interpretation':{'confirmed':'Motion speed ripple at regular host cadence; fresh feedback; no lead cap activation in retained jogs. A1 ripple predates group mode.',
            'leading_hypothesis':'50 Hz stepped position targets interacting with model-dependent drive position/velocity loop response.',
            'unresolved':'Insufficient sample rate to prove target-step excitation, identify oscillation frequency, or explain the separate A4 hold overspeed.',
            'next_isolation':'Capture high-rate drive feedback and transmitted targets around a short supported single-joint jog; compare current target steps with MCU-side interpolation. Read current drive tuning only with a supported disabled rig.'}}
    (out/'motor-jitter-evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps({'segments':stats,'fault_speed_deg_s':evidence['fault']['reported_velocity_deg_s']},indent=2))

    with (out/'motor-jitter-comparison.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['Joint','Model','Travel average deg/s','Reported speed minimum deg/s','Reported speed maximum deg/s','Reported speed std deg/s','Median following error deg'])
        for j,s in stats.items():writer.writerow([j,s['model'],s['position_derived_mean_deg_s_after_first_second'],*s['reported_speed_min_max_deg_s_after_first_second'],s['reported_speed_std_deg_s_after_first_second'],s['median_abs_tracking_error_deg_after_first_second']])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
