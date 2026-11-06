"""Measure existing Pocket channels without ROS publishers or motor access."""
import argparse
import json
from pathlib import Path
import statistics
import time
from .radio_input import parse_channels
from .pocket_input import PocketMapping


def changed_channel(samples):
    candidates = [i for i in range(16) if max(s[i] for s in samples)-min(s[i] for s in samples) >= 300]
    if len(candidates) != 1:
        raise ValueError(f'Expected one changing channel, found {[i+1 for i in candidates]}; move only the requested control')
    return candidates[0]


def axis_from_samples(center, positive, negative):
    index = changed_channel((center,positive,negative))
    lo, hi = sorted((positive[index],negative[index]))
    if min(center[index]-lo, hi-center[index]) < 200:
        raise ValueError('Axis needs a measured center with travel on both sides')
    return dict(channel=index+1, minimum=lo, center=center[index], maximum=hi,
                sign=1 if positive[index]>center[index] else -1, deadzone=.05)


def switch_from_samples(samples):
    index = changed_channel(samples)
    return dict(channel=index+1, values=[s[index] for s in samples], tolerance=80)


def capture(stream, prompt):
    input(prompt+'; hold it there and press Enter: ')
    stream.reset_input_buffer()
    samples = []
    end = time.monotonic()+2
    while time.monotonic() < end:
        line = stream.readline().decode('ascii', errors='replace')
        try:
            age, channels = parse_channels(line)
            if age < .15:
                samples.append(channels)
        except ValueError:
            continue
        if len(samples) >= 5:
            if any(max(s[i] for s in samples)-min(s[i] for s in samples)>60 for i in range(16)):
                raise ValueError('Controls moved during capture; repeat with all controls steady')
            return tuple(round(statistics.median(s[i] for s in samples)) for i in range(16))
    raise TimeoutError('No five fresh stable samples from the existing ESP32 radio stream')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    from serial import Serial
    config = json.loads(args.template.read_text())
    print('Controller calibration only. Stop other radio readers. No motor devices are opened.')
    print('Left vertical must self-center for release-to-zero chassis control. Confirm its spring return first.')
    with Serial(args.port, 115200, timeout=.2, exclusive=True) as stream:
        for name, label, positive, negative in (
                ('chassis_forward','LEFT vertical','up','down'),
                ('chassis_turn','LEFT horizontal','left','right'),
                ('arm_cw_horizontal','RIGHT horizontal','left (CW)','right (CCW)'),
                ('arm_cw_vertical','RIGHT vertical','up (CW)','down (CCW)')):
            center = capture(stream, f'Center both sticks for {label}')
            pos = capture(stream, f'Move only {label} fully {positive}')
            neg = capture(stream, f'Move only {label} fully {negative}')
            config['axes'][name] = axis_from_samples(center,pos,neg)
            print(name, config['axes'][name])
        input('Center both sticks and leave them centered. Press Enter: ')
        for name in ('sb','sc','se'):
            positions = ('released','held') if name == 'se' else ('top','middle','bottom')
            samples = [capture(stream,f'Set only {name.upper()} to {position}') for position in positions]
            config['switches'][name] = switch_from_samples(samples)
            print(name.upper(), config['switches'][name])
    PocketMapping(config)
    config['note'] = 'Radio channels measured by calibrate_pocket.py. CW signs are separate physical motor observations, viewed facing the output shaft.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix+'.tmp')
    temporary.write_text(json.dumps(config,indent=2)+'\n')
    temporary.replace(args.output)
    print(f'Saved {args.output}. CW raw signs remain {config["arm"]["cw_raw_signs"]}; no hardware validation flags changed.')


if __name__ == '__main__':
    main()
