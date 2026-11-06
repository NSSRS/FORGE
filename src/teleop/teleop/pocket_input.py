"""Pocket channel policy: left chassis, right arm, SB/SC selection, held SE.

Pure policy only. Deadlines retain the source age; publishing an output never
renews operator intent. Switch positions mean top/middle/bottom, not sorted PWM.
"""
from dataclasses import dataclass
import json
import math
from .radio_input import Axis, RadioCommand, parse_channels


@dataclass(frozen=True)
class Switch:
    channel: int
    values: tuple
    tolerance: int = 80

    def __post_init__(self):
        if type(self.channel) is not int or not 1 <= self.channel <= 16:
            raise ValueError('Calibrate switch channel numbers (1..16) first')
        if len(self.values) not in (2, 3) or any(type(v) is not int or not 0 <= v <= 2047 for v in self.values):
            raise ValueError('Switch values must contain two or three measured raw ticks')
        if type(self.tolerance) is not int or not 0 < self.tolerance <= 200:
            raise ValueError('Invalid switch tolerance')
        if any(abs(a-b) <= 2*self.tolerance for i,a in enumerate(self.values) for b in self.values[i+1:]):
            raise ValueError('Switch position bands overlap')

    def position(self, channels):
        matches = [i for i,v in enumerate(self.values) if abs(channels[self.channel-1]-v) <= self.tolerance]
        if len(matches) != 1:
            raise ValueError('Switch is outside its calibrated positions')
        return matches[0]


class PocketMapping:
    axis_names = ('chassis_forward', 'chassis_turn', 'arm_cw_horizontal', 'arm_cw_vertical')

    def __init__(self, config, *, hardware_arm=False):
        if config.get('version') != 1:
            raise ValueError('Pocket mapping version must be 1')
        self.axes = {name: Axis(**config['axes'][name]) for name in self.axis_names}
        self.switches = {name: Switch(**config['switches'][name]) for name in ('sb', 'sc', 'se')}
        if [len(self.switches[n].values) for n in ('sb','sc','se')] != [3,3,2]:
            raise ValueError('SB/SC require top/middle/bottom; SE requires released/held')
        channels = [a.channel for a in self.axes.values()] + [s.channel for s in self.switches.values()]
        if len(set(channels)) != len(channels):
            raise ValueError('Each stick axis and switch needs a distinct channel')
        self.timeout = config.get('timeout_s', .15)
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= .15:
            raise ValueError('Pocket timeout must be in (0, 0.15] seconds')
        arm = config['arm']
        self.cw_signs = tuple(arm['cw_raw_signs'])
        if len(self.cw_signs) != 6 or any(type(s) is not int or s not in (-1,0,1) for s in self.cw_signs):
            raise ValueError('Provide six CW raw-shaft signs: -1/+1, or 0 if unmeasured')
        if hardware_arm and 0 in self.cw_signs:
            raise ValueError('Measure cw_raw_signs for all six motors before hardware arm jogging')
        self.speed = math.radians(arm.get('speed_deg_s', 45))
        self.window = arm.get('window_deg', 90)
        if not math.isfinite(self.speed) or not 0 < self.speed <= math.radians(45):
            raise ValueError('Pocket jog speed must be in (0,45] degrees/s')
        if not math.isfinite(self.window) or not 0 < self.window <= 90:
            raise ValueError('Pocket jog window must be in (0,90] degrees')
        if 2*math.radians(self.window)/self.speed+2 > 12:
            raise ValueError('Reduce jog window to fit the existing 12-second move bound')

    @classmethod
    def read(cls, path, **kwargs):
        with open(path) as stream:
            return cls(json.load(stream), **kwargs)


@dataclass(frozen=True)
class ArmIntent:
    generation: int
    joint: int | None
    direction: int  # +1 CW, -1 CCW, 0 stopped; converted only at the arm adapter.
    deadline: float


@dataclass(frozen=True)
class PocketOutput:
    chassis: tuple
    arm: ArmIntent
    status: str


class PocketCommand:
    def __init__(self, mapping, speed=.08, yaw_rate=.28):
        self.mapping = mapping
        self.chassis = RadioCommand(mapping.axes['chassis_forward'], mapping.axes['chassis_turn'],
                                    speed, yaw_rate, mapping.timeout)
        self.generation = 0
        self.reset()

    def reset(self):
        self.chassis.reset()
        self.deadline = 0.
        self.last_channels = None
        self.selected = None
        self.previous_switches = None
        self.was_held = False
        self.authorized = False
        self.neutral_since = None
        self.ready = False
        self.direction = 0
        self.generation += 1
        self.status = 'Waiting for fresh radio; move SB or SC to select a motor'

    def inhibit_arm(self):
        self.authorized = self.ready = False
        self.neutral_since = None
        if self.direction:
            self.generation += 1
        self.direction = 0

    def receive(self, line, now):
        self.output(now)
        try:
            age, channels = parse_channels(line)
            if age >= self.mapping.timeout:
                raise ValueError('Stale radio')
        except ValueError:
            self.reset()
            return
        self.last_channels = channels
        self.chassis.receive(line, now)
        self.deadline = now + self.mapping.timeout - age
        try:
            sb, sc, se = (self.mapping.switches[n].position(channels) for n in ('sb','sc','se'))
        except ValueError:
            self.inhibit_arm()
            self.previous_switches = None
            self.selected = None
            self.was_held = True  # An uncertain SE must not manufacture a press edge.
            self.status = 'Invalid switch sample; reselect with SE released'
            return
        held = se == 1
        switches = (sb,sc)
        changes = [] if self.previous_switches is None else [i for i in (0,1) if switches[i] != self.previous_switches[i]]
        self.previous_switches = switches
        if changes:
            self.inhibit_arm()
            self.generation += 1
            self.selected = switches[changes[0]] + (1 if changes[0] == 0 else 4) if len(changes) == 1 else None
        x = self.mapping.axes['arm_cw_horizontal'].value(channels)
        y = self.mapping.axes['arm_cw_vertical'].value(channels)
        signs = {1 if v > 0 else -1 for v in (x,y) if v != 0}
        neutral, conflict = not signs, len(signs) > 1
        if not held:
            self.authorized = False
            if neutral:
                if self.neutral_since is None:
                    self.neutral_since = now
                self.ready = now-self.neutral_since >= .30
            else:
                self.ready = False
                self.neutral_since = None
        elif not self.was_held:
            self.authorized = self.ready and neutral and self.selected is not None and not changes
            self.ready = False
            self.neutral_since = None
        if conflict:
            self.inhibit_arm()
        direction = next(iter(signs)) if signs and not conflict and held and self.authorized else 0
        if direction != self.direction:
            self.generation += 1
        self.direction = direction
        self.was_held = held
        if self.selected is None:
            self.status = 'Move only SB or SC to select A1–A6'
        elif conflict:
            self.status = f'A{self.selected}: conflicting directions; release SE and center right stick'
        elif self.authorized:
            self.status = f'A{self.selected}: '+('CW' if direction == 1 else 'CCW' if direction == -1 else 'ready, SE held')
        elif held:
            self.status = f'A{self.selected}: release SE first; then center right stick'
        elif not neutral:
            active = ', '.join(f'CH{self.mapping.axes[name].channel}={channels[self.mapping.axes[name].channel-1]}'
                for name in ('arm_cw_horizontal','arm_cw_vertical')
                if self.mapping.axes[name].value(channels) != 0)
            self.status = f'A{self.selected}: center right stick ({active})'
        elif self.ready:
            self.status = f'A{self.selected}: ready to prime; hold SE with right stick centered'
        else:
            self.status = f'A{self.selected}: keep right stick centered briefly; SE released'

    def output(self, now):
        if self.deadline and now >= self.deadline:
            self.reset()
        return PocketOutput(self.chassis.output(now), ArmIntent(self.generation, self.selected,
                            self.direction, self.deadline), self.status)
