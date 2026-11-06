"""Explicit single-joint position port for unloaded arm commissioning.

Raw output-shaft radians and rad/s. No ROS, IK, background thread, calibration
writes or implicit re-enable. Adapter reuses the validated routed position-
velocity transport/lifecycle; production loaded-arm integration is separate.
"""
from dataclasses import dataclass, replace
import math
import time
from typing import Protocol
from .arm import ArmBus
from .arm_bench import BenchSession, preflight, validate_plan


@dataclass(frozen=True)
class JointPosition:
    joint: int
    position_rad: float
    origin_rad: float
    lower_rad: float
    upper_rad: float
    age_ms: int
    enabled: bool

    @property
    def relative_degrees(self):
        return math.degrees(self.position_rad-self.origin_rad)


class MotionCancelled(RuntimeError):
    """Fresh operator authorization was withdrawn; normal acknowledged stop."""


class PositionPort(Protocol):
    """Ports use physical A1..A6 numbers, raw radians, explicit selection authorization."""
    selected: int
    armed: bool
    moving: bool
    def select(self, joint: int) -> None: ...
    def arm(self) -> None: ...
    def move_to(self, angle_rad: float, speed_rad_s: float) -> None: ...
    def move_by(self, delta_rad: float, speed_rad_s: float) -> None: ...
    def poll(self) -> bool: ...
    def positions(self) -> list[JointPosition]: ...
    def cached_positions(self) -> list[JointPosition]: ...
    def release(self) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


def finite(value, label):
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return value


class ArmPositionPort:
    """One-device-owner adapter. Call poll() frequently while a move is active.

    Each move ends with acknowledged disable; authorization lasts until disarm.
    No automatic hold torque or
    multi-joint motion. arm() is operator authorization, not a CAN enable.
    Session coordinates are never saved as the motor's zero.
    """
    def __init__(self, plan, *, port='/dev/ttyACM0', bus=None, window_degrees=90,
                 motion_allowed=None):
        self.motors=validate_plan(plan,execute=True,all_motors=True,group_first=True)
        if any(m['can_id']!=i+1 or m['feedback_id']!=17+i for i,m in enumerate(self.motors)):
            raise ValueError('Position port requires verified A1..A6 order and DOF-matched IDs')
        finite(window_degrees,'window')
        if not 0 < window_degrees <= 90: raise ValueError('window must be >0 and <=90 degrees')
        self.window=math.radians(window_degrees)
        self.selected=1; self.armed=False; self.moving=False; self.closed=False
        self.motion_allowed=motion_allowed
        self._display_rows={}  # Samples already obtained by the control owner.
        self.report={'events':[],'disable_acknowledged':False,'mode_restoration_verified':False}
        self.bus=bus if bus is not None else ArmBus(port)
        self.session=None; self.target=None; self.settled=None
        try:
            observed=preflight(self.bus,self.motors)
            self.session=BenchSession(self.bus,self.motors,observed,self.report)
            self.session.prepare(jog_window_rad=self.window)
        except BaseException:
            self.close()
            raise

    def _event(self, event, **values):
        self.report['events'].append(dict(event=event,at_monotonic_s=time.monotonic(),**values))

    def _check(self):
        if self.closed: raise RuntimeError('Position port is closed')

    def _check_intent(self):
        if self.motion_allowed is not None and not self.motion_allowed():
            raise MotionCancelled('Operator intent released, changed or expired')

    def select(self, joint):
        self._check()
        if type(joint) is not int or not 1 <= joint <= 6: raise ValueError('Select A1..A6')
        self.stop(); self.selected=joint
        self._event('select',joint=joint)

    def arm(self):
        self._check(); self.stop()
        if any(self.bus.state(i).fault for i in range(6)):
            raise RuntimeError('Latched motor/controller fault; explicit recovery required')
        self.armed=True; self._event('armed',joint=self.selected)

    def _limits(self, slot):
        origin=self.session.origins[slot]
        return origin-self.window,origin+self.window

    def move_by(self, delta_rad, speed_rad_s):
        finite(delta_rad,'delta')
        self._check()
        if not self.armed or self.moving: raise RuntimeError('Arm selected joint; wait for current step to finish')
        current=self._stop_session()[self.selected-1].position
        self.move_to(current+delta_rad,speed_rad_s)

    def move_to(self, angle_rad, speed_rad_s):
        self._check(); finite(angle_rad,'angle'); finite(speed_rad_s,'speed')
        if not self.armed or self.moving: raise RuntimeError('Arm selected joint; wait for current step to finish')
        i=self.selected-1; lo,hi=self._limits(i)
        if not lo <= angle_rad <= hi: raise ValueError('Target exceeds session joint window')
        if not .001 <= speed_rad_s <= self.motors[i]['speed_trip_rad_s']:
            raise ValueError('Speed exceeds configured joint envelope')
        state=self._stop_session()[i]
        duration=abs(angle_rad-state.position)/speed_rad_s+2
        if duration>12: raise ValueError('Move would exceed the bounded 12-second operation')
        try:
            self._check_intent()
            self.bus.command(i,state.position,velocity=speed_rad_s)
            self._check_intent()
            self.bus.enable_bench_group([i])
            deadline=time.monotonic()+.4
            while True:
                self._check_intent()
                self.bus.command(i,state.position,velocity=speed_rad_s)
                sample=self.session.sample(i)
                if sample.status==1 and sample.enable_confirmed: break
                if time.monotonic()>=deadline: raise TimeoutError('Selected joint enable not confirmed')
                time.sleep(.005)
            self.target=angle_rad; self.speed=speed_rad_s
            self.deadline=time.monotonic()+duration; self.settled=None; self.moving=True
            self._check_intent()
            self.bus.command(i,angle_rad,velocity=speed_rad_s)
            self._event('move',joint=self.selected,target_rad=angle_rad,speed_rad_s=speed_rad_s)
        except BaseException:
            self.stop(); raise

    def poll(self):
        self._check()
        if not self.moving: return False
        i=self.selected-1
        try:
            self._check_intent()
            if time.monotonic()>=self.deadline: raise TimeoutError('Position move did not settle before deadline')
            self.bus.command(i,self.target,velocity=self.speed)
            sample=self.session.sample(i)
            self._remember_sample(i,sample)
            if sample.status!=1 or not sample.enable_confirmed: raise RuntimeError('Joint lost enabled confirmation')
            reached=abs(sample.position-self.target)<=math.radians(.5) and abs(sample.velocity)<=.03
            now=time.monotonic()
            self.settled=(now if self.settled is None else self.settled) if reached else None
            if self.settled is not None and now-self.settled>=.1:
                self._event('reached',joint=self.selected,position_rad=sample.position,target_rad=self.target)
                self._stop_session(); self.moving=False; self.target=None
            return self.moving
        except BaseException:
            self.stop(); raise

    def _remember_sample(self, slot, state):
        lo,hi=self._limits(slot)
        row=JointPosition(slot+1,state.position,self.session.origins[slot],lo,hi,
                          state.age_ms,bool(state.enabled))
        self._display_rows[slot]=(row,time.monotonic())
        return row

    def _stop_session(self):
        states=self.session.stop()
        for slot,state in states.items():
            self._remember_sample(slot,state)
        return states

    def cached_positions(self):
        """Copy existing control samples, aging them without USB or CAN requests."""
        now=time.monotonic()
        return [replace(row,age_ms=row.age_ms+max(0,round((now-at)*1000)))
                for _,(row,at) in sorted(self._display_rows.items())]

    def positions(self):
        self._check()
        result=[]
        for i in range(6):
            s=self.bus.state(i)
            if s.fault: raise RuntimeError(s.fault_detail())
            result.append(self._remember_sample(i,s))
        return result

    def release(self):
        """Disable motion, retaining selection authorization only on success."""
        authorized=self.armed
        self.stop()
        self.armed=authorized

    def stop(self):
        self.armed=False; self.moving=False; self.target=None
        if self.session is not None and self.session.configured:
            self._stop_session()
        self._event('stop')

    def close(self):
        if self.closed:return
        try:
            self.armed=False; self.moving=False
            if self.session is not None:self.session.cleanup()
        finally:
            self.closed=True
            self.bus.close()

    def __enter__(self): return self
    def __exit__(self,*args): self.close()


class SimulatedPositionPort:
    """Offline adapter for the identical selection/position/control interface."""
    def __init__(self, window_degrees=90):
        finite(window_degrees,'window')
        if not 0<window_degrees<=90:raise ValueError('window must be >0 and <=90 degrees')
        self.window=math.radians(window_degrees);self.values=[0.]*6
        self.selected=1;self.armed=False;self.moving=False;self.closed=False
        self.report={'simulation':True,'events':[]};self.last=time.monotonic()
    def select(self,joint):
        if type(joint) is not int or not 1<=joint<=6:raise ValueError('Select A1..A6')
        self.stop();self.selected=joint
    def arm(self):self.stop();self.armed=True
    def move_to(self,angle_rad,speed_rad_s):
        finite(angle_rad,'angle');finite(speed_rad_s,'speed')
        if not self.armed or self.moving:raise RuntimeError('Arm selected joint; wait for current step to finish')
        if not -self.window<=angle_rad<=self.window:raise ValueError('Target exceeds session joint window')
        if not .001<=speed_rad_s<=1.5:raise ValueError('Speed exceeds simulation envelope')
        if abs(angle_rad-self.values[self.selected-1])/speed_rad_s+2>12:raise ValueError('Move exceeds operation deadline')
        self.target=angle_rad;self.speed=speed_rad_s;self.last=time.monotonic();self.moving=True
    def move_by(self,delta_rad,speed_rad_s):
        finite(delta_rad,'delta');self.move_to(self.values[self.selected-1]+delta_rad,speed_rad_s)
    def poll(self):
        if self.closed:raise RuntimeError('Position port is closed')
        now=time.monotonic();dt=now-self.last;self.last=now
        if self.moving:
            i=self.selected-1;delta=self.target-self.values[i];step=self.speed*dt
            self.values[i]+=max(-step,min(step,delta))
            if abs(delta)<=step:self.moving=False
        return self.moving
    def positions(self):
        return [JointPosition(i+1,v,0.,-self.window,self.window,0,self.moving and i==self.selected-1) for i,v in enumerate(self.values)]
    def cached_positions(self):return self.positions()
    def release(self):self.moving=False
    def stop(self):self.armed=False;self.moving=False
    def close(self):
        self.stop();self.closed=True
        self.report.update(final_positions_rad=list(self.values),selected_joint=self.selected)
