"""Explicit single-joint position port for fixture-supported arm commissioning.

Raw output-shaft radians and rad/s. No ROS, IK, background thread, calibration
writes or implicit re-enable. Adapter reuses the validated routed position-
velocity transport/lifecycle; production loaded-arm integration is separate.
"""
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from copy import deepcopy
import math
import time
from typing import Protocol
from .arm import ArmBus, FAULT_REASONS, CAP_MANUAL_TRAVEL
from .arm_bench import (BenchSession, preflight, validate_plan, active_joints,
                        recover_fault, LatchedControllerFault, number as finite)

SUPPORTED_JOG_DEG_S = 5
TRIAL_JOG_DEG_S = 10
JOG_LEAD_S = .1

@dataclass(frozen=True)
class JointPosition:
    joint: int
    position_rad: float
    origin_rad: float
    lower_rad: float
    upper_rad: float
    age_ms: int
    enabled: bool
    velocity_rad_s: float | None = None
    torque_feedback_nm: float | None = None
    torque_trip_nm: float | None = None

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
    available_joints: tuple[int, ...]
    def select(self, joint: int) -> None: ...
    def arm(self) -> None: ...
    def move_to(self, angle_rad: float, speed_rad_s: float) -> None: ...
    def move_by(self, delta_rad: float, speed_rad_s: float) -> None: ...
    def jog(self, direction: int, speed_rad_s: float) -> None: ...
    def poll(self) -> bool: ...
    def positions(self) -> list[JointPosition]: ...
    def cached_positions(self) -> list[JointPosition]: ...
    def stationary_position(self) -> JointPosition: ...
    def release(self) -> None: ...
    def stop(self) -> None: ...
    def recover(self, report_path) -> None: ...
    def close(self) -> None: ...


def jog_window(window_degrees):
    if window_degrees is None:
        return None
    if not 0 < finite(window_degrees, 'window') <= 90:
        raise ValueError('window must be >0 and <=90 degrees')
    return math.radians(window_degrees)


class ArmPositionPort:
    """One-device-owner adapter. Call poll() frequently while a move is active.

    Each move ends with acknowledged disable; authorization lasts until disarm.
    No automatic hold torque or
    multi-joint motion. arm() is operator authorization, not a CAN enable.
    Session coordinates are never saved as the motor's zero.
    window_degrees=None explicitly selects manual travel without a startup window.
    Supported-arm mode defaults to <=5 degree/s and a 12-second continuous-hold cap;
    it still disables torque on stop and does not support the arm's weight.
    """
    def __init__(self, plan, *, port='/dev/ttyACM0', bus=None, window_degrees=90,
                 motion_allowed=None, allow_faulted_start=False, speed_trial=False):
        self.motors=validate_plan(plan,execute=True,all_motors=True,group_first=True)
        self.available_joints=active_joints(plan)
        self.slots=tuple(j-1 for j in self.available_joints)
        if any(m['can_id']!=i+1 or m['feedback_id']!=17+i for i,m in enumerate(self.motors)):
            raise ValueError('Position port requires verified A1..A6 order and DOF-matched IDs')
        self.window = jog_window(window_degrees)
        mode=plan.get('fixture_mode','unloaded')
        if mode not in ('unloaded','supported'):
            raise ValueError('Unknown fixture_mode')
        self.supported=mode=='supported'
        self.speed_trial=speed_trial
        if speed_trial and (not self.supported or window_degrees is not None):
            raise ValueError('Speed trials require supported manual travel')
        if self.supported and (plan.get('supported_confirmed') is not True or (window_degrees is not None and window_degrees>3)):
            raise ValueError('Supported arm requires explicit support confirmation and manual travel or window <=3 degrees')
        self.selected=self.available_joints[0]; self.armed=False; self.moving=False; self.closed=False
        self.motion_allowed=motion_allowed
        self._display_rows={}  # Samples already obtained by the control owner.
        self._failure_recorded=False
        self.recovery_required=False
        self._last_motion_log={}
        self.report={'events':[],'disable_acknowledged':False,'mode_restoration_verified':False,
                     'active_joints':self.available_joints,'fixture_mode':mode,
                     'travel_policy':'manual' if self.window is None else 'startup_window',
                     'window_degrees':window_degrees,
                     'control_mode':'position_velocity', 'commanded_torque_nm':None, 'speed_trial':speed_trial,
                     'motion_samples':[], 'motion_samples_dropped':0, 'motion_summary':{}}
        # Torque is feedback, not a command in mode 2. Record before cleanup
        # overwrites enabled feedback; do not introduce another transport reader.
        self.bus=bus if bus is not None else ArmBus(port)
        self.session=None; self._session_ready=False; self.target=None; self.settled=None; self.jog_direction=0
        try:
            if self.window is None and not self.bus.capabilities & CAP_MANUAL_TRAVEL:
                raise RuntimeError("Manual travel requires MC02 capability 0x8000; flash the updated firmware")
            self._prepare_session()
        except LatchedControllerFault as exc:
            if not allow_faulted_start:
                self.close()
                raise
            self.capture_failure(exc)  # Cached evidence only; leave latch for R.
        except BaseException:
            self.close()
            raise

    def _prepare_session(self):
        observed=dict(zip(self.slots,preflight(self.bus,[self.motors[i] for i in self.slots])))
        self.session=BenchSession(self.bus,self.motors,observed,self.report,slots=self.slots)
        self.session.prepare(jog_window_rad=self.window,manual_travel=self.window is None)
        self.report['session_joints']=[dict(
            joint=i+1, origin_rad=self.session.origins[i],
            command_lower_rad=self._limits(i)[0], command_upper_rad=self._limits(i)[1],
            firmware_lower_rad=self.session.bounds[i][0], firmware_upper_rad=self.session.bounds[i][1],
            torque_trip_nm=self.motors[i]['torque_trip_nm'],
            speed_trip_rad_s=self.motors[i]['speed_trip_rad_s'],
            startup_registers=observed[i], verified_motion_mode=2) for i in self.slots]
        self._session_ready=True

    def _event(self, event, **values):
        self.report['events'].append(dict(event=event,at_monotonic_s=time.monotonic(),**values))

    def _check(self):
        if self.closed: raise RuntimeError('Position port is closed')

    def _check_intent(self):
        if self.motion_allowed is not None and not self.motion_allowed():
            raise MotionCancelled('Operator intent released, changed or expired')

    @contextmanager
    def _guard_motion(self):
        try:
            yield
        except MotionCancelled:
            self.stop()
            raise
        except Exception as exc:
            raise RuntimeError(self.capture_failure(exc)) from exc
        except BaseException:
            self.stop()
            raise

    def select(self, joint):
        self._check()
        if type(joint) is not int or not 1 <= joint <= 6: raise ValueError('Select A1..A6')
        if joint not in self.available_joints:
            self.stop()
            raise ValueError(f'A{joint} is unavailable in this session')
        self.stop(); self.selected=joint
        self._event('select',joint=joint)

    def arm(self):
        self._check()
        if self.recovery_required:
            raise RuntimeError('Recovery required before arming')
        self.stop()
        if any(self.bus.state(i).fault for i in range(6)):
            raise RuntimeError('Latched motor/controller fault; explicit recovery required')
        self.armed=True; self._event('armed',joint=self.selected)

    def _limits(self, slot):
        if self.window is None:
            return self.session.bounds[slot]
        origin=self.session.origins[slot]
        return origin-self.window,origin+self.window

    def move_by(self, delta_rad, speed_rad_s):
        finite(delta_rad,'delta')
        self._check()
        if not self.armed or self.moving: raise RuntimeError('Arm selected joint; wait for current step to finish')
        current=self._stop_session()[self.selected-1].position
        self.move_to(current+delta_rad,speed_rad_s)

    def jog(self, direction, speed_rad_s):
        """Advance the mode-2 target while held; release/intent expiry stops."""
        if self.window is not None:
            raise ValueError('Continuous jog requires manual travel')
        if type(direction) is not int or direction not in (-1,1):
            raise ValueError('Jog direction must be -1 or +1')
        self._start_move(None,speed_rad_s,direction)

    def move_to(self, angle_rad, speed_rad_s):
        finite(angle_rad,'angle')
        self._start_move(angle_rad,speed_rad_s,0)

    def _start_move(self, angle_rad, speed_rad_s, direction):
        self._check(); finite(speed_rad_s,'speed')
        if not self.armed or self.moving: raise RuntimeError('Arm selected joint; wait for current step to finish')
        i=self.selected-1; lo,hi=self._limits(i)
        if angle_rad is not None and not lo <= angle_rad <= hi:
            raise ValueError('Target exceeds session joint window or protocol range')
        if not .001 <= speed_rad_s <= self.motors[i]['speed_trip_rad_s']:
            raise ValueError('Speed exceeds configured joint envelope')
        cap = TRIAL_JOG_DEG_S if self.speed_trial and direction else SUPPORTED_JOG_DEG_S
        if self.supported and speed_rad_s>math.radians(cap):
            raise ValueError(f'Supported-arm commissioning speed must not exceed {cap} degrees/s')
        state=self._stop_session()[i]
        if direction:
            angle_rad=state.position+direction*speed_rad_s*JOG_LEAD_S
            if not lo < angle_rad < hi:
                raise ValueError("Manual target exceeds protocol position range")
        duration=12. if direction else abs(angle_rad-state.position)/speed_rad_s+2
        if duration>12: raise ValueError('Move would exceed the bounded 12-second operation')
        with self._guard_motion():
            corridor_lo,corridor_hi=self.session.bounds[i]
            if not corridor_lo <= state.position <= corridor_hi:
                self.report['start_rejection']=dict(joint=i+1,position_rad=state.position,
                    origin_rad=self.session.origins[i],lower_rad=corridor_lo,upper_rad=corridor_hi,
                    target_rad=angle_rad,enable_sent=False)
                raise RuntimeError(f'A{i+1}: disabled joint shifted '
                    f'{math.degrees(state.position-self.session.origins[i]):+.2f} degrees from session start, '
                    'outside the startup corridor; no enable sent. Support/reposition the arm '
                    'and restart to capture a new session origin; bounds were not widened')
            self._check_intent()
            self.bus.command(i,state.position,velocity=speed_rad_s)
            self._check_intent()
            self.bus.enable_bench_group([i])
            deadline=time.monotonic()+.4
            while True:
                self._check_intent()
                self.bus.command(i,state.position,velocity=speed_rad_s)
                sample=self.session.sample(i)
                self._remember_sample(i,sample)
                if sample.status==1 and sample.enable_confirmed: break
                if time.monotonic()>=deadline: raise TimeoutError('Selected joint enable not confirmed')
                time.sleep(.005)
            self.target=angle_rad; self.speed=speed_rad_s; self.jog_direction=direction
            self.jog_started=time.monotonic(); self.jog_origin=state.position
            self.last_ramp_at=self.jog_started; self.lead_limited=False
            self.previous_sample_at=None
            self.duration=duration; self.deadline=time.monotonic()+duration; self.settled=None; self.moving=True
            self._check_intent()
            self.bus.command(i,angle_rad,velocity=speed_rad_s)
            self._event('jog' if direction else 'move',joint=self.selected,direction=direction,
                        start_position_rad=state.position,
                        target_rad=angle_rad,speed_rad_s=speed_rad_s,duration_limit_s=duration)
            self._record_motion_sample(i,sample)

    def _enabled_sample(self, slot):
        sample = self.session.sample(slot)
        self._remember_sample(slot, sample)
        if sample.status != 1 or not sample.enable_confirmed:
            raise RuntimeError('Joint lost enabled confirmation')
        return sample

    def _advance_jog(self, now, sample):
        if sample is None:
            self.target = self.jog_origin + self.jog_direction * self.speed * (now - self.jog_started + JOG_LEAD_S)
            return
        # Discard missed travel when shaft feedback lags during a speed trial.
        advance = self.speed * (now - self.last_ramp_at)
        room = self.jog_direction * (sample.position - self.target) + self.speed * JOG_LEAD_S
        self.lead_limited = room < advance
        self.target += self.jog_direction * min(advance, max(0., room))
        self.last_ramp_at = now

    def poll(self):
        self._check()
        if not self.moving: return False
        i=self.selected-1
        with self._guard_motion():
            self._check_intent()
            if time.monotonic()>=self.deadline:
                raise TimeoutError(f'{"Manual jog" if self.jog_direction else "Position move"} '
                                   f'host duration limit reached ({self.duration:.3f} s)')
            sample=None
            if self.jog_direction:
                now=time.monotonic()
                if self.speed_trial:
                    sample = self._enabled_sample(i)
                self._advance_jog(now, sample)
                lo,hi=self._limits(i)
                if not lo < self.target < hi:
                    raise RuntimeError('Manual target reached protocol position range')
            self._check_intent()
            self.bus.command(i,self.target,velocity=self.speed)
            if sample is None:
                sample = self._enabled_sample(i)
            self._record_motion_sample(i,sample)
            if self.jog_direction:
                return True  # A close tracking sample is not a release or a travel stop.
            reached=abs(sample.position-self.target)<=math.radians(.5) and abs(sample.velocity)<=.03
            now=time.monotonic()
            self.settled=(now if self.settled is None else self.settled) if reached else None
            if self.settled is not None and now-self.settled>=.1:
                self._event('reached',joint=self.selected,position_rad=sample.position,target_rad=self.target)
                self._stop_session(); self.moving=False; self.target=None
            return self.moving

    def capture_failure(self, error):
        """Stop first, then save cached fault evidence once, without clearing it.

        A motion-command rejection may be the first host observation of a
        firmware trip. Disable feedback overwrites current measurements, but
        fault_snapshot retains the sample that triggered the stop.
        """
        if self._failure_recorded:
            return self.report['error']
        self._failure_recorded=True
        self.recovery_required=True
        self.report['error_original']=self.report['error']=str(error)
        problems=[]
        try:
            self.stop()
        except Exception as exc:
            problems.append(f'stop: {exc}')
        try:
            snapshot=self.bus.fault_snapshot()
            self.report['fault_snapshot']=snapshot
            if snapshot and snapshot['fault']:
                slot=snapshot['slot']
                joint=f'A{slot+1}' if 0<=slot<6 else 'Controller'
                detail=f"{joint}: {FAULT_REASONS.get(snapshot['reason'], 'unknown fault')}"
                if snapshot['sample_flags'] & 1:
                    if snapshot['reason']==9:
                        detail+=(f"; measured {snapshot['torque_nm']:.4f} Nm, "
                                 f"limit {snapshot['torque_limit_nm']:.4f} Nm")
                    elif snapshot['reason']==8:
                        detail+=(f"; measured {snapshot['velocity_rad_s']:.4f} rad/s, "
                                 f"limit {snapshot['speed_limit_rad_s']:.4f} rad/s")
                        if snapshot['sample_flags'] & 2:
                            detail+=(f"; commanded {snapshot['command_velocity_rad_s']:.4f} rad/s "
                                     f"({math.degrees(snapshot['command_velocity_rad_s']):.2f} deg/s)")
                detail+=(f"; command age {snapshot['command_age_ms']} ms, "
                         f"feedback age {snapshot['feedback_age_ms']} ms; "
                         'fault latched, explicit recovery required')
                self.report['fault_summary']=detail
                self.report['error']=f'{detail}. Original error: {error}'
        except Exception as exc:
            problems.append(f'fault snapshot: {exc}')
        self.report['failure_states']={}
        for slot in self.session.configured if self.session is not None else ():
            try:
                self.report['failure_states'][slot]=asdict(self.bus.state(slot))
            except Exception as exc:
                problems.append(f'A{slot+1} state: {exc}')
        if problems:
            self.report['failure_capture_errors']=problems
            self.report['error']+='; diagnostics incomplete: '+'; '.join(problems)
        return self.report['error']

    def _remember_sample(self, slot, state):
        lo,hi=self._limits(slot)
        row=JointPosition(slot+1,state.position,self.session.origins[slot],lo,hi,
                          state.age_ms,bool(state.enabled), state.velocity, state.torque,
                          self.motors[slot]['torque_trip_nm'])
        self._display_rows[slot]=(row,time.monotonic())
        return row

    def _record_motion_sample(self, slot, state):
        """Use the owner's existing fresh feedback; never poll or extend a lease.

        Summary peaks use each host motion sample, not every 500 Hz CAN frame.
        A capped 10 Hz trace preserves enabled measurements after torque disable.
        """
        now=time.monotonic()
        sample=dict(at_monotonic_s=now,joint=slot+1,position_rad=state.position,
                    velocity_rad_s=state.velocity,torque_feedback_nm=state.torque,
                    requested_speed_rad_s=self.speed, direction=self.jog_direction, lead_limited=self.lead_limited,
                    tracking_error_rad=self.target-state.position, rotor_c=state.rotor_c, mos_c=state.mos_c,
                    sample_gap_ms=None if self.previous_sample_at is None else (now-self.previous_sample_at)*1000,
                    torque_trip_nm=self.motors[slot]['torque_trip_nm'],
                    target_rad=self.target,enabled=bool(state.enabled),
                    enable_confirmed=state.enable_confirmed,status=state.status,age_ms=state.age_ms)
        self.previous_sample_at=now
        key=str(slot+1)
        summary=self.report['motion_summary'].setdefault(key,dict(
            samples=0,first_at_monotonic_s=now,min_position_rad=state.position,
            max_position_rad=state.position,max_abs_torque_feedback_nm=0.,max_abs_velocity_rad_s=0.))
        summary['samples']+=1
        for field, value in (('max_abs_tracking_error_rad', abs(sample['tracking_error_rad'])),
                             ('max_sample_gap_ms', sample['sample_gap_ms'] or 0),
                             ('max_feedback_age_ms', state.age_ms)):
            summary[field]=max(summary.get(field, 0), value)
        summary['last_sample']=sample
        summary['min_position_rad']=min(summary['min_position_rad'],state.position)
        summary['max_position_rad']=max(summary['max_position_rad'],state.position)
        summary['max_abs_torque_feedback_nm']=max(summary['max_abs_torque_feedback_nm'],abs(state.torque))
        summary['max_abs_velocity_rad_s']=max(summary['max_abs_velocity_rad_s'],abs(state.velocity))
        if now-self._last_motion_log.get(slot,-math.inf)>=.1:
            rows=self.report['motion_samples']
            if len(rows)>=3000:
                del rows[:100]
                self.report['motion_samples_dropped']+=100
            rows.append(sample)
            self._last_motion_log[slot]=now

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

    def stationary_position(self):
        self._check()
        if self.moving or self.recovery_required:
            raise RuntimeError('Release motion and resolve faults before sampling')
        states = self._stop_session()
        for state in states.values():
            if state.fault:
                raise RuntimeError(state.fault_detail())
        return self._display_rows[self.selected-1][0]

    def positions(self):
        self._check()
        if self.recovery_required:
            return self.cached_positions()
        result=[]
        for i in self.slots:
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
        moving=self.moving
        stop_at=time.monotonic()
        last=self._display_rows.get(self.selected-1)
        self.armed=False; self.moving=False; self.target=None; self.jog_direction=0
        if self.session is not None and self.session.configured:
            states=self._stop_session()
            if moving and last:
                final=states[self.selected-1]
                self._event('motion_stop', joint=self.selected, request_at_monotonic_s=stop_at,
                            stationary_ack_ms=(time.monotonic()-stop_at)*1000,
                            last_motion_sample_age_ms=last[0].age_ms+(stop_at-last[1])*1000,
                            last_sample_to_stationary_rad=final.position-last[0].position_rad,
                            stationary_raw_rad=final.position)
        self._event('stop')

    def recover(self, report_path):
        """Explicit software recovery; stays disarmed and never sends a target.

        Reuses the single USB owner and the disabled CLI recovery interlocks.
        The caller must suspend polling/commands until this method returns.
        """
        self._check()
        if self.moving:
            raise RuntimeError('Release motion before recovery')
        if report_path is None:
            raise ValueError('Recovery requires a report path to preserve fault evidence')
        fields=('error','error_original','fault_snapshot','fault_summary','failure_states',
                'failure_capture_errors','start_rejection','stop_error')
        attempt=dict(at_monotonic_s=time.monotonic(),success=False,
                     failure_before={k:deepcopy(self.report[k]) for k in fields if k in self.report})
        self.report.setdefault('recoveries',[]).append(attempt)
        self.recovery_required=True
        try:
            self.armed=False;self.target=None;self.jog_direction=0
            # Recovery repairs invalidated range caches before waiting for
            # decoded stationary feedback (including after a lost reset ACK).
            recover_fault(self.bus,attempt,report_path)
            if not self._session_ready:
                if self.session is not None:
                    self.session.cleanup()
                    self.session=None
                self._prepare_session()
            else:
                observed=dict(zip(self.slots,preflight(self.bus,[self.motors[i] for i in self.slots])))
                self.session.prepare_motion(manual_travel=self.window is None,observed=observed)
            self._stop_session()
            attempt['success']=True
            for key in fields:self.report.pop(key,None)
            self._failure_recorded=False;self.recovery_required=False
            self._event('recovered',joint=self.selected,armed=False)
        except Exception as exc:
            attempt['error']=str(exc)
            self._failure_recorded=False
            raise RuntimeError(self.capture_failure(exc)) from exc
        finally:
            self.armed=False;self.moving=False;self.target=None;self.jog_direction=0

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
    def __init__(self, window_degrees=90, *, available_joints=tuple(range(1,7))):
        self.window = jog_window(window_degrees)
        self.values = [0.] * 6
        if (not available_joints or len(set(available_joints))!=len(available_joints) or
                any(type(j) is not int or not 1<=j<=6 for j in available_joints)):
            raise ValueError('available_joints must contain distinct A1..A6 numbers')
        self.available_joints=tuple(sorted(available_joints))
        self.selected=self.available_joints[0];self.armed=False;self.moving=False;self.closed=False
        self.report={'simulation':True,'events':[], 'travel_policy':'manual' if self.window is None else 'startup_window'}
        self.last=time.monotonic();self.jog_direction=0
    def select(self,joint):
        if type(joint) is not int or not 1<=joint<=6:raise ValueError('Select A1..A6')
        if joint not in self.available_joints:
            self.stop();raise ValueError(f'A{joint} is unavailable in this session')
        self.stop();self.selected=joint
    def arm(self):self.stop();self.armed=True
    def move_to(self,angle_rad,speed_rad_s):
        finite(angle_rad,'angle');finite(speed_rad_s,'speed')
        if not self.armed or self.moving:raise RuntimeError('Arm selected joint; wait for current step to finish')
        limit=12.5 if self.window is None else self.window
        if not -limit<=angle_rad<=limit:raise ValueError('Target exceeds session joint window or protocol range')
        if not .001<=speed_rad_s<=1.5:raise ValueError('Speed exceeds simulation envelope')
        if abs(angle_rad-self.values[self.selected-1])/speed_rad_s+2>12:raise ValueError('Move exceeds operation deadline')
        self.target=angle_rad;self.speed=speed_rad_s;self.last=time.monotonic();self.moving=True;self.jog_direction=0
    def jog(self,direction,speed_rad_s):
        if self.window is not None:raise ValueError('Continuous jog requires manual travel')
        if type(direction) is not int or direction not in (-1,1):raise ValueError('Jog direction must be -1 or +1')
        finite(speed_rad_s,'speed')
        if not .001<=speed_rad_s<=1.5:raise ValueError('Speed exceeds simulation envelope')
        if not self.armed or self.moving:raise RuntimeError('Arm selected joint; wait for current step to finish')
        self.jog_direction=direction;self.speed=speed_rad_s;self.last=time.monotonic()
        self.deadline=self.last+12.;self.moving=True
    def move_by(self,delta_rad,speed_rad_s):
        finite(delta_rad,'delta');self.move_to(self.values[self.selected-1]+delta_rad,speed_rad_s)
    def poll(self):
        if self.closed:raise RuntimeError('Position port is closed')
        now=time.monotonic();dt=now-self.last;self.last=now
        if self.moving and self.jog_direction:
            if now>=self.deadline:
                self.stop();raise TimeoutError('Manual jog host duration limit reached (12.000 s)')
            target=self.values[self.selected-1]+self.jog_direction*self.speed*dt
            if not -12.5<target<12.5:
                self.stop();raise RuntimeError('Manual target reached protocol position range')
            self.values[self.selected-1]=target
        elif self.moving:
            i=self.selected-1;delta=self.target-self.values[i];step=self.speed*dt
            self.values[i]+=max(-step,min(step,delta))
            if abs(delta)<=step:self.moving=False
        return self.moving
    def positions(self):
        limit=12.5 if self.window is None else self.window
        return [JointPosition(j,self.values[j-1],0.,-limit,limit,0,self.moving and j==self.selected)
                for j in self.available_joints]
    def cached_positions(self):return self.positions()
    def stationary_position(self):
        if self.closed or self.moving:
            raise RuntimeError('Release motion before sampling')
        return next(row for row in self.positions() if row.joint == self.selected)
    def release(self):self.moving=False
    def stop(self):self.armed=False;self.moving=False
    def recover(self,report_path=None):
        if self.closed:raise RuntimeError('Position port is closed')
        if self.moving:raise RuntimeError('Release motion before recovery')
        self.stop();self.report.pop('error',None)
        self.report['events'].append(dict(event='recovered',armed=False))
    def close(self):
        self.stop();self.closed=True
        self.report.update(final_positions_rad=list(self.values),selected_joint=self.selected)
