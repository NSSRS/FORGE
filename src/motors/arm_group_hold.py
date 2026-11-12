"""Explicit group position holding with one selected manual jog target.

One owner refreshes every active joint. Activation captures fresh stationary
positions; selection never disables a held joint. This is powered mode-2 hold,
not a brake: stop, faults, lease expiry and shutdown still disable the group.
"""
import math

from . import arm_position as position


class ArmGroupHoldPort(position.ArmPositionPort):
    def __init__(self, plan, **kwargs):
        self.hold_targets={}
        super().__init__(plan,position_hold=True,group_hold=True,**kwargs)

    def _refresh_targets(self):
        for i,target in self.hold_targets.items():
            self.bus.command(i,target,velocity=self.speed,
                             smooth=self.smooth_jog and self.moving and i==self.selected-1)

    def hold_all(self, speed_rad_s=math.radians(position.HOLD_JOG_DEG_S)):
        """Explicit H activation; never runs automatically on launch/recovery."""
        self._check()
        if self.recovery_required:raise RuntimeError('Recovery required before holding')
        if self.hold_targets:
            self.pause();return  # Do not recapture passive drift as new targets.
        if not math.isfinite(speed_rad_s) or not .001<=speed_rad_s<=math.radians(position.HOLD_JOG_DEG_S):
            raise ValueError('Group hold speed must be >0 and <=10 degrees/s')
        if self.smooth_jog:self.bus.check_smooth_jog_speed(speed_rad_s)
        with self._guard_motion():
            self._check_intent()
            states=self._stop_session()
            if any(s.fault or s.enabled or s.age_ms>=80 or abs(s.velocity)>.03 for s in states.values()):
                raise RuntimeError('Group hold requires fresh stationary disabled feedback for every joint')
            self.speed=speed_rad_s;self.lead_limited=False;self.previous_sample_at=None
            self.hold_targets={i:states[i].position for i in self.slots}
            self.target=self.hold_targets[self.selected-1]
            self._refresh_targets()
            self._check_intent()
            self.bus.enable_bench_group(self.slots)
            deadline=position.time.monotonic()+.4
            while True:
                self._check_intent()
                self._refresh_targets()
                states={i:self.session.sample(i) for i in self.slots}
                if all(s.enable_confirmed and s.status==1 for s in states.values()):break
                if position.time.monotonic()>=deadline:raise TimeoutError('Group hold enable not confirmed')
            self._check_intent()
            self.holding=True;self.armed=False;self.moving=False;self.jog_direction=0
            for i,s in states.items():
                self._remember_sample(i,s)
                self._record_motion_sample(i,s,hold_target=self.hold_targets[i])
            self._event('hold_all',joints=list(self.available_joints),targets_rad=dict(self.hold_targets))

    def arm(self):
        if not self.hold_targets:raise RuntimeError('Press H to hold all joints before arming a jog')
        # The control owner checks the complete enabled group before authorizing.
        self.poll()
        self.armed=True;self._event('armed',joint=self.selected)

    def select(self,joint):
        self._check()
        if type(joint) is not int or joint not in self.available_joints:
            raise ValueError('Select a connected joint')
        if self.hold_targets:
            self.pause()
            self.selected=joint;self.target=self.hold_targets[joint-1]
        else:
            super().select(joint);return
        self._event('select',joint=joint,group_holding=True)

    def _start_move(self,angle_rad,speed_rad_s,direction):
        if not self.hold_targets:raise RuntimeError('Press H to hold all joints first')
        with self._guard_motion():
            self.poll()
            super()._start_move(angle_rad,speed_rad_s,direction)
            self.hold_targets[self.selected-1]=self.target
            self._refresh_targets()

    def poll(self):
        self._check()
        if not self.hold_targets:return False
        with self._guard_motion():
            poll_started=position.time.monotonic()
            # Command and feedback requests are bounded by the existing leases.
            # A failed request faults/stops the whole group through this guard.
            states={i:self._enabled_sample(i) for i in self.slots}
            feedback_done=position.time.monotonic()
            if self.moving:
                self._check_intent()
                i=self.selected-1
                self._advance_jog(position.time.monotonic(),states[i])
                lo,hi=self._limits(i)
                if not lo<self.target<hi:raise RuntimeError('Manual target reached protocol position range')
                self.hold_targets[i]=self.target
            commands_started=position.time.monotonic()
            self._refresh_targets()
            poll_done=position.time.monotonic()
            self._control_timing=dict(owner_poll_ms=(poll_done-poll_started)*1000,
                feedback_batch_ms=(feedback_done-poll_started)*1000,
                command_batch_ms=(poll_done-commands_started)*1000)
            for i,s in states.items():
                if self.moving and i==self.selected-1:self._record_motion_sample(i,s)
                else:self._record_motion_sample(i,s,hold_target=self.hold_targets[i])
        return self.moving

    def release(self):
        if not self.hold_targets:return super().release()
        with self._guard_motion():
            if self.moving:
                sample=self._enabled_sample(self.selected-1)
                self.target=sample.position
                self.hold_targets[self.selected-1]=self.target
                self._event('hold',joint=self.selected,target_rad=self.target)
            self.moving=False;self.holding=True;self.jog_direction=0;self.lead_limited=False
            self.poll()  # Refresh every fixed target, including the selected one.

    def stop(self):
        self.hold_targets.clear()
        super().stop()

    def close(self):
        self.hold_targets.clear()
        super().close()


class SimulatedGroupHoldPort(position.SimulatedPositionPort):
    group_hold=True

    def __init__(self,**kwargs):
        self.hold_targets={}
        super().__init__(position_hold=True,**kwargs)
        self.report['group_hold']=True

    def hold_all(self,speed_rad_s=math.radians(position.HOLD_JOG_DEG_S)):
        if self.hold_targets:self.pause();return
        self.hold_targets={j-1:self.values[j-1] for j in self.available_joints}
        self.speed=speed_rad_s;self.target=self.values[self.selected-1]
        self.holding=True;self.armed=False

    def arm(self):
        if not self.hold_targets:raise RuntimeError('Press H to hold all joints first')
        self.armed=True

    def select(self,joint):
        if joint not in self.available_joints:raise ValueError('Select a connected joint')
        if not self.hold_targets:return super().select(joint)
        self.pause();self.selected=joint;self.target=self.hold_targets[joint-1]

    def release(self):
        if self.moving:self.hold_targets[self.selected-1]=self.values[self.selected-1]
        super().release()

    def positions(self):
        return [position.replace(row,enabled=bool(self.hold_targets)) for row in super().positions()]

    def stop(self):
        self.hold_targets.clear();super().stop()

    def close(self):
        self.hold_targets.clear();super().close()
