"""One arm transport owner, isolated from radio ingestion and chassis output."""
import threading
import time
from motors.arm_position import MotionCancelled
from .pocket_input import ArmIntent
from .pocket_display import position_snapshot


class IntentMailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.intent = ArmIntent(0, None, 0, 0.)

    def put(self, intent):
        with self.lock:
            self.intent = intent

    def get(self):
        with self.lock:
            return self.intent


class ArmJogRunner:
    """Single-owner tick; even a slow enable must recheck the latest input."""
    def __init__(self, port, mailbox, mapping, *, clock=time.monotonic):
        self.port, self.mailbox, self.mapping, self.clock = port, mailbox, mapping, clock
        self.active = None
        self.finished = None
        # ArmPositionPort invokes this inside enable and before every target.
        self.port.motion_allowed = self.allowed

    @staticmethod
    def token(intent):
        return (intent.generation, intent.joint, intent.direction)

    def allowed(self):
        intent = self.mailbox.get()
        return (self.active == self.token(intent) and intent.direction != 0
                and intent.joint is not None and self.clock() < intent.deadline)

    def tick(self):
        intent = self.mailbox.get()
        token = self.token(intent)
        if not intent.direction or intent.joint is None or self.clock() >= intent.deadline:
            if self.active is not None or self.port.armed:
                self.port.stop()
            self.active = None
            return
        if self.active is not None and token != self.active:
            self.port.stop()
            self.active = None
        if token == self.finished:
            return  # A held stick must not restart after reaching a boundary.
        try:
            if self.active is None:
                self.active = token
                self.port.select(intent.joint)
                self.port.arm()
                row = self.port.positions()[intent.joint-1]
                # Zero is permitted only in offline simulation, visibly uncalibrated.
                sign = (self.mapping.cw_signs[intent.joint-1] or 1)*intent.direction
                target = row.upper_rad if sign > 0 else row.lower_rad
                if not self.allowed():
                    raise MotionCancelled('Input changed during arm preparation')
                self.port.move_to(target, self.mapping.speed)
            if not self.allowed():
                raise MotionCancelled('Input expired before poll')
            if not self.port.poll():
                self.finished = token
                self.active = None
        except MotionCancelled:
            self.port.stop()
            self.finished = token
            self.active = None


class ArmWorker:
    def __init__(self, factory, mapping):
        self.mailbox = IntentMailbox()
        self.ready = threading.Event()
        self.shutdown = threading.Event()
        self.error = None
        self.report = {}
        self.display_lock = threading.Lock()
        self.display = {}
        self.factory, self.mapping = factory, mapping
        self.thread = threading.Thread(target=self.run, name='pocket-arm-owner', daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        port = None
        try:
            port = self.factory()
            runner = ArmJogRunner(port, self.mailbox, self.mapping)
            self.ready.set()
            while not self.shutdown.is_set():
                runner.tick()
                snapshot = position_snapshot(port, time.monotonic())
                with self.display_lock:
                    self.display = snapshot
                self.shutdown.wait(.02)
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.ready.clear()
            if port is not None:
                try:
                    port.close()
                except Exception as exc:
                    self.error = f'{self.error or ""} Cleanup failed: {exc}'
                self.report = port.report

    def display_snapshot(self, now):
        with self.display_lock:
            snapshot = dict(self.display)
        at = snapshot.pop('at', None)
        snapshot.update(ready=self.ready.is_set(), error=self.error,
                        sample_age_ms=max(0.,(now-at)*1000) if at is not None else None)
        return snapshot

    def close(self):
        self.mailbox.put(ArmIntent(0, None, 0, 0.))
        self.shutdown.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=8)
            if self.thread.is_alive():
                raise TimeoutError('Arm owner did not exit; motor command lease must expire')
        if self.error:
            raise RuntimeError(self.error)
