"""Process-scoped Windows timer resolution for 100 Hz acquisition."""
import ctypes
import os
import math
import hashlib
import threading
import time
from collections import deque


class AcquisitionWait:
    """High-resolution bounded sleeps; Event.wait rounds short Windows waits up."""
    def __init__(self):
        self.handle = None
        if os.name == 'nt':
            self.api = ctypes.WinDLL('kernel32', use_last_error=True)
            self.api.CreateWaitableTimerExW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_ulong, ctypes.c_ulong]
            self.api.CreateWaitableTimerExW.restype = ctypes.c_void_p
            self.api.SetWaitableTimer.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_longlong), ctypes.c_long, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_bool]
            self.api.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            self.api.CloseHandle.argtypes = [ctypes.c_void_p]
            self.handle = self.api.CreateWaitableTimerExW(None, None, 2, 0x1F0003)

    def wait(self, seconds, stop):
        if stop.is_set() or seconds <= 0:
            return
        if self.handle:
            due = ctypes.c_longlong(-max(1, round(seconds * 10_000_000)))
            if self.api.SetWaitableTimer(self.handle, ctypes.byref(due), 0, None, None, False):
                self.api.WaitForSingleObject(self.handle, max(1, int(seconds * 1000) + 100))
                return
        stop.wait(seconds)

    def close(self):
        if self.handle:
            self.api.CloseHandle(self.handle)
            self.handle = None


class BufferedImu:
    """Decouple latest-only RPC polling from publication using real sample history.

    The consumer drains to a delayed reception-time watermark. Intermediate
    native samples are decimated; consumed samples are never reused or stamped.
    This is a bounded in-process queue, not a UE shared-memory transport.
    """
    def __init__(self, query, latency=.05, poll_interval=.00125, close_timeout=5.):
        if not 0. <= latency <= .5 or poll_interval <= 0.:
            raise ValueError('Invalid IMU buffering/poll interval')
        self.query, self.latency, self.poll_interval = query, latency, poll_interval
        self.close_timeout = close_timeout
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.samples = deque(maxlen=1024)
        self.last_ns = 0
        self.error = None
        self.overruns = 0
        self.thread = threading.Thread(target=self.run, name='imu_rpc', daemon=True)
        self.thread.start()

    def store(self, received, ns, data):
        with self.lock:
            if ns <= 0 or ns < self.last_ns:
                raise ValueError('IMU acquisition clock invalid/reset; restart acquisition')
            self.error = None
            if ns == self.last_ns:
                return
            self.last_ns = ns
            if len(self.samples) == self.samples.maxlen:
                self.overruns += 1
            self.samples.append((received, ns, data))

    def read(self, now=None):
        cutoff = (time.perf_counter() if now is None else now)-self.latency
        with self.lock:
            if self.error is not None:
                raise RuntimeError('IMU RPC sampler failed: '+str(self.error))
            selected = None
            while self.samples and self.samples[0][0] <= cutoff:
                _, ns, data = self.samples.popleft()
                selected = ns, data
            return selected

    def run(self):
        waiter = AcquisitionWait()
        try:
            while not self.stop.is_set():
                begun = time.perf_counter()
                try:
                    ns, data = self.query()
                    self.store(time.perf_counter(), ns, data)
                    delay = max(0., self.poll_interval-(time.perf_counter()-begun))
                except Exception as exc:
                    with self.lock:
                        self.error = exc
                    delay = .05
                waiter.wait(delay, self.stop)
        finally:
            waiter.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=self.close_timeout)
        if self.thread.is_alive():
            raise RuntimeError('IMU RPC sampler did not stop before its configured timeout')


class SensorLease:
    """One writer per AirSim endpoint/vehicle/kind across all ROS domains."""
    def __init__(self, config, kind):
        self.handle = None
        if os.name != 'nt' or config['backend'] != 'airsim':
            return
        endpoint = config['airsim']
        key = f"{endpoint['host']}:{endpoint['port']}:{endpoint['vehicle']}:{kind}"
        name = 'Local\\EVTOL_sensor_' + hashlib.sha256(key.encode()).hexdigest()[:24]
        self.api = ctypes.WinDLL('kernel32', use_last_error=True)
        self.api.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
        self.api.CreateMutexW.restype = ctypes.c_void_p
        self.api.CloseHandle.argtypes = [ctypes.c_void_p]
        self.handle = self.api.CreateMutexW(None, False, name)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        if ctypes.get_last_error() == 183:
            self.close()
            raise RuntimeError('Another AirSim '+kind+' publisher is already running for '+key)

    def close(self):
        if self.handle:
            self.api.CloseHandle(self.handle)
            self.handle = None


class AsyncRaycasts:
    """Refresh an entire LOS snapshot off the publication thread; never invent LOS."""
    def __init__(self, query, stations, interval, max_age):
        self.query, self.stations = query, [tuple(p) for p in stations]
        self.interval, self.max_age = interval, max_age
        self.lock = threading.Lock()
        self.wake, self.stop = threading.Event(), threading.Event()
        self.position = None
        self.snapshot = {}
        self.updated = 0.
        self.last_error = ''
        self.started = time.perf_counter()
        self.thread = threading.Thread(target=self.run, name='airsim_los', daemon=True)
        self.thread.start()

    def update(self, position):
        with self.lock:
            self.position = tuple(position)
        self.wake.set()

    def ready(self):
        with self.lock:
            age = time.perf_counter()-self.updated if self.updated else time.perf_counter()-self.started
            ready = bool(self.snapshot) and age <= self.max_age
            error = self.last_error
        if not ready and (error or age > self.max_age):
            raise RuntimeError('LOS snapshot unavailable/stale: '+(error or f'age={age:.3f}s'))
        return ready

    def lookup(self, position):
        if not self.ready():
            raise RuntimeError('LOS snapshot not ready')
        with self.lock:
            return self.snapshot[tuple(position)]

    def run(self):
        while not self.stop.is_set():
            self.wake.wait(.1)
            with self.lock:
                position = self.position
            if position is None:
                continue
            # Timestamp the query start, so a slow query cannot masquerade as fresh.
            begun = time.perf_counter()
            try:
                result = {p: bool(self.query(position, p)) for p in self.stations}
                with self.lock:
                    self.snapshot, self.updated, self.last_error = result, begun, ''
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
            self.wake.clear()
            self.stop.wait(max(0., self.interval-(time.perf_counter()-begun)))

    def close(self):
        self.stop.set()
        self.wake.set()
        self.thread.join(timeout=5.)


class RateGate:
    """Wall-clock schedule that advances only after a fresh sample is emitted.

    Faster polling can retry an old RPC sample within the same publication slot.
    Missed slots are skipped, never filled by re-stamping previous measurements.
    """
    def __init__(self, rate, recover_phase=False):
        self.period = 1. / rate
        self.due = None
        self.phase = None
        self.recover_phase = recover_phase
        self.dropped_slots = 0

    def ready(self, now):
        return self.due is None or now >= self.due

    def accepted(self, now):
        if self.recover_phase:
            if self.phase is None:
                self.phase = now + self.period
            else:
                self.phase += self.period
                # Recover ordinary scheduler jitter using fresh buffered samples,
                # bounded to 2x the configured instantaneous publication rate.
                # A long stall discards expired slots; never replay stale history.
                if now-self.phase > 3*self.period:
                    skipped = math.floor((now-self.phase)/self.period)+1
                    self.dropped_slots += skipped
                    self.phase += skipped*self.period
            self.due = max(self.phase, now + self.period*.5)
            return
        if self.due is None:
            self.due = now + self.period
        else:
            self.due += max(1, math.floor((now-self.due)/self.period)+1) * self.period


class TimerResolution:
    def __init__(self, high_rate=False):
        self.active = False
        if os.name == 'nt':
            # Windows 11 may ignore timer resolution for an occluded process.
            # Explicitly opt this acquisition process out of that policy.
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.GetCurrentProcess.restype = ctypes.c_void_p
            process = kernel.GetCurrentProcess()
            class PowerState(ctypes.Structure):
                _fields_ = [('Version',ctypes.c_ulong),('ControlMask',ctypes.c_ulong),('StateMask',ctypes.c_ulong)]
            # Disable execution-speed throttling as well as timer-resolution
            # throttling for hidden Windows acquisition processes.
            state = PowerState(1, 1 | 4, 0)
            kernel.SetProcessInformation.argtypes = [ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_uint]
            kernel.SetProcessInformation(process, 4, ctypes.byref(state), ctypes.sizeof(state))
            if high_rate:
                # Above-normal is deliberately below high/realtime priority.
                kernel.SetPriorityClass.argtypes = [ctypes.c_void_p,ctypes.c_uint]
                if not kernel.SetPriorityClass(process, 0x8000):
                    raise ctypes.WinError(ctypes.get_last_error())
            self.api = ctypes.WinDLL('winmm')
            if self.api.timeBeginPeriod(1) != 0:
                raise RuntimeError('Cannot enable Windows 1 ms scheduling resolution')
            self.active = True

    def close(self):
        if self.active:
            self.api.timeEndPeriod(1)
            self.active = False
