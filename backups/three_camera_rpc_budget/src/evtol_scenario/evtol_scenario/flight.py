"""Measured actor placement and waypoint arrival; no ROS or ego commands."""
import math
import time
import numpy as np


def vector(value):
    return np.array([value[k + '_val'] for k in ('x', 'y', 'z')], dtype=float)


def xyz(value):
    return dict(zip(('x_val', 'y_val', 'z_val'), map(float, value)))


class PositionFlight:
    def __init__(self, rpc, name, stop, tolerance=.5, clock=time.monotonic):
        self.rpc, self.name, self.stop = rpc, name, stop
        self.tolerance, self.clock = float(tolerance), clock
        self.last_command=np.zeros(3);self.last_step=None

    def state(self):
        kin = self.rpc.call('getMultirotorState', self.name)['kinematics_estimated']
        position, velocity = vector(kin['position']), vector(kin['linear_velocity'])
        if not np.isfinite([position, velocity]).all():
            raise RuntimeError(self.name + ': non-finite flight state')
        return position, velocity

    def step(self, target, speed):
        position, velocity = self.state()
        error = np.asarray(target, dtype=float) - position
        command = .9 * error - .5 * velocity
        norm = float(np.linalg.norm(command))
        if norm > speed:
            command *= speed / norm
        now=self.clock()
        dt=.1 if self.last_step is None else min(.25,max(.01,now-self.last_step))
        change=command-self.last_command;length=float(np.linalg.norm(change))
        if length>3.*dt:command=self.last_command+change*(3.*dt/length)
        self.last_step=now;self.last_command=command.copy()
        if not self.stop.is_set():
            # The RPC blocks for its duration. With 32 actors, 0.1 s blocking
            # calls plus long moveToPosition tasks starved the ego gateway.
            self.rpc.call('moveByVelocity', *map(float, command), .02, 0,
                          dict(is_rate=True, yaw_or_rate=0.), self.name)
        return float(np.linalg.norm(error)), float(np.linalg.norm(velocity))

    def move(self, target, speed, timeout):
        """Bounded velocity feedback; no long-running path RPC or short-path no-op."""
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all() or not math.isfinite(speed) or speed <= 0:
            raise ValueError('Invalid actor waypoint or speed')
        position, _ = self.state()
        distance = float(np.linalg.norm(target - position))
        deadline = self.clock() + max(float(timeout), distance / speed * 1.7 + 8.)
        settled = None
        error = float('inf')
        while not self.stop.is_set():
            error, velocity = self.step(target, float(speed))
            now = self.clock()
            if error <= self.tolerance and velocity <= .4:
                if settled is None:
                    settled = now
                if now - settled >= .25:
                    return self.state()[0], error
            else:
                settled = None
            if now >= deadline:
                collision = self.rpc.call('simGetCollisionInfo', self.name)
                raise RuntimeError(f'{self.name}: waypoint timeout, error={error:.3f} m, '
                                   f'speed={velocity:.3f} m/s, collision={collision.get("object_name", "")}')
            self.stop.wait(.08)
        return None

    def place(self, target):
        """One pre-experiment spawn operation; never used between flight waypoints."""
        zero = xyz([0., 0., 0.])
        self.rpc.call('cancelLastTask', self.name)
        self.rpc.call('simSetKinematics', dict(position=xyz(target),
            orientation=dict(x_val=0., y_val=0., z_val=0., w_val=1.),
            linear_velocity=zero, angular_velocity=zero,
            linear_acceleration=zero, angular_acceleration=zero), True, self.name)
        if self.stop.wait(.05):
            return False
        if not self.rpc.call('armDisarm', True, self.name):
            raise RuntimeError(self.name + ': arming failed')
        return self.move(target, 1., 15.) is not None
