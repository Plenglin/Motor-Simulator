from collections import namedtuple
import math

import numpy as np

EPSILON = 0.001

FlywheelState = namedtuple('FlywheelState', ('pos', 'vel', 'acc'))
        
class Flywheel:

    def __init__(self, mass, stat_fric=0, kin_fric=0):
        self.mass = mass  # kg
        self.pos = 0  # rad
        self.vel = 0  # rad/s
        self.stat_fric = stat_fric  # N
        self.kin_fric = kin_fric  # N
        self._impulses = 0  # n*m/s
        self._torques = 0  # n*m
        self.history = []

    @property
    def ang_momentum(self):
        return self.vel * mass

    def torque_step(self, dt):
        if self.is_moving:
            friction = -np.sign(self.vel) * self.kin_fric
            self.apply_torque(friction)

        impulse = self._impulses + self._torques * dt

        self._impulses = 0
        self._torques = 0
        return impulse
        
    def step(self, dt):
        impulse = self.torque_step(dt)
        
        acc = impulse / self.mass
        self.vel += acc
        self.pos += self.vel * dt

        if not self.is_moving:
            self.vel = 0

        self.history.append(FlywheelState(self.pos, self.vel, acc))
            
    @property
    def is_moving(self):
        return abs(self.vel) > EPSILON

    @property
    def velocities(self):
        return (s.vel for s in self.history)

    @property
    def positions(self):
        return (s.pos for s in self.history)

    def apply_impulse(self, impulse):
        self._impulses += impulse

    def apply_torque(self, torque):
        self._torques += torque

    def halt(self):
        self.vel = 0
        self._impulses = 0
        self._torques = 0
        
class PID:

    def __init__(self, p, i=0, d=0, f=0):
        self.p = p
        self.i = i
        self.d = d
        self.f = f

        self._sum = 0
        self._last = 0

    def push_error(self, error, dt, feed=0):
        de = (error - self._last) / dt
        self._sum += error * dt
        out = self.p * error + self.i * self._sum + self.d * de + self.f * feed
        self._last = error
        return out

MotorState = namedtuple('MotorState', ('power', 'torque'))

class Motor:

    def __init__(self, flywheel, max_vel, stall_torque):
        self.flywheel = flywheel
        self.max_vel = max_vel
        self.stall_torque = stall_torque
        self._power = 0
        self.history = []

    @property
    def power(self):
        return self._power
    
    @power.setter
    def power(self, val):
        self._power = max(min(1, val), -1)

    @property
    def torque(self):
        mag = max(0, abs(self.power) - abs(self.flywheel.vel) / self.max_vel) * self.stall_torque
        return np.sign(self.power) * mag

    @property
    def powers(self):
        return (s.power for s in self.history)

    @property
    def torques(self):
        return (s.torque for s in self.history)

    def step(self, dt):
        self.history.append(MotorState(self.power, self.torque))
        self.flywheel.apply_torque(self.torque)
        
