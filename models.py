import math

import numpy as np

EPSILON = 0.001

class Flywheel:

    def __init__(self, mass, stat_fric=0, kin_fric=0):
        self.mass = mass  # kg
        self.pos = 0  # rad
        self.vel = 0  # rad/s
        self.stat_fric = stat_fric  # N
        self.kin_fric = kin_fric  # N
        self._impulses = 0  # n*m/s
        self._torques = 0  # n*m

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
            
    @property
    def is_moving(self):
        return abs(self.vel) > EPSILON

    def apply_impulse(self, impulse):
        self._impulses += impulse

    def apply_torque(self, torque):
        self._torques += torque

    def halt(self):
        self.vel = 0
        self._impulses = 0
        self._torques = 0
        
class PID:

    def __init__(self, p, i, d, f=0):
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

class Motor:

    def __init__(self, flywheel, max_vel, stall_torque, min_power=0):
        self.flywheel = flywheel
        self.max_vel = max_vel
        self.stall_torque = stall_torque
        self._power = 0
        self.min_power = min_power

    @property
    def power(self):
        return self._power
    
    @power.setter
    def power(self, val):
        self._power = max(min(1, val), -1)
        if abs(self._power) < self.min_power:
            self._power = 0

    @property
    def torque(self):
        mag = max(0, abs(self.power) - abs(self.flywheel.vel) / self.max_vel) * self.stall_torque
        return np.sign(self.power) * mag

    def step(self, dt):
        self.flywheel.apply_torque(self.torque)
        
