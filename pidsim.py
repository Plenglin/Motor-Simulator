import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec


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

    def torque_step(self, dt, friction=True):
        impulses = self._impulses + self._torques * dt

        if friction:
            if self.is_moving:
                dec = self.kin_fric * dt
                if impulses > 0:
                    impulses -= dec
                else:
                    impulses += dec
            else:
                dec = min(self.stat_fric * dt, impulses)
                if impulses > 0:
                    impulses -= dec
                else:
                    impulses += dec
                    
        self._impulses = 0
        self._torques = 0
        return impulses
        
    def step(self, dt, impulses=None, friction=True):
        impulses = self.torque_step(dt, friction) if impulses is None else impulses
        
        acc = impulses / self.mass
        self.vel += acc * dt
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

class PID:

    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

        self._sum = 0
        self._last = 0

    def push_error(self, e, dt):
        de = (e - self._last) * dt
        self._sum += e * dt
        out = self.p * e + self.i * self._sum + self.d * de
        self._last = e
        return out

class Motor:

    def __init__(self, flywheel, max_torque, max_speed=math.inf):
        self.flywheel = flywheel
        self.max_torque = max_torque  # N*m
        self.max_speed = max_speed  # rad/s
        self._power = 0

    @property
    def power(self):
        return self._power
    
    @power.setter
    def power(self, power):
        self._power = max(min(1, power), -1)

    @property
    def torque(self):
        return self.power * self.max_torque

    def step(self, dt):
        impulses = self.flywheel.torque_step(dt)
        if abs(self.flywheel.vel) < self.max_speed:
            impulses += self.torque * dt
        self.flywheel.step(dt, impulses=impulses)

DURATION = 100
STEP = 0.01
CYCLES = int(DURATION / STEP)

def main():
    
    f = Flywheel(.05, .1, .05)
    m = Motor(f, 3, 13)
    p = PID(1, 0, 0)
    
    frames = [t * STEP for t in range(0, CYCLES)]
    positions = []
    velocities = []
    targets = []
    powers = []

    target = 10
    
    for t in frames:
        m.power = p.push_error(target - f.vel, STEP)
        m.step(STEP)

        powers.append(m.power)
        targets.append(target)
        positions.append(f.pos)
        velocities.append(f.vel)

    #print(positions)

    plt.title('PID Simulation')

    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    axs = plt.subplot(gs[0])
    axm = plt.subplot(gs[1])

    axs.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    lines, linet = axs.plot(frames, positions, 'r', frames, targets, 'b--')
    axs.set_ylabel('pos (rad)')

    axv = axs.twinx()
    linev, = axv.plot(frames, velocities, 'g-.')
    axv.set_ylabel('vel (rad/s)')

    axs.legend((lines, linet, linev), ('pos', 'targ', 'vel'), loc='lower left')

    plt.setp(axs.get_xticklabels(), visible=False)

    axm.plot(frames, powers)
    axm.set_ylim([-1.1, 1.1])
    axm.set_xlabel('time (s)')
    axm.set_ylabel('motor power')
    axm.set_xticks(np.arange(min(frames), max(frames)+1, 10))
    axm.set_yticks(np.arange(-1, 1.1, 0.5))

    plt.subplots_adjust(hspace=.0)
    plt.show()


if __name__ == '__main__':
    main()
