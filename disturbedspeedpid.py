from models import *

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec
import graphutils
import control

DURATION = 20
STEP = 0.001
CYCLES = int(DURATION / STEP) + 1


class DisturbedSpeedSimulation(control.TargetedSimulation):

    def __init__(self):
        super().__init__(20, control_frequency=100)
        self.flywheel = Flywheel(0.01, kin_fric=0.01)
        self.motor = Motor(self.flywheel, 556, 2.42)
        self.pid = PID(1)
        self.disturbances = []

    def get_target(self, i, t, dt):
        return 100

    def init(self):
        self.add_motor(self.motor)
        self.add_flywheel(self.flywheel)

    def loop(self, i, t, dt):
        self.motor.power = self.pid.push_error(self.target - self.flywheel.vel, STEP)

    def raw_loop(self, i, t, dt):
        super().raw_loop(i, t, dt)
        if t < 2:
            disturb = 0
        elif t < 5:
            disturb = 0.5
        elif t < 8:
            disturb = -1.5
        elif t < 14:
            disturb = -2.1
        elif t < 15:
            disturb = 0
        elif t < 15.5:
            disturb = -4
        else:
            disturb = 0
        self.flywheel.apply_torque(disturb)
        self.disturbances.append(disturb)


def main():

    sim = DisturbedSpeedSimulation()
    sim.simulate()

    frames = sim.frames
    velocities = list(sim.flywheel.velocities)
    targets = sim.targets
    powers = list(sim.motor.powers)
    torques = list(sim.motor.torques)
    disturbances = sim.disturbances

    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axv = plt.subplot(gs[0])
    plt.title(f'AM-0255 CIM Velocity Disturbance Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d})')
    
    axm = plt.subplot(gs[1], sharex=axv)
    plt.title('Motor')

    axv.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    linev, linet = axv.plot(frames, velocities, 'r', frames, targets, 'b--')
    axv.set_xlabel('time (s)')
    axv.set_ylabel('velocity (rad/s)')

    axk = axv.twinx()
    axk.set_xlabel('torque (N*m)')

    axv.legend((linet, linev), ('target (rad)', 'velocity'), loc='lower right')

    linem, = axm.plot(frames, powers, 'b')
    axm.set_ylim([-1.1, 1.1])
    axm.set_xlabel('time (s)')
    axm.set_ylabel('motor power')
    axm.set_yticks(np.arange(-1, 1.1, 0.5))

    axt = axm.twinx()
    axt.set_ylabel('torque (N*m)')
    linetor, lined = axt.plot(frames, torques, 'g', frames, disturbances, 'm:')

    axm.legend((linem, linetor, lined), ('power', 'torque', 'disturbance'), loc='lower right')

    plt.show()
    
if __name__ == '__main__':
    main()
