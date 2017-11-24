from models import *

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec


THRESHOLD = 1

DURATION = 20
STEP = 0.01
CYCLES = int(DURATION / STEP) + 1


def main():

    f = Flywheel(0.01, 0, kin_fric=0.05)
    m = Motor(f, 556, 2.42)
    
    frames = [t * STEP for t in range(0, CYCLES)]
    masses = [m / 50 for m in range(1, 5 * 50)]
    velocities = []
    targets = []
    powers = []
    torques = []

    _impulses = []

    print(CYCLES)
    
    for t in frames:
        if t < 3:
            target = 0
        elif t < 7:
            target = 300
        elif t < 15:
            target = -200
        else:
            target = 0

        if abs(f.vel - target) > THRESHOLD:
            if f.vel < target:
                m.power = 1
            else:
                m.power = -1
        else:
            m.power = 0
        torques.append(m.torque)
        m.step(STEP)
        _impulses.append(f._torques)
        f.step(STEP)
        velocities.append(f.vel)
        powers.append(m.power)
        targets.append(target)
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axv = plt.subplot(gs[0])
    plt.title(f'AM-0255 CIM Velocity Bang-Bang Test')
    
    axm = plt.subplot(gs[1], sharex=axv)
    plt.title('Motor')

    axv.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    linev, linet = axv.plot(frames, velocities, 'r', frames, targets, 'b--')
    axv.set_xlabel('time (s)')
    axv.set_ylabel('velocity (rad/s)')

    axv.legend((linet, linev), ('target (rad)', 'velocity'), loc='lower right')

    linem, = axm.plot(frames, powers, 'b')
    axm.set_ylim([-1.1, 1.1])
    axm.set_xlabel('time (s)')
    axm.set_ylabel('motor power')
    axm.set_yticks(np.arange(-1, 1.1, 0.5))

    axt = axm.twinx()
    axt.set_ylabel('torque (N*m)')
    linetor, = axt.plot(frames, torques, 'g')

    axm.legend((linem, linetor), ('power', 'torque'), loc='lower right')

    plt.show()
    
if __name__ == '__main__':
    main()
