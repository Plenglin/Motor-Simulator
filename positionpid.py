from models import *

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec

DURATION = 20
STEP = 0.001
CYCLES = int(DURATION / STEP) + 1

def main():

    f = Flywheel(0.01, 0, kin_fric=0.01)
    m = Motor(f, 11, .65)
    p = PID(0.3, 0.00, 0.2)
    
    frames = [t * STEP for t in range(0, CYCLES)]
    masses = [m / 50 for m in range(1, 5 * 50)]
    positions = []
    velocities = []
    targets = []
    powers = []
    torques = []

    _impulses = []

    print(CYCLES)
    
    for t in frames:
        if t < 3:
            target = 0
        elif t < 10:
            target = 6
        elif t < 15:
            target = -4
        else:
            target = 0
        m.power = p.push_error(target - f.pos, STEP)
        #m.power = 1
        torques.append(m.torque)
        m.step(STEP)
        _impulses.append(f._torques)
        f.step(STEP)
        positions.append(f.pos)
        velocities.append(f.vel)
        powers.append(m.power)
        targets.append(target)
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axs = plt.subplot(gs[0])
    plt.title('PID Test')
    
    axm = plt.subplot(gs[1])
    plt.title('Motor')

    axs.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    lines, linet = axs.plot(frames, positions, 'r', frames, targets, 'b--')
    axs.set_ylabel('position (rad)')
    axs.set_xlabel('time (s)')

    axv = axs.twinx()
    linev, = axv.plot(frames, velocities, 'g-.')
    axv.set_ylabel('velocity (rad/s)')

    axs.legend((lines, linet, linev), ('position', 'target (rad)', 'velocity'), loc='lower right')

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
