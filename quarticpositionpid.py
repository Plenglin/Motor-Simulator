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
    p = PID(0.6, 1, 0.8, 0.15)
    
    frames = [t * STEP for t in range(0, CYCLES)]
    masses = [m / 50 for m in range(1, 5 * 50)]
    positions = []
    velocities = []
    targets = []
    powers = []
    torques = []
    derivatives = []

    print(CYCLES)
    
    for t in frames:
        target = 0.005 * (t - 7.27) * (t - 15.67) * (t - 18.7) * t
        vel = 0.005 * (4 * t**3 - 124.92 * t**2 + 1085.7978 * t - 2130.32083)  # Derivative of the above
        m.power = p.push_error(target - f.pos, STEP, vel)
        #m.power = 1
        torques.append(m.torque)
        m.step(STEP)
        f.step(STEP)
        
        positions.append(f.pos)
        derivatives.append(vel)
        velocities.append(f.vel)
        powers.append(m.power)
        targets.append(target)
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axs = plt.subplot(gs[0])
    plt.title(f'NeveRest 60 Motion Processing PID Test (P={p.p}, I={p.i}, D={p.d}, F={p.f})\nTarget function is s(t) = 0.005t(t - 7.27)(t - 15.67)(t - 18.7)')

    axm = plt.subplot(gs[1])
    plt.title('Motor')

    axs.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    lines, linet = axs.plot(frames, positions, 'r', frames, targets, 'b--')
    axs.set_ylabel('position (rad)')
    axs.set_xlabel('time (s)')

    axv = axs.twinx()
    linev, lined = axv.plot(frames, velocities, 'g-.', frames, derivatives, 'm:')
    axv.set_ylabel('velocity (rad/s)')

    axs.legend((lines, linet, linev, lined), ('position', 'target (rad)', 'velocity', 'derivative'), loc='lower right')

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
