from models import *

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec

DURATION = 20
STEP = 0.001
CYCLES = int(DURATION / STEP) + 1

def main():

    f = Flywheel(0.01, 0, kin_fric=0.05)
    m = Motor(f, 556, 2.42)
    p = PID(1, 0, 0)
    
    frames = [t * STEP for t in range(0, CYCLES)]
    masses = [m / 50 for m in range(1, 5 * 50)]
    velocities = []
    targets = []
    powers = []
    torques = []
    disturbances = []

    target = 100

    print(CYCLES)
    
    for t in frames:
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
        
        f.apply_torque(disturb)
        m.power = p.push_error(target - f.vel, STEP)
        #m.power = 1
        torques.append(m.torque)
        m.step(STEP)
        f.step(STEP)
        velocities.append(f.vel)
        powers.append(m.power)
        targets.append(target)
        disturbances.append(disturb)
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axv = plt.subplot(gs[0])
    plt.title(f'AM-0255 CIM Velocity Disturbance Test (P={p.p}, I={p.i}, D={p.d})')
    
    axm = plt.subplot(gs[1])
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
