from models import *
import graphutils

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec

DURATION = 20
STEP = 0.001
CYCLES = int(DURATION / STEP) + 1

def main():

    f = Flywheel(0.01, 0, kin_fric=0.01)
    m = Motor(f, 11, .65)
    pid = PID(0.3, 0.00, 0.2)
    
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
        m.power = pid.push_error(target - f.pos, STEP)
        #m.power = 1
        torques.append(m.torque)
        m.step(STEP)
        _impulses.append(f._torques)
        f.step(STEP)
        positions.append(f.pos)
        velocities.append(f.vel)
        powers.append(m.power)
        targets.append(target)

    title = f'NeveRest 60 Position PID Test (P={pid.p}, I={pid.i}, D={pid.d})'
    graphutils.graph_pos_target(title, frames, positions, velocities, targets, powers, torques)
    
if __name__ == '__main__':
    main()
