

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec

def graph_vel_target(title, motor, frames, targets):
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    f = motor.flywheel
    velocities = f.velocities
    powers = motor.powers
    torques = motor.torques

    plt.subplots_adjust(hspace=0.4)
    axv = plt.subplot(gs[0])
    plt.title(title)
    
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

def graph_pos_target(title, motor, frames, targets):
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    f = motor.flywheel
    powers = list(motor.powers)
    torques = list(motor.torques)
    positions = list(f.positions)
    velocities = list(f.velocities)

    plt.subplots_adjust(hspace=0.4)
    axs = plt.subplot(gs[0])
    plt.title(title)

    axm = plt.subplot(gs[1], sharex=axs)
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
    
def graph_ff_target(title, frames, positions, velocities, targets, derivatives, powers, torques):
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    plt.subplots_adjust(hspace=0.4)
    axs = plt.subplot(gs[0])
    plt.title(title)

    axm = plt.subplot(gs[1], sharex=axs)
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
