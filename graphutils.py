import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec

import numpy as np
import matplotlib.pyplot as plt


def align_yaxis(ax1, v1, ax2, v2):
    """adjust ax2 ylimit so that v2 in ax2 is aligned to v1 in ax1"""
    _, y1 = ax1.transData.transform((0, v1))
    _, y2 = ax2.transData.transform((0, v2))
    inv = ax2.transData.inverted()
    _, dy = inv.transform((0, 0)) - inv.transform((0, y1-y2))
    miny, maxy = ax2.get_ylim()
    ax2.set_ylim(miny+dy, maxy+dy)


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


def graph_vel(title, motor, frames):
    gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1])

    f = motor.flywheel
    velocities = list(f.velocities)
    powers = list(motor.powers)
    torques = list(motor.torques)

    plt.subplots_adjust(hspace=0.4)
    axv = plt.subplot(gs[0])
    plt.title(title)

    axm = plt.subplot(gs[1], sharex=axv)
    plt.title('Motor')

    axv.grid(color='0.75', linewidth=1)
    axm.grid(color='0.75', linewidth=1)
    linev = axv.plot(frames, velocities, 'r')
    axv.set_xlabel('time (s)')
    axv.set_ylabel('velocity (rad/s)')

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

    align_yaxis(axs, 0, axv, 0)

    plt.show()


def graph_ff_target(title, motor, frames, targets, derivatives):

    f = motor.flywheel
    positions = list(f.positions)
    velocities = list(f.velocities)
    powers = list(motor.powers)
    torques = list(motor.torques)

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

    align_yaxis(axs, 0, axv, 0)

    plt.show()
