from models import *
import graphutils
import control

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import gridspec


class PositionSimulation(control.Simulation):

    def __init__(self):
        super().__init__(0.001, 20, 60)
        self.flywheel = Flywheel(0.01, 0, kin_fric=0.01)
        self.motor = Motor(self.flywheel, 11, .65)
        self.pid = PID(0.3, 0.00, 0.2)
        self.targets = []
        self.target = 0
        
    def init(self):
        self.add_flywheel(self.flywheel)
        self.add_motor(self.motor)
        self.targets = []

    def raw_loop(self, i, t, dt):
        if t < 3:
            self.target = 0
        elif t < 10:
            self.target = 6
        elif t < 15:
            self.target = -4
        else:
            self.target = 0
        self.targets.append(self.target)

    def loop(self, i, t, dt):
        self.motor.power = self.pid.push_error(self.target - self.flywheel.pos, dt)

def main():

    sim = PositionSimulation()
    sim.simulate()

    title = f'NeveRest 60 Position PID Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d})'
    graphutils.graph_pos_target(title, sim.motor, sim.frames, sim.targets)
    
if __name__ == '__main__':
    main()
