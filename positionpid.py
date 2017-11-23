import control
import graphutils
from models import *


class PositionSimulation(control.TargetedSimulation):

    def __init__(self):
        super().__init__(20)
        self.flywheel = Flywheel(0.01, 0, kin_fric=0.01)
        self.motor = Motor(self.flywheel, 11, .65)
        self.pid = PID(0.3, 0.00, 0.2)
        
    def init(self):
        self.add_flywheel(self.flywheel)
        self.add_motor(self.motor)

    def get_target(self, i, t, dt):
        if t < 3:
            return 0
        elif t < 10:
            return 6
        elif t < 15:
            return -4
        else:
            return 0

    def loop(self, i, t, dt):
        self.motor.power = self.pid.push_error(self.target - self.flywheel.pos, dt)


def main():
    sim = PositionSimulation()
    sim.simulate()

    title = f'NeveRest 60 Position PID Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d})'
    graphutils.graph_pos_target(title, sim.motor, sim.frames, sim.targets)
    
if __name__ == '__main__':
    main()
