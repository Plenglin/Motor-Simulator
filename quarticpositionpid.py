import control
import graphutils
from models import *


class QuarticPositionSimulation(control.TargetedSimulation):

    def __init__(self):
        super().__init__(20)
        self.flywheel = Flywheel(0.01, kin_fric=0.01)
        self.motor = Motor(self.flywheel, 11, .65)
        self.pid = PID(0.6, 1, 0.8, 0.15)

    def get_target(self, i, t, dt):
        return 0.005 * (t - 7.27) * (t - 15.67) * (t - 18.7) * t

    def get_derivative(self, i, t, dt):
        return 0.005 * (4 * t**3 - 124.92 * t**2 + 1085.7978 * t - 2130.32083)

    def init(self):
        self.add_flywheel(self.flywheel)
        self.add_motor(self.motor)

    def loop(self, i, time, dt):
        self.motor.power = self.pid.push_error(self.target - self.flywheel.pos, dt, self.derivative)


def main():
    sim = QuarticPositionSimulation()
    sim.simulate()

    title = f'NeveRest 60 Motion Processing PID Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d}, F={sim.pid.f})\nTarget function is s(t) = 0.005t(t - 7.27)(t - 15.67)(t - 18.7)'
    graphutils.graph_ff_target(title, sim.motor, sim.frames, sim.targets, sim.derivatives)
    
if __name__ == '__main__':
    main()
