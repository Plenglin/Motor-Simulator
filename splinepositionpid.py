import control
import graphutils
import mathutils
from models import *


points = [0, 5, 10, 0, -8, -2, 3]
duration = 10
time_between = duration / (len(points) - 1)


class SplinePositionSimulation(control.TargetedSimulation):

    def __init__(self):
        super().__init__(duration)
        self.flywheel = Flywheel(0.01, kin_fric=0.1)
        self.motor = Motor(self.flywheel, 11, .65)
        self.pid = PID(2, 1.5, 0.9, 0.15)
        self.spline = lambda x: mathutils.Spline(*points)(x / time_between)
        self.deriv = mathutils.derivative(self.spline, h=0.000001)

    def get_target(self, i, t, dt):
        return self.spline(t)

    def get_derivative(self, i, t, dt):
        return self.deriv(t)

    def init(self):
        self.add_flywheel(self.flywheel)
        self.add_motor(self.motor)

    def loop(self, i, time, dt):
        self.motor.power = self.pid.push_error(self.target - self.flywheel.pos, dt, self.derivative)


def main():
    sim = SplinePositionSimulation()
    sim.simulate()

    title = f'NeveRest 60 Spline Motion Processing PID Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d}, F={sim.pid.f})\nTarget function is a spline crossing points {points}'
    graphutils.graph_ff_target(title, sim.motor, sim.frames, sim.targets, sim.derivatives)
    
if __name__ == '__main__':
    main()
