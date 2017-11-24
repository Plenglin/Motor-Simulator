import control
import graphutils
import mathutils
from models import *


points = [0, 5, -2]
duration = 60
time_between = duration / (len(points) - 1)


class SplinePositionSimulation(control.TargetedSimulation):

    def __init__(self):
        super().__init__(duration, control_frequency=100)
        self.flywheel = Flywheel(0.5, kin_fric=0.1, stat_fric=0.2)
        self.motor = Motor(self.flywheel, 11, .65, deadzone=0)
        self.pid = PID(5, 1.2, 3, 1)
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
        self.motor.set_power_adj(self.pid.push_error(self.target - self.flywheel.pos, dt, self.derivative))


def main():
    sim = SplinePositionSimulation()
    sim.simulate()
    error = max(*((f, t - p) for f, t, p in zip(sim.frames, sim.targets, sim.flywheel.positions)), key=lambda x: x[1])[0]

    print(f'Cycles: {sim.cycles}')
    print(f'Largest error: {error}')

    title = f'''NeveRest 60 Spline Motion Processing PID Test (P={sim.pid.p}, I={sim.pid.i}, D={sim.pid.d}, F={sim.pid.f})
Target function is a spline crossing points {points}
{sim.flywheel.mass} kg flywheel (fs = {sim.flywheel.stat_fric} N*m, fk = {sim.flywheel.kin_fric} N*m)'''
    graphutils.graph_ff_target(title, sim.motor, sim.frames, sim.targets, sim.derivatives)
    
if __name__ == '__main__':
    main()
