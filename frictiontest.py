import control
import graphutils
from models import *


STATIC_FRIC = 0.2
KINETIC_FRIC = 0.1


class FrictionTest(control.Simulation):

    def __init__(self):
        super().__init__(20)
        self.flywheel = Flywheel(0.01, kin_fric=KINETIC_FRIC, stat_fric=STATIC_FRIC)
        self.motor = Motor(self.flywheel, 11, .65)

    def init(self):
        self.add_flywheel(self.flywheel)
        self.add_motor(self.motor)

    def loop(self, i, t, dt):
        self.motor.power = t / 10


if __name__ == '__main__':
    sim = FrictionTest()
    sim.simulate()
    graphutils.graph_vel(f"Friction Test (fs = {STATIC_FRIC} N*m, fk = {KINETIC_FRIC} N*m)", sim.motor, sim.frames)
