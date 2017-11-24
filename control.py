from abc import ABC, abstractmethod

from models import *


class Simulation(ABC):

    def __init__(self, duration, step=0.001, control_frequency=50):
        self.step = step
        self.duration = duration
        self.control_frequency = control_frequency
        
        self.cycles = int(duration / step) + 1
        self.steps_per_control = int(1 / control_frequency / step)

        self._frames = None
        self.motors = []
        self.flywheels = []

    @property
    def frames(self):
        if self._frames is None:
            self._frames = [t * self.step for t in range(0, self.cycles)]
        return self._frames

    @property
    def control_frames(self):
        if self._frames is None:
            self._frames = [t * self.step * self.steps_per_control for t in range(0, self.cycles)]
        return self._frames

    @abstractmethod
    def init(self):
        pass
    
    @abstractmethod
    def loop(self, i, t, dt):
        pass

    def raw_loop(self, i, t, dt):
        pass

    def add_motor(self, motor):
        assert isinstance(motor, Motor), 'not a motor'
        self.motors.append(motor)
        return motor

    def add_flywheel(self, flywheel):
        assert isinstance(flywheel, Flywheel), 'not a flywheel'
        self.flywheels.append(flywheel)
        return flywheel

    def simulate(self):
        self.flywheels = []
        self.motors = []
        self.init()
        for i, t in enumerate(self.frames):
            self.raw_loop(i, t, self.step)
            if i % self.steps_per_control == 0:
                self.loop(i / self.steps_per_control, t, self.step * self.steps_per_control)
            for f in self.flywheels:
                f.step(self.step)
            for m in self.motors:
                m.step(self.step)


class TargetedSimulation(Simulation, ABC):

    def __init__(self, duration, step=0.001, control_frequency=100):
        super().__init__(duration, step, control_frequency)
        self.target = 0
        self.targets = []
        self.derivative = 0
        self.derivatives = []

    @abstractmethod
    def get_target(self, i, t, dt):
        pass

    def get_derivative(self, i, t, dt):
        return 0

    def raw_loop(self, i, t, dt):
        self.target = self.get_target(i, t, dt)
        self.targets.append(self.target)
        self.derivative = self.get_derivative(i, t, dt)
        self.derivatives.append(self.derivative)


def main():
    
    class _ExampleSimulation(Simulation):

        def __init__(self):
            super().__init__(0.001, 20, 60)
            self.flywheel = Flywheel(0.01, kin_fric=0.01)
            self.motor = Motor(self.flywheel, 11, .65)
            
        def init(self):
            self.add_flywheel(self.flywheel)
            self.add_motor(self.motor)
            self.motor.power = 10

        def loop(self, i, time, dt):
            pass

    sim = _ExampleSimulation()
    sim.simulate()
    print(sim.flywheel.vel)

if __name__ == '__main__':
    main()
