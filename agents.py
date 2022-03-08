import numpy as np
from constants import *
from env import Env


class Agent:
    def __init__(self,
                 pos: np.ndarray = np.array([0., 0.]),
                 vel: np.ndarray = np.array([0., 0.]),
                 acc: np.ndarray = np.array([0., 0.]),
                 env: Env = None) -> None:

        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.env = env

    def display(self,) -> None:
        """        
        Pygame operations to render the agent
        """

        return None

    def move(self, forces) -> None:

        self.acc = forces
        self.vel = self.vel + DT * self.acc
        self.pos = self.pos + DT * self.vel


class Seeker(Agent):
    def __init__(self,):
        super(Seeker, self).__init__()
        self.status = {
            "free": True,  # No target assigned
            "going": False,  # Going to the target
            "checking": False,  # Currently checking a target
        }


class Checker(Agent):
    def __init__(self,):
        super(Checker, self).__init__()


class Target(Agent):
    def __init__(self,):
        super(Target, self).__init__()
        self.status = {
            "targeted": False,  # Targeted by a seeker
            "assigned": False,  # Assigned to checker
            "checked": False,  # Cheked by checker
        }


if __name__ == '__main__':
    seeker = Seeker()

    print(seeker.pos)
