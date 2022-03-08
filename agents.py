import numpy as np
from env import Env


class Agent:

    def __init__(self, pos: np.ndarray, vel: np.ndarray, acc: np.ndarray,
                 env: Env):

        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.env = env


class Chercheur(Agent):

    def __init__(self,):
        super(Chercheur, self).__init__()


class Verificateur(Agent):
    def __init__(self,):
        super(Verificateur, self).__init__()


class Target(Agent):
    def __init__(self,):
        super(Target, self).__init__()


if __name__ == '__main__':
    chercheur = Chercheur()
    print(chercheur)
