from agents import *
from utils import *


class Env:

    def __init__(self,) -> None:
        pass

    def update(self,):
        pass

    def spring_force(self, agentA: Agent, agentB: Agent) -> np.ndarray:
        return agentA.K * \
            (distance(agentA, agentB) - agentA.L0) * \
            normalize(agentA, agentB)

    def newton_force(self, agentA: Agent, agentB: Agent, cst: float) -> np.ndarray:
        if distance(agentA, agentB) == 0:
            print("Oops we dodged a division by zero -> ratio + pa lu")
            return np.zeros(DIM)
        return -cst * normalize(agentA, agentB) / distance(agentA, agentB)**2

    def render(self,):
        pass
