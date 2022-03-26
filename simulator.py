from __future__ import annotations
import numpy as np
import uuid
# from env import Env

from utils import normalize
from constants import (DIM, RES, DT, F_FLUID, K_SEEKER, L0_SEEKER,
                       K_CHECKER, L0_CHECKER, C_NODE)


################################################################################
# AGENT CLASSES
################################################################################


class Agent:
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM, dtype=float),
                 vel: np.ndarray = np.zeros(DIM, dtype=float),
                 acc: np.ndarray = np.zeros(DIM, dtype=float),
                 env: Env = None,
                 radius: float = RES) -> None:
        self.id = uuid.uuid4()
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.env = env
        self.radius = radius  # Radius of communication/detection of other agents

    def move(self, forces: np.ndarray) -> None:
        self.acc = forces
        self.vel += DT * self.acc
        self.pos += DT * self.vel

    def fluid_force(self,) -> np.ndarray:
        return -F_FLUID * self.vel

    def spring_force(self, agentB) -> np.ndarray:
        return self.K * \
            (self.distance(agentB) - self.L0) * \
            normalize(self, agentB)

    def newton_force(self, agentB, cst: float) -> np.ndarray:
        if self.distance(agentB) == 0:
            print("Oops we dodged a division by zero -> ratio + pa lu")
            return np.zeros(DIM, dtype=float)
        return -cst * normalize(self, agentB) / self.distance(agentB)**2

    def distance(self, agent) -> float:
        """
        Distance between 2 agents
        """
        return np.sqrt(np.linalg.norm(self.pos - agent.pos))

    def neighbors_agents(self,
                         class_list={"Seeker", "Checker", "Target", "Node"}) -> set:
        """
        Returns the surrounding agents of the desired class(es), in a set
        """
        ids = set()
        for agent in self.env.agents:
            if (type(agent).__name__ in class_list) and (self.distance(agent) <= self.radius):
                if agent.id != self.id:
                    ids.add(agent.id)
        return ids

    def __repr__(self):
        return f"""{type(self).__name__} :\n
                    id = {self.id},\n
                    pos = {self.pos},\n
                    vel = {self.vel},\n
                    acc = {self.acc}
                """


class Seeker(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM, dtype=float),
                 vel: np.ndarray = np.zeros(DIM, dtype=float),
                 acc: np.ndarray = np.zeros(DIM, dtype=float),
                 env: Env = None) -> None:
        super(Seeker, self).__init__(pos, vel, acc, env)
        self.K = K_SEEKER
        self.L0 = L0_SEEKER


class Checker(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM, dtype=float),
                 vel: np.ndarray = np.zeros(DIM, dtype=float),
                 acc: np.ndarray = np.zeros(DIM, dtype=float),
                 env: Env = None) -> None:
        super(Checker, self).__init__(pos, vel, acc, env)
        self.K = K_CHECKER
        self.L0 = L0_CHECKER
        self.status = {
            "free": True,  # No target assigned
            "going": False,  # Going to the target
            "checking": False,  # Currently checking a target
        }


class Target(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM, dtype=float),
                 vel: np.ndarray = np.zeros(DIM, dtype=float),
                 acc: np.ndarray = np.zeros(DIM, dtype=float),
                 env: Env = None) -> None:
        super(Target, self).__init__(pos, vel, acc, env)
        self.status = {
            "targeted": False,  # Targeted by a seeker
            "assigned": False,  # Assigned to checker
            "checked": False,  # Cheked by checker
        }


class Node(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM, dtype=float),
                 vel: np.ndarray = np.zeros(DIM, dtype=float),
                 acc: np.ndarray = np.zeros(DIM, dtype=float),
                 env: Env = None) -> None:
        super(Node, self).__init__(pos, vel, acc, env)
        self.visited = False

    def value(self,) -> float:
        """
        Returns a newton force coefficient determined by the other nearby nodes
        Here, the value returned is inversely proportional to 1 plus the number
        of adjacent nodes
        """
        return C_NODE / (len(self.neighbors_agents(class_list={Node})) + 1)


################################################################################
# ENV CLASSES
################################################################################

class Env:

    def __init__(self, N_S: int, N_C: int, N_T: int) -> None:
        self.agents = set()

        for i in range(N_S):
            self.agents.add(Seeker(self.pos_init(), env=self))

        for i in range(N_C):
            self.agents.add(Checker(self.pos_init(), env=self))

        for i in range(N_T):
            self.agents.add(Checker(self.pos_init(), env=self))

    def add_agent(self, agent):
        self.agents.add(agent)

    def get_agents_by_type(self, test):
        res = set()
        for agent in self.agents:
            if type(agent).__name__ == test:
                res.add(agent)
        return res

    def update(self,):
        pass

    def render(self,):
        pass

    def pos_init(self, lower_bounds=[0., 0.], upper_bounds=[1., 1.]):

        return np.random.uniform(lower_bounds, upper_bounds, size=2)

    def vel_init(self):
        pass


if __name__ == '__main__':
    env = Env(3, 3, 3)
    for agent in env.agents:
        print(agent)
