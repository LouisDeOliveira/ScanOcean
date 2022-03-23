import numpy as np
import uuid

from utils import get_angle
from constants import *
from env import Env


class Agent:
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM),
                 vel: np.ndarray = np.zeros(DIM),
                 acc: np.ndarray = np.zeros(DIM),
                 env: Env = None,
                 radius: float = RES) -> None:
        self.id = uuid.uuid1()
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.env = env
        self.radius = radius # Radius of communication/detection of other agents

    def move(self, forces: np.ndarray) -> None:
        self.acc = forces
        self.vel += DT * self.acc
        self.pos += DT * self.vel

    def fluid_force(self,) -> np.ndarray:
        return -F_FLUID * self.vel

    def distance(self, agentB) -> float:
        """
        Distance between to agents 
        """
        return np.sqrt(np.linalg.norm(self.pos-agentB.pos))

    def neighbors_agents(self, class_list = {"Seeker", "Checker", "Target", "Node"}) -> set:
        """
        Returns the surrounding agents of the desired class(es)
        """
        ids = set()
        for agent in self.env.agents : 
            if (type(agent) in class_list) and (self.distance(self, agent) <= self.radius):
                ids.add(agent.id)
        return ids


class Seeker(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM),
                 vel: np.ndarray = np.zeros(DIM),
                 acc: np.ndarray = np.zeros(DIM),
                 env: Env = None) -> None:
        super(Seeker, self).__init__(pos, vel, acc, env)
        self.K = K_SEEKER
        self.L0 = L0_SEEKER


class Checker(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM),
                 vel: np.ndarray = np.zeros(DIM),
                 acc: np.ndarray = np.zeros(DIM),
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
                 pos: np.ndarray = np.zeros(DIM),
                 vel: np.ndarray = np.zeros(DIM),
                 acc: np.ndarray = np.zeros(DIM),
                 env: Env = None) -> None:
        super(Target, self).__init__(pos, vel, acc, env)
        self.status = {
            "targeted": False,  # Targeted by a seeker
            "assigned": False,  # Assigned to checker
            "checked": False,  # Cheked by checker
        }


class Node(Agent):
    def __init__(self,
                 pos: np.ndarray = np.zeros(DIM),
                 vel: np.ndarray = np.zeros(DIM),
                 acc: np.ndarray = np.zeros(DIM),
                 env: Env = None) -> None:
        super(Node, self).__init__(pos, vel, acc, env)
        self.visited= False

    def value(self,) -> float:
        """
        Returns a newton force coefficient determined by the other nearby nodes
        Here, the value returned is inversely proportional to 1 plus the number of adjacent nodes
        """
        return C_NODE / (len(neighbors_agents(self, radius = RES, class_list = Node)) + 1)


if __name__ == '__main__':
    seeker = Seeker()

    print(seeker.pos)
