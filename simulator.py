from __future__ import annotations
import matplotlib.pyplot as plt
import numpy as np
import uuid
import time


from utils import normalize
from constants import (DIM, RES, DT, F_FLUID, K_SEEKER, L0_SEEKER,
                       K_CHECKER, L0_CHECKER, CONSTANTS, C_NODE)


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

    def move(self) -> None:
        self.vel += DT * self.acc
        self.pos += DT * self.vel

    def fluid_force(self,) -> np.ndarray:
        return -F_FLUID * self.vel

    def spring_force(self, agents) -> np.ndarray:
        force = np.zeros(DIM, dtype=float)
        for agentB in agents:
            if agentB is None:
                continue
            else:
                force += self.K * \
                    (self.distance(agentB) - self.L0) * \
                    normalize((agentB.pos - self.pos))
        return force

    def newton_force(self, agents) -> np.ndarray:
        force = np.zeros(DIM, dtype=float)
        for agentB in agents:
            if self.distance(agentB) == 0:
                print("Oops we dodged a division by zero -> ratio + pa lu")
                continue
            if agentB is None:
                return np.zeros(DIM, dtype=float)
            else:

                force -= CONSTANTS[type(self).__name__][type(agentB).__name__] * \
                    normalize((agentB.pos - self.pos)) / \
                    self.distance(agentB)**2
        return force

    def distance(self, agent) -> float:
        """
        Distance between 2 agents
        """
        return np.sqrt(np.linalg.norm(self.pos - agent.pos))

    def neighbors_agents(self,
                         class_set={"Seeker",
                                    "Checker",
                                    "Target",
                                    "Node", }) -> set:
        """
        Returns the surrounding agents of the desired class(es), in a set
        """
        neighbours = set()
        for agent in self.env.agents:
            if (type(agent).__name__ in class_set) and (self.distance(agent) <= self.radius):
                if agent.id != self.id:
                    neighbours.add(agent)
        return neighbours

    def get_forces(self,):
        self.acc = np.zeros(2)

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

    def get_forces(self,):
        forces = []
        forces.append(self.fluid_force())
        forces.append(self.spring_force([agent
                                         for agent in self.neighbors_agents(class_set={"Seeker", "Checker"})]))
        forces.append(self.newton_force([agent
                                         for agent in self.neighbors_agents({"Node"})]))
        forces.append(self.newton_force([agent
                                         for agent in self.neighbors_agents({"Target"})
                                         if not agent.status["targeted"]
                                         and not agent.status["assigned"]
                                         and not agent.status["checked"]]))
        self.acc = np.sum(np.array(forces), axis=0)


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

    def get_forces(self,):
        forces = []
        forces.append(self.fluid_force())
        forces.append(self.spring_force([agent
                                         for agent in self.neighbors_agents(class_set={"Checker"})]))
        forces.append(self.newton_force([agent
                                        for agent in self.neighbors_agents({"Target"})
                                        if not agent.status["assigned"]
                                        and not agent.status["checked"]]))
        self.acc = np.sum(np.array(forces), axis=0)


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
        self.time = 0

        for _ in range(N_S):
            self.agents.add(Seeker(self.pos_init(), env=self))

        for _ in range(N_C):
            self.agents.add(Checker(self.pos_init(), env=self))

        for _ in range(N_T):
            self.agents.add(Target(self.pos_init(), env=self))

    def add_agent(self, agent: Agent) -> None:
        self.agents.add(agent)

    def get_agents_by_type(self, test: str):
        res = set()
        for agent in self.agents:
            if type(agent).__name__ == test:
                res.add(agent)
        return res

    def update(self,):
        # For all agents type, sum the forces, update their pos/speed/acc
        self.time += DT

        for agent in self.agents:
            # All forces must be computed before updating
            # positions and velocities
            agent.get_forces()

        for agent in self.agents:
            # with all forces updated, we can move all the agents
            agent.move()

    def render(self,):

        colors = {"Checker": "red",
                  "Seeker": "blue",
                  "Target": "green",
                  "Node": "yellow"}

        for agent in self.agents:
            plt.scatter(agent.pos[0], agent.pos[1],
                        c=colors[type(agent).__name__])

        plt.show()
        time.sleep(DT)
        plt.close()

    def pos_init(self, lower_bounds=[-10, -10.], upper_bounds=[10., 10.]):

        return np.random.uniform(lower_bounds, upper_bounds, size=2)


if __name__ == '__main__':
    env = Env(5, 5, 5)
    env.render()
    for _ in range(10):
        env.update()
        env.render()
