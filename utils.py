import numpy as np

from agents import Agent


def get_angle(vector: np.ndarray) -> float:
    """
    Returns the angle formed by a 2d vector in radians
    """
    return np.arctan2(*vector)


def normalize(vector: np.ndarray) -> np.ndarray:
    """
    Normalizes a vector, return 0 if input vector is 0
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector/norm


def distance(agentA: Agent, agentB: Agent) -> float:
    """
    Distance between to agents 
    """
    return np.sqrt(np.linalg.norm(agentA.pos-agentB.pos))


def neighbors_agents(agentA, radius, class_list = {"Seeker", "Checker", "Target", "Node"}) -> set:
    """
    Returns the surrounding agents of the desired class(es)
    """
    ids = set()
    for agentB in Agent:  # does that work ? 
        if (class(agentB) in class_list) and (distance(agentA, agentB) <= radius):
            ids.add(agentB.id)
    return ids

if __name__ == '__main__':
    print(get_angle(np.array([0, 0])))
