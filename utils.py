import numpy as np

from simulator import Agent


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
    return vector / norm


if __name__ == '__main__':
    print(get_angle(np.array([0, 0])))
