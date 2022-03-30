# PYGAME CONSTANTS
WIDTH = 800
HEIGHT = 800
TICK_FREQ = 100
DT = 1 / TICK_FREQ
SIZE = 5
COLORS = {
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "shadow": (80, 80, 80),
    "light_green": (0, 255, 0),
    "green": (0, 200, 0),
    "blue": (0, 0, 128),
    "lightblue": (0, 0, 255),
    "lightred": (255, 100, 100),
    "purple": (102, 0, 102),
    "lightpurple": (153, 0, 153),
    "black": (0, 0, 0)
}


# SIMULATION PARAMETERS
DIM = 2
MAX_SPEED = 100
MAX_ACC = 900
F_FLUID = 2
RES = 150

K_SEEKER = 20
K_CHECKER = 10

L0_SEEKER = 150
L0_CHECKER = 200

FOV_SEEKER = 50
FOV_CHECKER = 50

C_NODE = 100000  # Constant in the Newtonian force attracting the Seekers to the Nodes

# F_SEEKER_TARGET = 100000

CONSTANTS = {"Seeker": {"Node": 100000, "Target": 100000},
             "Checker": {"Node": 100000, "Target": 100000}}
