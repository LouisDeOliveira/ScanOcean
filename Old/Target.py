import pygame
import math
import uuid
import random
import numpy as np
from Utils import *

white = (255, 255, 255)
red = (255, 0, 0)
f = 2
maxacc = 900.0
maxspeed = 100.0
circle_list = []
shadow = (80, 80, 80)
lightgreen = (0, 255, 0)
green = (0, 200, 0)
blue = (0, 0, 128)
lightblue = (0, 0, 255)
lightred = (255, 100, 100)
purple = (102, 0, 102)
lightpurple = (153, 0, 153)
res = 150
k = 50000


class Target():
    """ Voir drone
        targeted : Boolean : True si ciblé par un Chercheur """

    def __init__(self, x, y, id, env):
        self.id = id
        self.speed = np.array([0., 0.])
        self.acc = np.array([0., 0.])
        self.env = env
        self.pos = np.array([x, y])
        self.targeted = False
        self.checked = False     # False si non trouvé ou seulement vu ; True si vérifié
        self.checking = False  # True si la cible est attribué à un vérificateur

    def display(self):
        x, y = self.pos
        pygame.draw.circle(self.env.screen, red, (x, y), 8, 5)
        if self.targeted:
            pygame.draw.circle(self.env.screen, blue, (x, y), 8, 5)
        if self.checked:
            pygame.draw.circle(self.env.screen, green, (x, y), 8, 5)
