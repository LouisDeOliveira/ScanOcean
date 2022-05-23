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


class Verificateur():
    """
    speed : int/float : vitess du drone
    env : Environment
    state : normal (pas de cible, en recherche), finder (à trouvé une cible), helper (aide un finder)
    battery : niveau de batterie : 0 -> dcd
    dir : float : direction du drone
    size : Taille d'affichage du drone
    target : Target :  cible du drone
    id : int : identifiant unique du drone
    destination : tuple : destination du drone
    inbox : list : liste des messages reçus par le drone
    message :  dict : message a envoyer

    """

    def __init__(self, x, y, speed, direction, size, id, env):
        self.speed = speed
        self.env = env
        self.pos = np.array([x, y])
        self.speed = np.array([0., 0.])
        self.acc = np.array([0., 0.])
        self.k = 10
        self.l0 = 200
        self.maxspeed = maxspeed
        self.state = 'normal'
        self.battery = None
        self.dir = direction
        self.size = size
        # [coordonnées de destination, id du Target associé]
        self.target = None  # objet de la classe target
        self.id = id
        self.destination = None
        self.inbox = []
        self.message = {'sender_id': None, 'recipient_id': None, 'time': None, 'message': {'status': {'x': None, 'y': None, 'z': None, 'dir': None,
                                                                                                      'speed': None, 'state': None, 'battery': None}, 'alert': {'verif': None, 'help': None, 't_x': None, 't_y': None, 't_z': None}}}
        self.dico_cible = {}  # {id:{pos, state, id}}
        self.time = 0

    def check_target(self):
        if self.env.time-self.time >= 1 and self.time != 0:
            self.dico_cible[self.target.id][1] = True
            self.target.checked = True
            self.target = None
            self.time = 0

    def move(self):
        try:

            d = point_distance(
                self.x, self.y, self.destination[0], self.destination[1])

            cosr = (self.destination[0] - self.x)/d
            sinr = (self.destination[1] - self.y)/d
            new_x = max(min(self.x + cosr * self.speed*dt,
                            self.env.width), 0)
            new_y = max(min(self.y + sinr * self.speed*dt,
                            self.env.height), 0)
            new_dir = math.atan2(-sinr, cosr)
            return new_x, new_y, new_dir

        except:
            return self.x, self.y, self.dir

    def step(self):
        self.target = self.target_agent()
        self.wander()
        self.x, self.y, self.dir = self.move()

    def display(self):
        """ Dessine le drone sur l'écran """

        a = self.dir
        x = self.pos[0]
        y = self.pos[1]
        s = self.size

        pygame.draw.line(self.env.screen, lightgreen,
                         (x - (s * math.sqrt(130) / 12) * math.cos(math.atan(7 / 9) - a),
                          y - (s * math.sqrt(130) / 12) * math.sin(math.atan(7 / 9) - a)),
                         (x + s * math.cos(-a), y + s * math.sin(-a)))

        pygame.draw.line(self.env.screen, lightgreen,
                         (x - (s * math.sqrt(130) / 12) * math.cos(math.atan(7 / 9) + a),
                          y + (s * math.sqrt(130) / 12) * math.sin(math.atan(7 / 9) + a)),
                         (x + s * math.cos(a), y + s * math.sin(-a)))

        pygame.draw.line(self.env.screen, lightgreen,
                         (x - (s * math.sqrt(2) / 2) * math.cos(-a + math.pi / 4),
                          y - (s * math.sqrt(2) / 2) * math.sin(-a + math.pi / 4)),
                         (x - (s * math.sqrt(2) / 2) * math.cos(a + math.pi / 4),
                          y + (s * math.sqrt(2) / 2) * math.sin(a + math.pi / 4)))

    def neighbours(self, r=200):
        """ liste des agents(Chercheur ou Target) à distance <= r du Chercheur """

        return [self.env.Agent_list[i] for i in range(len(self.env.Agent_list)) if distance(self, self.env.Agent_list[i]) < r and self.env.Agent_list[i] != self]
