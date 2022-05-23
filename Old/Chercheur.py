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


class Chercheur():
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
        self.env = env
        self.pos = np.array([x, y])
        self.speed = np.array([0., 0.])
        self.acc = np.array([0., 0.])
        self.k = 20
        self.l0 = 150
        self.maxspeed = speed
        self.state = 'normal'
        self.battery = None
        self.dir = direction
        self.size = size
        self.target = None
        self.id = id
        self.cdv = 50
        self.destination = None
        self.inbox = []
        self.message = {'sender_id': None, 'recipient_id': None, 'time': None, 'message': {'status': {'x': None, 'y': None, 'z': None, 'dir': None,
                                                                                                      'speed': None, 'state': None, 'battery': None}, 'alert': {'verif': None, 'help': None, 't_x': None, 't_y': None, 't_z': None}}}
        self.dico_cible = {}  # {id:{pos, state}}

    def make_message(self, recipient):
        self.message['sender_id'] = self.id
        self.message['recipient_id'] = recipient.id
        self.message['time'] = t
        self.message['message']['status']['x'] = self.x
        self.message['message']['status']['y'] = self.y
        self.message['message']['status']['z'] = self.z
        self.message['message']['status']['dir'] = self.dir
        self.message['message']['status']['speed'] = self.speed
        self.message['message']['status']['state'] = self.state
        self.message['message']['status']['battery'] = self.battery

        if self.target != None:
            self.message['message']['alert']['t_x'] = self.target.x
            self.message['message']['alert']['t_y'] = self.target.y
            self.message['message']['alert']['t_z'] = self.target.z

    def read_message(self):
        pass

    def send_message(self):
        pass

    def score(self):
        pass

    def display(self):
        """ Dessine le drone sur l'écran """

        a = self.dir
        x = self.pos[0]
        y = self.pos[1]
        s = self.size

        pygame.draw.line(self.env.screen, white,
                         (x - (s * math.sqrt(130) / 12) * math.cos(math.atan(7 / 9) - a),
                          y - (s * math.sqrt(130) / 12) * math.sin(math.atan(7 / 9) - a)),
                         (x + s * math.cos(-a), y + s * math.sin(-a)))

        pygame.draw.line(self.env.screen, white,
                         (x - (s * math.sqrt(130) / 12) * math.cos(math.atan(7 / 9) + a),
                          y + (s * math.sqrt(130) / 12) * math.sin(math.atan(7 / 9) + a)),
                         (x + s * math.cos(a), y + s * math.sin(-a)))

        pygame.draw.line(self.env.screen, white,
                         (x - (s * math.sqrt(2) / 2) * math.cos(-a + math.pi / 4),
                          y - (s * math.sqrt(2) / 2) * math.sin(-a + math.pi / 4)),
                         (x - (s * math.sqrt(2) / 2) * math.cos(a + math.pi / 4),
                          y + (s * math.sqrt(2) / 2) * math.sin(a + math.pi / 4)))

    def neighbours(self, r=200):
        """ liste des agents (Chercheur ou Target) à distance <= r du Chercheur """

        return [self.env.Agent_list[i] for i in range(len(self.env.Agent_list)) if distance(self, self.env.Agent_list[i]) < r and self.env.Agent_list[i] != self]

    def check_mesh(self):
        """ Vérifie si des points du maillage sont visibles par le chercheur et les désactive si c'est le cas, on suppose pour le moment que la résolution
        du maillage coincide avec le champ de vision des chercheurs.
        On utiise un try/except pour éviter des bugs de nature inconnue qui causent des crashes
        """
        try:
            colonne = round(self.pos[0] / self.env.res)
            ligne = round(self.pos[1] / self.env.res)
            for i in range(-2, 3):
                for j in range(-2, 3):
                    try:
                        if point_distance(self.pos[0], self.pos[1], self.env.res*(colonne+i), self.env.res*(ligne+j)) < self.cdv:
                            self.env.mesh[ligne+j][colonne+i] = 0
                    except:
                        pass

        except:
            pass
