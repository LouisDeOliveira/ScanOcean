import pygame
import math
import uuid
import random
import numpy as np
np.seterr(divide='ignore', invalid='ignore')


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


def distance(Agent1, Agent2):
    """ distance entre deux agents """
    x1, y1 = Agent1.pos
    x2, y2 = Agent2.pos

    d = math.sqrt((x1-x2)**2+(y1-y2)**2)
    return d


def point_distance(x1, y1, x2, y2):
    """ distance entre deux points """
    d = math.sqrt((x1-x2)**2+(y1-y2)**2)
    return d


def vect_AB(agentA, agentB):
    v_x = agentB.pos[0]-agentA.pos[0]
    v_y = agentB.pos[1]-agentA.pos[1]

    """if (v_x**2+v_x**2) == 0:
        return np.array([0, 0])
    else:
        return np.array([v_x, v_y])/np.sqrt(v_x**2+v_x**2)"""
    return np.array([v_x, v_y])/np.sqrt(v_x**2+v_x**2)


def vect_norme_carre(vect):
    return vect[0]**2 + vect[1]**2


def normalize_vector(vect):
    return vect/np.sqrt(vect_norme_carre(vect))


def fusion_dico(dic1, dic2):
    dic = dic1
    for id in dic2.keys():
        if id in dic1.keys():
            if dic1[id][3] == False:
                dic[id] = dic2[id]
            else:
                if dic1[id][1] == False:
                    dic[id] = dic2[id]
        else:
            dic[id] = dic2[id]
    return dic


def distance_pos(a, b):
    dx = a[0]-b[0]
    dy = a[1]-b[1]
    return math.sqrt((dx)**2+(dy)**2)
