import cv2
import numpy as np
import math
import time
import sys
import os

sys.path.insert(1, os.path.join(sys.path[0], ".."))
from Utils.state import State

GRAVITY = 9.81


class Robot:
    def __init__(
        self,
        state=State(0, 1.15, 0),
        width=0.1,
        wheelbase=0.2,
        mass=1,
        steering_lim=1,
        max_speed=3,
        max_acc=1.5,
        friction=0.5,
    ):
        self.state = state
        self.width = width  # width of robot
        self.wheelbase = wheelbase  # wheelbase of robot
        self.mass = mass  # mass of robot
        self.steering_lim = steering_lim  # steering limits of robot
        self.max_speed = max_speed  # maximum speed of robot
        self.max_acc = max_acc  # maximum acceleration of robot
        self.friction = friction  # friction coefficient of robot (rubber on lino)

    def run(self):
        position = self.localise()

    def localise(self):
        # implementation of localisation algorithm
        pass

    def get_control(self):
        # implementation of control algorithm
        pass
