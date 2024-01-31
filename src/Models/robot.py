
import cv2
import numpy as np
import math
import time
import sys
import os

GRAVITY = 9.81

class Robot:
    def __init__(self, x=0, y=0, theta=0, phi=0, v=0, w=0, width=0, wheelbase=0.2, mass = 1, steering_lim=45, max_speed = 3, max_acc = 0.5, friction = 0.5):
        self.x = x                      # x position
        self.y = y                      # y position
        self.theta = theta              # angle of robot
        self.phi = phi                  # steering angle of robot
        self.v = v                      # linear velocity
        self.w = w                      # angular velocity
        self.width = width              # width of robot
        self.wheelbase = wheelbase      # wheelbase of robot
        self.mass = mass                # mass of robot
        self.steering_lim = steering_lim # steering limits of robot
        self.max_speed = max_speed      # maximum speed of robot
        self.max_acc = max_acc          # maximum acceleration of robot
        self.friction = friction        # friction coefficient of robot (rubber on lino)

    def run(self):
        position = self.localise()

    def localise(self):
        # implementation of localisation algorithm
        pass

    def get_control(self):
        # implementation of control algorithm
        pass




