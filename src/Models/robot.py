
import cv2
import numpy as np
import math
import time
import sys
import os

class Robot:
    def __init__(self, x=0, y=0, theta=0, phi=0, v=0, w=0, width=0, length=0, steering_lims=45):
        self.x = x              # x position
        self.y = y              # y position
        self.theta = theta      # angle of robot
        self.phi = phi          # steering angle of robot
        self.v = v              # linear velocity
        self.w = w              # angular velocity
        self.width = width      # width of robot
        self.length = length    # length of robot
        self.steering_lims = steering_lims # steering limits of robot

    def localise(self):
        # implementation of localisation algorithm
        pass

    def get_control(self):
        # implementation of control algorithm
        pass




