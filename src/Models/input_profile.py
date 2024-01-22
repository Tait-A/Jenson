import numpy as np
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
from splines import Spline

class Profiler:
    def __init__(self, trajectory, robot, intervals = 200):
        self.intervals = intervals
        self.trajectory = self.trajectory
        self.robot = robot

        self.profile()

    def profile(self):
        robot = self.robot
        trajectory = self.trajectory
        ti = np.linspace(0, 1, self.intervals, endpoint=False)
        steering_lim = self.robot.steering_lim
        max_speed = self.robot.max_speed
        max_acc = self.robot.max_acc

        nodes = trajectory.spline(ti).T

        # calculate the curvature of the trajectory
        curvature = trajectory.curvature()