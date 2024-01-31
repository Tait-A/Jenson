import numpy as np
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
from splines import Spline
from robot import Robot

class Profiler:
    def __init__(self, trajectory: Spline, robot: Robot, intervals = 200):
        self.intervals = intervals
        self.trajectory = trajectory
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

        # calculate the curvature of the trajectory and radius of curvature
        _, radius = trajectory.calc_curvature(self.intervals)

        steering_angle = np.arctan(robot.wheelbase / radius)
        self.steering_profile = Spline(steering_angle)
        gravity = 9.81

        max_velocity = np.sqrt(robot.friction * radius * gravity) # set frictional force equal to centripetal force to get max velocity

        velocity = np.minimum(robot.max_speed, max_velocity)

        first_lap_velocity = velocity.copy()

        first_lap_velocity[0] = max_acc

        for i in range(1, len(first_lap_velocity)):
            if first_lap_velocity[i] - first_lap_velocity[i-1] > max_acc:
                first_lap_velocity[i] = first_lap_velocity[i-1] + max_acc
            if first_lap_velocity[i] > max_velocity:
                first_lap_velocity[i] = max_velocity

        if velocity[0] - first_lap_velocity[-1] > max_acc:
            velocity[0] = first_lap_velocity[-1] + max_acc:

        for i in range(1, len(velocity)):
            if velocity[i] - velocity[i-1] > max_acc:
                velocity[i] = velocity[i-1] + max_acc
            if velocity[i] > max_velocity:
                velocity[i] = max_velocity




        acceleration =
