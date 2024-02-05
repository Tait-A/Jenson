import numpy as np
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
from splines import Spline
from robot import Robot
import sys
sys.path.insert(1,"/Users/alistair/Projects/Dissertation/Jenson/src")
from Utils.state import State
from Utils.action import Action

STATE_TIMESTEP = 0.1

class Profiler:
    def __init__(self, trajectory: Spline, robot: Robot, intervals = 200):
        self.intervals = intervals
        self.trajectory = trajectory
        self.robot = robot

        self.profile()

    def profile(self) -> list[(State, Action)]:
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
        # ensure all steering angle values are below the steering limit
        steering_angle = np.where(abs(steering_angle) > steering_lim, np.sign(steering_angle) * steering_lim, steering_angle)
        steering_profile = Spline(steering_angle, t = ti)
        radius_profile = Spline(radius, t = ti)
        gravity = 9.81

        max_cornering_velocity = np.sqrt(robot.friction * radius * gravity) # set frictional force equal to centripetal force to get max velocity

        velocity = np.minimum(max_speed, max_cornering_velocity)

        first_lap_velocity = velocity.copy()

        first_lap_velocity[0] = max_acc



        for i in range(1, len(first_lap_velocity)):
            dist = self.trajectory.lin_dist(ti[i-1],ti[i])
            t = 2 * dist / (first_lap_velocity[i] + first_lap_velocity[i-1])
            if first_lap_velocity[i] - first_lap_velocity[i-1] > max_acc * t:
                first_lap_velocity[i] = first_lap_velocity[i-1] + max_acc * t
            if first_lap_velocity[i] > max_cornering_velocity:
                first_lap_velocity[i] = max_cornering_velocity


        if velocity[0] - first_lap_velocity[-1] > max_acc:
            velocity[0] = first_lap_velocity[-1] + max_acc

        for i in range(1, len(velocity)):
            dist = self.trajectory.lin_dist(ti[i-1],ti[i])
            t = 2 * dist / (velocity[i] + velocity[i-1])
            if velocity[i] - velocity[i-1] > max_acc * t:
                velocity[i] = velocity[i-1] + max_acc * t
            if velocity[i] > max_cornering_velocity:
                velocity[i] = max_cornering_velocity

        first_lap_velocity_spline = Spline((first_lap_velocity.append(velocity[0])), t = np.linspace(0,1, self.intervals+1))
        velocity_spline = Spline(velocity.append(velocity[0]), t = np.linspace(0,1, self.intervals+1))

        # Parameterise by displacement
        states = []
        actions = []
        d = 0
        while (d < 2):
            if d < 1:
                v = first_lap_velocity_spline.spline(d)
            else:
                v = velocity_spline.spline(d-1)

            x, y = trajectory.spline(d)
            theta = trajectory.slope(d)
            w = v / radius_profile.spline(d) 
            phi = steering_profile(d)
            if d == 0:
                acc = (v - 0) / STATE_TIMESTEP
            acc = (v - states[-1].v) / STATE_TIMESTEP
            state = State(x, y, theta, v, w)
            action = Action(phi, acc, robot)

            states.append(state)
            actions.append(action)

            d += v * STATE_TIMESTEP

        return list(zip(states, actions))
            


            




        

        

