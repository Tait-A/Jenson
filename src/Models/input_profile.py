import numpy as np
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
from splines import Spline
from robot import Robot
import sys

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Utils.state import State
from Utils.action import Action
from Utils.trajectory import Trajectory

STATE_TIMESTEP = 0.1


class Profiler:
    def __init__(self, path: Spline, robot: Robot, laps=2, intervals=200):
        self.intervals = intervals
        self.path = path
        self.robot = robot
        self.laps = laps

    def profile(self) -> list[Trajectory]:
        robot = self.robot
        path = self.path
        ti = np.linspace(0, 1, self.intervals, endpoint=False)
        steering_lim = self.robot.steering_lim
        max_speed = self.robot.max_speed
        max_acc = self.robot.max_acc
        max_decel = (
            self.robot.max_acc * 2
        )  # Approximation that robot can brake twice as fast as it accelerates

        nodes = path.spline(ti).T

        # calculate the curvature of the trajectory and radius of curvature
        k, radius = path.calc_curvature(self.intervals)

        sign = np.sign(k)

        steering_angle = sign * np.arctan(robot.wheelbase / radius)
        # ensure all steering angle values are below the steering limit
        steering_angle = np.where(
            abs(steering_angle) > steering_lim,
            np.sign(steering_angle) * steering_lim,
            steering_angle,
        )
        steering_profile = Spline(steering_angle, t=ti)
        radius_profile = Spline(radius, t=ti)
        gravity = 9.81

        max_cornering_velocity = np.sqrt(
            robot.friction * radius * gravity
        )  # set frictional force equal to centripetal force to get max velocity

        velocity = np.minimum(max_speed, max_cornering_velocity)

        first_lap_velocity = velocity.copy()

        first_lap_velocity[0] = max_acc * STATE_TIMESTEP

        # Henri uses a backwards pass to ensure that the updates dont push the velocity beyond it's limits with braking

        for i in range(1, len(first_lap_velocity)):
            dist = self.path.lin_dist(ti[i - 1], ti[i])
            t = 2 * dist / (first_lap_velocity[i - 1] + first_lap_velocity[i])
            if first_lap_velocity[i] - first_lap_velocity[i - 1] > max_acc * t:
                first_lap_velocity[i] = first_lap_velocity[i - 1] + max_acc * t
            if first_lap_velocity[i] > max_cornering_velocity[i]:
                first_lap_velocity[i] = max_cornering_velocity[i]

        for i in range(len(first_lap_velocity) - 1, 0, -1):
            dist = self.path.lin_dist(ti[i - 1], ti[i])
            t = 2 * dist / (first_lap_velocity[i - 1] + first_lap_velocity[i])
            decel = first_lap_velocity[i - 1] - first_lap_velocity[i]
            if decel > max_decel * t:
                first_lap_velocity[i - 1] = first_lap_velocity[i] + max_decel * t

        if velocity[0] - first_lap_velocity[-1] > max_acc * t:
            velocity[0] = first_lap_velocity[-1] + max_acc * t
        elif first_lap_velocity[-1] - velocity[0] > max_decel * t:
            velocity[0] = first_lap_velocity[-1] - max_decel * t

        for i in range(1, len(velocity)):
            dist = self.path.lin_dist(ti[i - 1], ti[i])
            t = 2 * dist / (velocity[i - 1] + velocity[i])
            if velocity[i] - velocity[i - 1] > max_acc * t:
                velocity[i] = velocity[i - 1] + max_acc * t
            if velocity[i] > max_cornering_velocity[i]:
                velocity[i] = max_cornering_velocity[i]

        for i in range(len(velocity) - 1, 0, -1):
            dist = self.path.lin_dist(ti[i - 1], ti[i])
            t = 2 * dist / (velocity[i - 1] + velocity[i])
            if velocity[i - 1] - velocity[i] > max_decel * t:
                velocity[i - 1] = velocity[i] + max_decel * t

        t1 = [0]
        for i in range(1, len(velocity)):
            dist = self.path.lin_dist(ti[i - 1], ti[i])
            t1.append(t1[-1] + 2 * dist / (velocity[i - 1] + velocity[i]))

        first_lap_velocity_spline = Spline(
            (np.append(first_lap_velocity, velocity[0])),
            t=np.linspace(0, 1, self.intervals + 1),
        )
        velocity_spline = Spline(
            np.append(velocity, velocity[0]), t=np.linspace(0, 1, self.intervals + 1)
        )

        # Parameterise by displacement
        states = [[] for _ in range(self.laps)]
        actions = [[] for _ in range(self.laps)]
        d = 0
        while d < self.laps:
            lap = int(d // 1.0)
            dist = d - lap
            if d < 1:
                v = first_lap_velocity_spline.spline(d)

            else:
                v = velocity_spline.spline(dist)

            x, y = path.spline(dist)
            theta = path.slope(dist)
            w = v / radius_profile.spline(dist)
            phi = steering_profile.spline(dist)
            if d == 0:
                acc = (v - 0) / STATE_TIMESTEP
            else:
                if len(states[lap]) == 0:
                    acc = (v - states[lap - 1][-1].v) / STATE_TIMESTEP
                else:
                    acc = (v - states[lap][-1].v) / STATE_TIMESTEP
            state = State(x, y, theta, v, w)
            action = Action(phi, acc, robot)

            states[lap].append(state)
            actions[lap].append(action)

            d += (v * STATE_TIMESTEP + 0.5 * acc * STATE_TIMESTEP**2) / path.length

        self.trajectories = [
            Trajectory(states[i], actions[i]) for i in range(self.laps)
        ]
        return self.trajectories
