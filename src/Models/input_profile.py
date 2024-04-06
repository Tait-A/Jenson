import numpy as np
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
import sys

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Utils.state import State
from Utils.action import Action
from Utils.trajectory import Trajectory
from Utils.splines import Spline
from Models.robot import Robot


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

        radius = np.array(radius)
        gravity = 9.81

        # print(radius)
        # steering_profile.plot()
        # radius_profile.plot()

        max_cornering_velocity = np.sqrt(
            robot.friction * radius * gravity
        )  # set frictional force equal to centripetal force to get max velocity

        avg_radius = radius.mean()
        print("Avg radius: ", avg_radius)

        avg_velocity = max_cornering_velocity.mean()
        print("Avg velocity: ", avg_velocity)

        velocity = np.minimum(max_speed, max_cornering_velocity)

        self.max_velocity = velocity.copy()

        max_velocity_spline = Spline(self.max_velocity, t=ti)
        # max_velocity_spline.plot()

        first_lap_velocity = velocity.copy()

        first_lap_velocity[0] = 0

        param = [0] * 200

        for i in range(1, len(first_lap_velocity)):
            dist = self.path.lin_dist(ti[i - 1], ti[i])  # linear distance to the next
            param[i] = param[i - 1] + dist

            # time taken to travel the distance (assuming constant acceleration)
            u = first_lap_velocity[i - 1]
            a = max_acc
            t = (-u + np.sqrt(u**2 + 2 * a * dist)) / a

            if first_lap_velocity[i] - first_lap_velocity[i - 1] > max_acc * t:
                first_lap_velocity[i] = first_lap_velocity[i - 1] + max_acc * t

        param = np.array(param) / param[-1]
        velocity[0] = 2  # first_lap_velocity[-1]

        for i in range(1, len(velocity)):
            dist = self.path.lin_dist(ti[i - 1], ti[i])

            u = velocity[i - 1]
            a = max_acc
            t = (-u + np.sqrt(u**2 + 2 * a * dist)) / a
            if velocity[i] - velocity[i - 1] > max_acc * t:
                velocity[i] = velocity[i - 1] + max_acc * t

        # Backwards pass to ensure that the updates don't push the velocity beyond it's limits with braking
        velocity[-1] = velocity[0]

        for i in range(len(velocity) - 1, 0, -1):
            dist = self.path.lin_dist(ti[i - 1], ti[i])

            u = velocity[i]
            a = max_decel
            t = (-u + np.sqrt(u**2 + 2 * a * dist)) / a

            if velocity[i - 1] - velocity[i] > max_decel * t:
                velocity[i - 1] = velocity[i] + max_decel * t

        first_lap_velocity[-1] = velocity[0]

        for i in range(len(first_lap_velocity) - 1, 0, -1):
            dist = self.path.lin_dist(ti[i - 1], ti[i])

            u = first_lap_velocity[i]
            a = max_decel
            t = (-u + np.sqrt(u**2 + 2 * a * dist)) / a

            decel = first_lap_velocity[i - 1] - first_lap_velocity[i]
            if decel > max_decel * t:
                first_lap_velocity[i - 1] = first_lap_velocity[i] + max_decel * t

        self.first_lap_velocity = first_lap_velocity
        self.velocity = velocity

        avg_velocity = velocity.mean()
        print("Avg velocity: ", avg_velocity)

        first_lap_velocity_spline = Spline(
            first_lap_velocity,
            t=param,
        )
        velocity_spline = Spline(
            velocity,
            t=param,
        )

        # first_lap_velocity_spline.plot()
        # velocity_spline.plot()

        # Print the path length
        print(f"Path length: {path.length}")

        # Parameterise by displacement
        states = [[] for _ in range(self.laps)]
        x, y = path.spline(0)
        prev_state = State(0, y, 0, 0, 0)
        # print(x, y)
        actions = [[] for _ in range(self.laps)]
        d = max_acc * STATE_TIMESTEP**2 / 2 / path.length
        # print(d)
        while d < self.laps:
            lap = int(d // 1.0)
            dist = d - lap

            u = prev_state.v
            if d < 1:
                v = first_lap_velocity_spline.spline(dist)
            else:
                v = velocity_spline.spline(dist)

            x, y = path.spline(dist)
            theta = path.slope(dist)
            phi = steering_profile.spline(dist)
            w = v / self.robot.wheelbase * np.tan(phi)

            acc = (v - u) / STATE_TIMESTEP

            state = State(x, y, theta, v, w)
            action = Action(phi, acc, robot)

            # state.pretty_print()
            # act_state = action.apply(state)
            # act_state.pretty_print()

            states[lap].append(prev_state)
            actions[lap].append(action)

            d += ((u + v) * STATE_TIMESTEP) / (2 * path.length)
            prev_state = state

        self.trajectories = [
            Trajectory(states[i], actions[i]) for i in range(self.laps)
        ]
        return self.trajectories
