import numpy as np
import sys
import os
import casadi as ca

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.robot import Robot
from Utils.state import State


class Action:
    def __init__(self, steering, acceleration, car: Robot, timestep=0.1):
        self.steering = steering
        self.acceleration = acceleration
        self.timestep = timestep
        self.car = car

    def apply(self, state: "State") -> "State":
        # apply the action to the state to get a new state
        v_old = state.v
        w_old = state.w
        v_new = v_old + self.acceleration * self.timestep
        v_new = ca.fmin(v_new, self.car.max_speed)
        w_new = (v_new / self.car.wheelbase) * np.tan(self.steering)
        delta_theta = ((w_old + w_new) / 2) * state.timestep
        theta_new = state.theta + delta_theta
        dx, dy = self.integrate(v_new, v_old, state.theta, w_old, w_new)

        x_new = state.x + dx
        y_new = state.y + dy

        return State(x_new, y_new, theta_new, v_new, w_new)

    def integrate(self, v, u, theta, w_o, w_n, steps=20):  # approximate integration
        dx = 0
        dy = 0
        acc = v - u
        w_acc = w_n - w_o

        for i in range(steps):
            step = 2 * i + 1
            frac = step / (2 * steps)
            t = frac * self.timestep

            inst_v = u + acc * t

            inst_theta = theta + (w_o + w_acc * frac) * t

            dy += inst_v * np.sin(inst_theta) * self.timestep / steps
            dx += inst_v * np.cos(inst_theta) * self.timestep / steps

            # dx += inst_v * self.cos_approx(inst_theta) * self.timestep / steps
            # dy += inst_v * self.sin_approx(inst_theta) * self.timestep / steps
        return dx, dy

    def to_dict(self):
        return {
            "steering": float(self.steering),
            "acceleration": float(self.acceleration),
            "timestep": float(self.timestep),
        }
