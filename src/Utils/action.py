import numpy as np
from Models.robot import Robot
from Utils.state import State


class Action:
    def __init__(self, steering, acceleration, car: Robot, timestep= 0.1):
        self.steering = steering
        self.acceleration = acceleration
        self.timestep = timestep
        self.car = car

    def apply(self, state: "State") -> "State":
        # apply the action to the state to get a new state
        v_old = state.v
        w_old = state.w
        v_new = v_old + self.acceleration
        w_new = (v_new/self.car.wheelbase) * np.tan(self.steering)
        w_acc = (w_new - w_old) * state.timestep
        delta_theta = ((w_old + w_new)/2) * state.timestep
        theta_new = (state.theta + delta_theta) % (2*np.pi)
        dx, dy = self.integrate(v_new, v_old, state.theta, w_old, w_acc)

        x_new = state.x + dx
        y_new = state.y + dy

        return State(x_new, y_new, theta_new, v_new, w_new)

    
    def integrate(self, v, u, theta, w, w_a, steps = 20): # approximate integration
        dx = 0
        dy = 0
        for i in range(steps):
            t = (2*i+1)
            inst_v = t/(2 * steps) * v + (2 * steps - t)/(2 * steps) * u
            inst_theta = (theta + w * t + (w_a * t**2) / 2) % (2 * np.pi)

            dx += inst_v * np.cos(inst_theta) * self.timestep/steps
            dy += inst_v * np.sin(inst_theta) * self.timestep/steps
        return dx, dy
