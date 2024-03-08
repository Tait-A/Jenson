import numpy as np

class State:
    def __init__(self, x, y, theta, v = 0, w = 0, timestep = 0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.timestep = timestep

    def new_state_local(self, x_new, y_new, theta_new) -> "State": #Not used anywhere
        delta_theta = theta_new - self.theta # NEEDS WRAPAROUND FIXING
        w_new = delta_theta / self.timestep
        dist = np.sqrt((x_new - self.x)**2 + (y_new - self.y)**2)
        turn_radius = dist / (2 * np.sin(delta_theta/2))
        arc_length = turn_radius * delta_theta
        v_new = self.v + (2 * (arc_length - self.v * self.timestep)) / self.timestep
        return State(x_new, y_new, theta_new, v_new, w_new)
    
    def distance(self, state: "State") -> float:
        return np.sqrt((self.x - state.x)**2 + (self.y - state.y) ** 2) + 0.25 * abs(self.theta - state.theta)
