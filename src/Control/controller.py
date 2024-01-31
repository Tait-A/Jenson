import numpy as np
import casadi as ca
import sys
sys.path.insert(1,"/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.robot import Robot
from Models.splines import Spline



class MPCController:
    def __init__(self, car: Robot, trajectory: Spline):
        self.car = car
        self.pred_horizon = 10
        self.contr_horizon = 5
        self.timestep = 0.1
        self.localise()


    def cost(self, steerings, accelerations):
        # Cost Function to be minimised
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a cost value
        xs, ys, thetas = self.model(steerings, accelerations)
        pass

    def model(self, actions):
        # Car Dynamics Model
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a series of predicted states (x, y, theta)

        pass



    def localise(self):
        pass
        

    class State:
        def __init__(self, x, y, theta, v = 0, w = 0, timestep = 0.1):
            self.x = x
            self.y = y
            self.theta = theta
            self.v = v
            self.w = w
            self.timestep = timestep

        def new_state_local(self, x_new, y_new, theta_new) -> "MPCController.State":
            delta_theta = theta_new - self.theta
            w_new = delta_theta / self.timestep
            dist = np.sqrt((x_new - self.x)**2 + (y_new - self.y)**2)
            turn_radius = dist / (2 * np.sin(delta_theta/2))
            arc_length = turn_radius * delta_theta
            v_new = self.v + (2 * (arc_length - self.v * self.timestep)) / self.timestep
            return MPCController.State(x_new, y_new, theta_new, v_new, w_new)
    
        
    class Action:
        def __init__(self, steering, acceleration, car: Robot):
            self.steering = steering
            self.acceleration = acceleration
            self.car = car

        def apply(self, state: "MPCController.State") -> "MPCController.State":
            # apply the action to the state to get a new state
            v_old = state.v
            w_old = state.w
            v_new = v_old + self.acceleration
            w_new = (v_new/self.car.wheelbase) * np.tan(self.steering)
            w_acc = (w_new - w_old) * state.timestep
            delta_theta = ((w_old + w_new)/2) * state.timestep
            theta_new = state.theta + delta_theta
            dx, dy = self.integrate(v_new, v_old, state.theta, w_old, w_acc, 10)

            x_new = state.x + dx
            y_new = state.y + dy

            return MPCController.State(x_new, y_new, theta_new, v_new, w_new)

        
        def integrate(self, v, u, theta, w, w_a, steps): # approximate integration
            dx = 0
            dy = 0
            for i in range(steps):
                t = (2*i+1)

                inst_v = t/(2 * steps) * v + (2 * steps - t)/(2 * steps) * u
                inst_theta = theta + w * t + (w_a * t**2) / 2

                dx += inst_v * np.cos(inst_theta) * self.timestep/steps
                dy += inst_v * np.sin(inst_theta) * self.timestep/steps
            return dx, dy
            



