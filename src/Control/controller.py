from typing import Any, Sequence
import numpy as np
import casadi as ca
import cvxpy as cp
import sys

sys.path.insert(1,"/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.robot import Robot
from Models.splines import Spline
from Utils.state import State
from Utils.action import Action



class MPCController:
    def __init__(self, car: Robot, trajectory: Spline):
        self.car = car
        self.pred_horizon = 10
        self.contr_horizon = 5
        self.timestep = 0.1
        self.state = State()

    def run(self):
        # Run the controller
        # INPUT: None
        # OUTPUT: An array of control actions
        self.state = self.localise(self.state)
        ref_states = self.getRefStates(self.state)

        steerings = cp.Variable(self.contr_horizon)
        accelerations = cp.Variable(self.contr_horizon)

        def find_actions(steerings, accelerations):
            actions = [Action(steerings[i], accelerations[i], self.car) for i in range(self.contr_horizon)]
            return self.cost(actions, ref_states)
        
        objective = cp.Minimize(find_actions(steerings, accelerations))
        constraints = [steerings <= self.car.steering_lim, steerings >= -self.car.steering_lim, accelerations <= self.car.max_acc, accelerations >= -self.car.max_acc]

        prob = cp.Problem(objective, constraints)

        prob.solve()

        actions = [Action(steerings.value[i], accelerations.value[i], self.car) for i in range(self.contr_horizon)]
        return actions


    def cost(self, actions: list[Action], ref_states: list[State]) -> float:
        # Cost Function to be minimised
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a cost value
        states = self.model(actions)
        cost = 0
        for i in range(1,len(states)):
            cost += states[i].distance(ref_states[i]) ** 2 + (0.25 * abs(states[i].theta - states[i-1].theta)) ** 2
        return cost

    def model(self, actions: list[Action]) -> list[State]:
        # Car Dynamics Model
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a series of predicted states (x, y, theta)
        states = []
        cur_state = self.state
        states.append(cur_state)
        for action in actions:
            state = action.apply(cur_state)
            states.append(state)
            cur_state = state
        return states

    def getRefStates(self, state: State) -> list[State]:
        # Get reference states from the trajectory by calculating the closest state and returning the next n states (n is pred_horizon)
        # INPUT: current state
        # OUTPUT: a series of reference states

        # Potentially want to edit this function to find states in front of the car
        dists = [point.distance(state) for point in self.trajectory]
        closest = np.argmin(dists)
        if closest + self.pred_horizon > len(self.trajectory):
            return self.trajectory[closest:] + self.trajectory[:(closest+self.pred_horizon-len(self.trajectory))]
        return self.trajectory[closest:closest+self.pred_horizon]

        

    def localise(self, prev_state) -> State:
        pass
