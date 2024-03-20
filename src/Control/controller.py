from typing import Any, Sequence
import numpy as np
import casadi as ca
import cvxpy as cp
from scipy.optimize import minimize, LinearConstraint
import sys
import random

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.robot import Robot
from Models.splines import Spline
from Utils.state import State
from Utils.action import Action
from Utils.trajectory import Trajectory


class MPCController:
    def __init__(self, car: Robot, trajectories: list[Trajectory]):
        self.car = car
        self.pred_horizon = 10
        self.contr_horizon = 5
        self.timestep = 0.1
        self.state = car.state
        self.trajectory1, self.trajectory2 = trajectories
        self.lap = 1

    def run(self, state):
        # Run the controller
        # INPUT: None
        # OUTPUT: An array of control actions
        # self.state = self.localise(self.state)
        self.state = state
        ref_states = self.getRefStates(self.state)

        horizon = min(self.contr_horizon, len(ref_states))

        steerings = ca.MX.sym("steerings", horizon)
        accelerations = ca.MX.sym("accelerations", horizon)

        low_bounds = [-self.car.steering_lim] * 5 + [-2 * self.car.max_acc] * 5
        high_bounds = [self.car.steering_lim] * 5 + [self.car.max_acc] * 5

        actions = [
            Action(steerings[i], accelerations[i], self.car) for i in range(horizon)
        ]
        cost = self.cost(actions, ref_states)

        nlp = {"x": ca.vertcat(steerings, accelerations), "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp)

        sol = solver(
            lbx=low_bounds,
            ubx=high_bounds,
        )

        steering_values = ca.vertsplit(sol["x"][:horizon])
        acceleration_values = ca.vertsplit(sol["x"][horizon:])

        actions = [
            Action(steering_values[i], acceleration_values[i], self.car)
            for i in range(horizon)
        ]

        print(self.cost(actions, ref_states))

        return actions

    def cost(self, actions: list[Action], ref_states: list[State]) -> float:
        # Cost Function to be minimised
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a cost value
        states = self.model(actions)
        cost = 0
        for i in range(1, len(states)):
            ref_states[i].pretty_print()
            cost += (
                (states[i].x - ref_states[i].x) ** 2
                + (states[i].y - ref_states[i].y) ** 2
                + 0.25 * (states[i].theta - ref_states[i].theta) ** 2
                + (states[i].v - ref_states[i].v) ** 2
                + (states[i].w - ref_states[i].w) ** 2
                + 0.25 * (states[i].v - states[i - 1].v) ** 2
                + 0.25 * (states[i].w - states[i - 1].w) ** 2
            )
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

        if self.lap == 1:
            trajectory = self.trajectory1
        else:
            trajectory = self.trajectory2

        dists = [point.distance(state) for point in trajectory.states]
        closest = np.argmin(dists)

        if closest + self.pred_horizon > trajectory.length:
            self.lap += 1  # LAP CHANGE DETERMINATION NEEDS FIXING
            return (
                trajectory[closest:]
                + self.trajectory2[: (closest + self.pred_horizon - trajectory.length)]
            )

        # Make a trajectory out of these states and save it to reference_0.json
        trajectory = Trajectory(
            trajectory.states[closest : closest + self.pred_horizon],
            [Action(0, 0, self.car)] * self.pred_horizon,
        )
        trajectory.write_to_json("reference_0.json")

        return trajectory.states[closest : closest + self.pred_horizon]

    def localise(self, prev_state) -> State:
        pass


if __name__ == "__main__":
    # Load in trajectories 0 and 1 from the trajectory json files
    car = Robot()
    trajectory1 = Trajectory.from_json("trajectory_0.json", car)
    trajectory2 = Trajectory.from_json("trajectory_1.json", car)

    # Create a controller
    controller = MPCController(car, [trajectory1, trajectory2])

    # generate fake localisation state (trajectory state with some added noise)
    state = trajectory1.states[0]
    state.x += 0.1 * random.random()
    state.y += 0.1 * random.random()
    state.theta += 0.1 * random.random()

    # Run the controller and save the output to control_0.json
    actions = controller.run(state)
    trajectory = Trajectory([state] * len(actions), actions)
    trajectory.write_to_json("control_0.json")
