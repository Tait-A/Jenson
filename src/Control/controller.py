from typing import Any, Sequence
import numpy as np
import casadi as ca
import cvxpy as cp
from scipy.optimize import minimize, LinearConstraint
import sys
import random
import os
import matplotlib.pyplot as plt

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.robot import Robot
from Utils.splines import Spline
from Utils.state import State
from Utils.action import Action
from Utils.trajectory import Trajectory
import config


class MPCController:
    def __init__(self, car: Robot, trajectories: list[Trajectory]):
        self.car = car
        self.pred_horizon = 10
        self.contr_horizon = 5
        self.timestep = 0.1
        self.state = car.state
        self.trajectory1, self.trajectory2 = trajectories
        self.lap = 2
        self.i = 0

    def run(self, state):
        # Run the controller
        # INPUT: None
        # OUTPUT: An array of control actions
        # self.state = self.localise(self.state)
        self.state = state
        ref_states, ref_actions = self.getRefStates(self.state)

        horizon = min(self.contr_horizon, len(ref_states))

        steerings = ca.MX.sym("steerings", horizon)
        accelerations = ca.MX.sym("accelerations", horizon)

        low_bounds = [-self.car.steering_lim] * 5 + [-2 * self.car.max_acc] * 5
        high_bounds = [self.car.steering_lim] * 5 + [self.car.max_acc] * 5

        actions = [
            Action(steerings[i], accelerations[i], self.car) for i in range(horizon)
        ] + ref_actions[horizon : self.pred_horizon]
        cost = self.cost(actions, ref_states)

        steering_guess = [action.steering for action in ref_actions[:horizon]]
        steering_guess[0] -= 0.1
        acceleration_guess = [action.acceleration for action in ref_actions[:horizon]]
        guesses = steering_guess + acceleration_guess

        nlp = {"x": ca.vertcat(steerings, accelerations), "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp)

        sol = solver(
            lbx=low_bounds,
            ubx=high_bounds,
            x0=guesses,
        )

        steering_values = ca.vertsplit(sol["x"][:horizon])
        acceleration_values = ca.vertsplit(sol["x"][horizon:])

        actions = [
            Action(float(steering_values[i]), float(acceleration_values[i]), self.car)
            for i in range(horizon)
        ]

        actions += ref_actions[horizon : self.pred_horizon]

        states = self.model(actions)

        print(f"Cost: {self.cost(actions, ref_states)}")

        self.i += 1
        return actions, states

    def cost(self, actions: list[Action], ref_states: list[State]) -> float:
        # Cost Function to be minimised
        # INPUT: a series of potential control actions (delta_phi and acceleration)
        # OUTPUT: a cost value
        states = self.model(actions)
        cost = 0
        print(len(states), len(ref_states))
        for i in range(1, len(states)):
            factor = 1 + 0.1 * i
            theta_diff = ca.fabs(
                ca.fmod((states[i].theta - ref_states[i].theta + np.pi), (2 * np.pi))
                - np.pi
            )

            cost += (
                factor * (states[i].x - ref_states[i].x) ** 2
                + factor * (states[i].y - ref_states[i].y) ** 2
                + factor * 0.25 * (theta_diff) ** 2
                + 0.1 * (states[i].v - ref_states[i].v) ** 2
                # + 0.25 * (states[i].w - ref_states[i].w) ** 2
                # + 0.25 * (states[i].v - states[i - 1].v) ** 2
                # + 0.25 * (states[i].w - states[i - 1].w) ** 2
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

        print("Closest: ", closest)
        print("Length: ", trajectory.length)

        if closest + self.pred_horizon >= trajectory.length:
            self.lap += 1  # LAP CHANGE DETERMINATION NEEDS FIXING
            trajectory = Trajectory(
                trajectory.states[closest:]
                + self.trajectory2.states[
                    : (closest + self.pred_horizon - trajectory.length + 1)
                ],
                trajectory.actions[closest:]
                + self.trajectory2.actions[
                    : (closest + self.pred_horizon - trajectory.length + 1)
                ],
            )
        else:
            # Make a trajectory out of these states and save it to reference_0.json
            trajectory = Trajectory(
                trajectory.states[closest : closest + self.pred_horizon + 1],
                trajectory.actions[closest : closest + self.pred_horizon + 1],
            )

        trajectory.write_to_json("reference_0.json")

        print("Current Lap: ", self.lap)

        # trajectory.plot_states()
        path = os.path.join(config.SRC_PATH, "JSON")
        trajectory.write_to_json(path + f"/reference_{self.i}.json")

        return trajectory.states, trajectory.actions

    def localise(self, prev_state) -> State:
        pass


### TESTING ###


def generate_loc_states(controller: MPCController, n: int, noise: float) -> list[State]:
    # Generate n states by choosing a random state from the trajectory and adding some noise
    # INPUT: controller
    # OUTPUT: a list of n states
    loc_states = []
    for i in range(n):
        old_state = controller.trajectory2.states[(12 * i) + 3 + random.randint(0, 5)]
        state = old_state.copy()
        state.x += 2 * noise * random.uniform(-1, 1)
        state.y += 2 * noise * random.uniform(-1, 1)
        state.theta += noise * random.uniform(-1, 1)
        loc_states.append(state)
    return loc_states


def test_controller(controller: MPCController):

    loc_states = generate_loc_states(controller, 4, 0.1)

    # Run the controller and save the output to control_0.json
    for i, state in enumerate(loc_states):
        actions, states = controller.run(state)
        trajectory = Trajectory(states[: len(actions)], actions)
        path = os.path.join(config.SRC_PATH, "JSON")
        trajectory.write_to_json(path + f"/control_{i}.json")


def plot_tests(controller: MPCController):
    # Plot the controller trajectory
    traj_x = [state.x for state in controller.trajectory2.states]
    traj_y = [state.y for state in controller.trajectory2.states]
    traj_x.append(traj_x[0])
    traj_y.append(traj_y[0])

    base = plt.plot(traj_x, traj_y, label="Base Trajectory")

    path = os.path.join(config.SRC_PATH, "JSON")

    for i in range(4):
        control = Trajectory.from_json(path + f"/control_{i}.json", controller.car)
        traj_x = [state.x for state in control.states]
        traj_y = [state.y for state in control.states]
        control_line = plt.plot(traj_x, traj_y, label="Control", color="orange")

        ref = Trajectory.from_json(path + f"/reference_{i}.json", controller.car)
        cur_state = control.states[0]
        x = [cur_state.x]
        y = [cur_state.y]
        for j in range(len(control.states) - 1):
            new_state = ref.actions[j].apply(cur_state)
            x.append(new_state.x)
            y.append(new_state.y)
            cur_state = new_state
        reference = plt.plot(x, y, label="Reference", color="red")

    plt.legend(
        [base[0], control_line[0], reference[0]],
        ["Base", "Control", "Reference"],
        loc="center left",
        bbox_to_anchor=(1, 0.5),
    )
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Control vs Reference Trajectory Comparison")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    # Load in trajectories 0 and 1 from the trajectory json files
    car = Robot()
    trajectory1 = Trajectory.from_json(config.SRC_PATH + "/JSON/trajectory_0.json", car)
    trajectory2 = Trajectory.from_json(config.SRC_PATH + "/JSON/trajectory_1.json", car)

    # Create a controller
    controller = MPCController(car, [trajectory1, trajectory2])

    # Test the controller
    # test_controller(controller)

    plot_tests(controller)
