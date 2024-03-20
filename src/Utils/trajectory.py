from Utils.action import Action
from Utils.state import State
import json


# The Trajectory of a single lap
class Trajectory:
    def __init__(self, states: list[State], actions: list[Action]):
        assert len(states) == len(actions)
        self.states = states
        self.actions = actions
        self.length = len(states)
        self.timestep = states[0].timestep

    def to_dict(self):
        states = [state.to_dict() for state in self.states]
        actions = [action.to_dict() for action in self.actions]

        return {
            "states": states,
            "actions": actions,
            "length": self.length,
            "timestep": self.timestep,
        }

    def write_to_json(self, filename: str):
        # Write the trajectory to a json file
        # INPUT: filename
        # OUTPUT: None
        data = self.to_dict()
        with open(filename, "w") as file:
            json.dump(data, file)

    def plot_states(self):
        # Plot the states of the trajectory
        # INPUT: None
        # OUTPUT: None
        import matplotlib.pyplot as plt

        x = [state.x for state in self.states]
        y = [state.y for state in self.states]
        plt.plot(x, y)
        plt.show()

    @classmethod
    def from_json(cls, json_file, car) -> "Trajectory":
        # Load a trajectory from a json file
        # INPUT: filename
        # OUTPUT: Trajectory
        with open(json_file, "r") as file:
            data = json.load(file)
        states = [State(**state) for state in data["states"]]
        actions = [Action(car=car, **action) for action in data["actions"]]
        return Trajectory(states, actions)
