import matplotlib.pyplot as plt
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

        x = [state.x for state in self.states]
        y = [state.y for state in self.states]
        plt.plot(x, y)
        plt.show()

    def plot_actions(self):
        cur_state = self.states[0]
        x = [cur_state.x]
        y = [cur_state.y]
        for action in self.actions:
            new_state = action.apply(cur_state)
            x.append(new_state.x)
            y.append(new_state.y)
            cur_state = new_state
        plt.plot(x, y)
        plt.show()

    def plot_states_and_actions(self):
        cur_state = self.states[0]
        x = [cur_state.x]
        y = [cur_state.y]
        for action in self.actions:
            new_state = action.apply(cur_state)
            x.append(new_state.x)
            y.append(new_state.y)
            cur_state = new_state

        x2 = [state.x for state in self.states]
        y2 = [state.y for state in self.states]
        x2.append(self.states[0].x)
        y2.append(self.states[0].y)

        plt.plot(x2, y2)
        plt.plot(x, y)
        plt.grid()
        plt.ylim(-1.5, 1.5)
        plt.xlim(-1.75, 1.75)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Parameterised States vs Modelled Actions")
        plt.legend(
            ["States", "Modelled Actions"],
            loc="center left",
            bbox_to_anchor=(1, 0.5),
        )
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
