from action import Action
from state import State

# The Trajectory of a single lap
class Trajectory:
    def __init__(self, states: list[State], actions: list[Action]):
        assert len(states) == len(actions)
        self.states = states
        self.actions = actions
        self.length = len(states)
        self.timestep = states[0].timestep