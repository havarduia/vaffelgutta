from enum import Enum

class State(Enum):
    REST = 0,
    HOME = 1,
    SLEEP = 2,
    state1 = 3,
    state2 = 4,
    #...fill in states here


class CurrentState():
    def __init__(self):
        self.state = State.SLEEP

    def set(self, new_state: State) -> None:
        self.state = new_state
        
    def get(self) -> State:
        return self.state