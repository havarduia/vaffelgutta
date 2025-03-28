from enum import Enum


class State(Enum):
    REST = 1
    HOME = 2
    SLEEP = 3
    OPEN_IRON = 4 
    PICK_UP_SPRAY = 5
    SPRAY = 6
    PUT_DOWN_SPRAY = 7
    PLACE_CUP_TO_STAT = 8
    FILL_CUP = 9
    CUP_TO_IRON = 10
    EMPTY_CUP = 11
    RETURN_CUP = 12
    CLOSE_IRON = 13
    FUN_TIME = 14
    OPEN_IRON2 = 15
    PICK_UP_WAFFLE = 16
    SERVE_WAFFLE = 17
    RETURN_STICK = 18
    ERROR = 666
    #...fill in states here


class CurrentState:
    def __init__(self):
        self.state = State.SLEEP

    def set(self, new_state: State) -> None:
        self.state = new_state
        
    def get(self) -> State:
        return self.state
