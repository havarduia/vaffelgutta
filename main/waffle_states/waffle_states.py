from enum import Enum


class State(Enum):
    REST = 1
    HOME = 2
    SLEEP = 3
    OPEN_IRON = 4
    PICK_UP_SPRAY = 5
    SPRAY = 6
    PUT_DOWN_SPRAY = 7
    PICK_UP_LADLE = 8
    POUR_BATTER = 9
    RETURN_LADLE = 10
    CLOSE_IRON = 11
    FUN_TIME = 12
    OPEN_IRON2 = 13
    RETURN_STICK = 14
    ERROR = 666
    # ...fill in states here


class CurrentState:
    def __init__(self, startstate: State):
        self.state = startstate

    def set(self, new_state: State) -> None:
        self.state = new_state

    def get(self) -> State:
        return self.state
    
class Tags(Enum):
    CLOSED_IRON_TAG = "1"
    OPENED_IRON_TAG = "2"
    SPRAY_TAG = "3"
    LADLE_TAG = "4"
    STICKS_TAG = "5"
    JAM_TAG = "6"
    FOLLOW_ME_TAG = "7"
    ORIGIN_TAG = "0"
    
    # ...fill in tags here

