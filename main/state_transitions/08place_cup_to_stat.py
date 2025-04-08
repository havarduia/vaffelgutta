
from waffle_states.waffle_states import State 

def place_cup_to_stat(state: "CurrentState", bot: "Wafflebot"):
    try:
        fill_cup() # TODO this function needs a spark of life, #maspåAleksander.
    except NameError:
        state.set(State.ERROR)
        raise NotImplementedError("The filling hardware does not exist. Or if it does,\nThe filling software does not exist.")
    
    state.set(State.FILL_CUP)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
