placeholder = 1

def place_cup_to_stat(state: "State", bot: "Wafflebot"):
    if placeholder == 1:
        state.set(State.FILL_CUP)
    elif placeholder == 2:
        state.set(State.ERROR)
    

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
