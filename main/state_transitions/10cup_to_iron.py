from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def cup_to_iron(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)
    try:
        actions.pour_batter()   
        state.set(State.EMPTY_CUP)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
