from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def cup_to_iron(state: "State", bot: "Wafflebot"):
    actions = Actions(bot)
    actions.pour_batter()   
    state.set(State.EMPTY_CUP)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
