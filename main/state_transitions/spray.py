from waffle_states.waffle_states import State
from robot.robot_controllers.movements.action_header import Actions


def spray(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)

    try:
        bot.move("front_of_tool_station")
        actions.put_down_lube()
        bot.move("front_of_tool_station")
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return
    state.set(State.PUT_DOWN_SPRAY)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
