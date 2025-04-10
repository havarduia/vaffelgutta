from waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

# TODO 
# placeholder = 1


def pick_up_waffle(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)
    
    bot.move("waffle_serve_station")
    actions.serve_waffle()
    bot.move("waffle_serve_station")
    state.set(State.SERVE_WAFFLE)
    


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
