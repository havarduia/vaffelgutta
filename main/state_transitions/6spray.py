from robot.tools.file_manipulation import Jsonreader
from ai.timmy_detector import Timmydetector


def spray(state: "State", bot: "Wafflebot"):

    timmy_alarm = Timmydetector()
    if timmy_alarm == False:

        bot.move(tool_station)
        bot.move(remove_cup)
        state.set(State.PUT_DOWN_SPRAY)
    else:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
