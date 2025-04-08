from robot.tools.file_manipulation import Jsonreader
from ai.timmy_detector import Timmydetector


def put_down_spray(state: "State", bot: "Wafflebot"):
    reader = Jsonreader()
    timmy_alarm = Timmydetector()
    bot.camera_start()
    tags = reader.read("camera_readings")

    if timmy_alarm == False:
        if 4 in tags.keys():
            bot.move(bottle)
            bot.move(pick_up_cup)
            bot.move(filling_station)
            bot.move(place_cup_filling_station)
            state.set(State.PLACE_CUP_TO_STAT)
        else:
            print("Bottle is not in camera_readings")
            state.set(State.ERROR)
    else:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
