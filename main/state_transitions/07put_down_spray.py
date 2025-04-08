from robot.tools.file_manipulation import Jsonreader
from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions


def put_down_spray(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)
    reader = Jsonreader()
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if "4" in tags.keys():
        bot.move("front_of_cup") 
        actions.pick_up_cup()
        bot.move("front_of_filling_station")
        actions.place_cup(True)
        state.set(State.PLACE_CUP_TO_STAT)
    else:
        print("Bottle is not in camera_readings")
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
