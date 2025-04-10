from robot.tools.file_manipulation import Jsonreader
from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions


def put_down_spray(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)
    reader = Jsonreader()
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if "4" in tags.keys(): # tag id 4 is ladle
        bot.move("front_of_bowl") #make it go to tag that is closer
        bot.move("front_of_ladle")
        actions.pick_up_cup() # rename to pick_up_ladle
        bot.move("front_of_waffleiron")
        state.set(State.CUP_TO_IRON)
    else:
        print("Bottle is not in camera_readings")
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
