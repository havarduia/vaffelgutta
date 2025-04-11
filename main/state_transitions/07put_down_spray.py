from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions
from main.tag_enum.tags import Tags

def put_down_spray(state: "CurrentState", bot: "Wafflebot", tag: "CurrentTag"):
    actions = Actions(bot)
    reader = Jsonreader()
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if Tags.LADLE_TAG in tags.keys(): # tag id 4 is ladle
        try:
            bot.move("front_of_bowl") #make it go to tag that is closer
            bot.move("front_of_ladle")
            actions.pick_up_ladle() # rename to pick_up_ladle
            bot.move("front_of_waffleiron")
            state.set(State.CUP_TO_IRON)
        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return
    else:
        print("Ladle is not in camera_readings")
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
    from main.tag_enum.tags import CurrentTag
