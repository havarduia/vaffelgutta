from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from waffle_states.waffle_states import State
from main.tag_enum.tags import Tags
#TODO: Rename accordingly
def return_cup(state: "CurrentState", bot: "Wafflebot", tag: "CurrentTag"):
    reader = Jsonreader()
    actions = Actions(bot)
    reader.pop("camera_readings",Tags.OPENED_IRON_TAG)
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    if Tags.OPENED_IRON_TAG in tags.keys():     
        bot.move("front_of_waffle_iron")
        actions.close_waffle_iron()
        bot.move("front_of_waffle_iron")
        state.set(State.CLOSE_IRON)
    elif Tags.CLOSED_IRON_TAG in tags.keys():
        state.set(State.CLOSE_IRON) 
    else:
        state.set(State.ERROR)
    


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
    from main.tag_enum.tags import CurrentTag
