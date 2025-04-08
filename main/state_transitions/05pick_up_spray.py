from robot.tools.file_manipulation import Jsonreader
from waffle_states.waffle_states import State
from robot.robot_controllers.movements.action_header import Actions

def open_iron(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)
    reader = Jsonreader()

    reader.pop("camera_readings", "2")
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if "2" in tags.keys():
        bot.move("front_of_waffle_iron")
        actions.apply_lube()
        state.set(State.SPRAY)

    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
