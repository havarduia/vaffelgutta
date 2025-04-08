from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from waffle_states.waffle_states import State

def open_iron(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)

    reader = Jsonreader()
    reader.pop("camera_readings", "1")
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if "1" in tags.keys():
        bot.move("waffle_iron")
        actions.open_waffle_iron() # Todo implenment this as a movement sequence
        state.set(State.OPEN_IRON)

    else:
        state.set(State.OPEN_IRON)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState
