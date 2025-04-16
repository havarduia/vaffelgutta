from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions

def open_iron(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)
    reader = Jsonreader()

    reader.pop("camera_readings", Tags.OPENED_IRON_TAG) # tag id 2 is opened iron
    bot.cam.start("all")
    tags = reader.read("camera_readings")

    if Tags.OPENED_IRON_TAG in tags.keys():
        try:
            actions.spray_lube() 
            state.set(State.SPRAY)
        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 