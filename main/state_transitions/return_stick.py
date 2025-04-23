from main.waffle_states.waffle_states import State, Tags
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

# TODO 
# placeholder = 1

def return_stick(state: "CurrentState", bot: "Wafflebot"):
    # TODO finish logic
    reader = Jsonreader()
    reader.clear("camera_readings")
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    if Tags.IRON_TAG in tags.keys():
        state.set(State.HOME)
    elif Tags.IRON_TAG not in tags.keys():
        state.set(State.OPEN_IRON)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
