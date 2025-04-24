from main.waffle_states.waffle_states import State, Tags
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

# TODO
# placeholder = 1

def return_stick(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    # TODO finish logic
    reader = Jsonreader()


    reader.clear("camera_readings")
    vision.run_once(show_image=False, detect_hands=False)
    tags = reader.read("camera_readings")

    # Check for iron tag (both as string and integer)
    iron_tag_value = Tags.IRON_TAG.value
    iron_tag_present = iron_tag_value in tags.keys() or int(iron_tag_value) in tags.keys()

    if iron_tag_present:
        state.set(State.HOME)
    else:
        state.set(State.OPEN_IRON)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
