from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.waffle_states.waffle_states import State, Tags
from camera.vision import Vision


def home(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):

    actions = Actions(bot)
    reader = Jsonreader()
    camera_1 = vision.add_camera(name="cam1")

    # Clear any existing tag data
    reader.clear("camera_readings")
    # Run camera to detect markers
    vision.cam1.run_once(return_image=False, detect_hands=False, detect_markers=True)
    tags = reader.read("camera_readings")

    # Check for iron tag (both as string and integer)
    iron_tag_value = Tags.IRON_TAG.value
    if iron_tag_value in tags.keys() or int(iron_tag_value) in tags.keys():
        try:
            bot.move("waffle_iron")
            actions.open_waffle_iron()
            bot.move("waffle_iron")
        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return

    state.set(State.OPEN_IRON)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
