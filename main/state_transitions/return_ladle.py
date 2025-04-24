from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.waffle_states.waffle_states import State, Tags
from camera.vision import Vision

def return_ladle(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    reader = Jsonreader()
    actions = Actions(bot)

    # Clear any existing tag data
    reader.clear("camera_readings")
    # Run camera to detect markers
    vision.run_once(return_image=False, detect_hands=False)
    tags = reader.read("camera_readings")

    # Check for iron tag (both as string and integer)
    iron_tag_value = Tags.IRON_TAG.value
    iron_tag_present = iron_tag_value in tags.keys() or int(iron_tag_value) in tags.keys()

    try:
        if not iron_tag_present:
            bot.move("front_of_waffle_iron")
            actions.close_waffle_iron()
            bot.move("front_of_waffle_iron")
            state.set(State.CLOSE_IRON)
        elif iron_tag_present:
            state.set(State.CLOSE_IRON)
        else:
            state.set(State.ERROR)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return




if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
