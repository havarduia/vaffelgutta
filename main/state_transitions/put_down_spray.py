from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

def put_down_spray(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    actions = Actions(bot)
    reader = Jsonreader()

    # Clear any existing tag data
    reader.clear("camera_readings")

    if bot.automatic_mode:
        vision.run_once()
        
    tags = reader.read("camera_readings")

    # Check for ladle tag (both as string and integer)
    ladle_tag_value = Tags.LADLE_TAG.value
    ladle_tag_present = ladle_tag_value in tags.keys() or int(ladle_tag_value) in tags.keys()

    if ladle_tag_present or not bot.automatic_mode: # tag id 1 is ladle
        try:
            bot.move("prep_ladle")
            actions.pick_up_ladle()
            bot.move("prep_ladle")
            state.set(State.PICK_UP_LADLE)
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
