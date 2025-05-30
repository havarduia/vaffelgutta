from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

def pick_up_spray(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):

    actions = Actions(bot)
    reader = Jsonreader()

    # Clear any existing tag data
    reader.clear("camera_readings")
    
    if bot.automatic_mode:
        vision.run_once()
        
    tags = reader.read("camera_readings")

    # Check for iron tag (both as string and integer)
    iron_tag_value = Tags.IRON_TAG.value
    iron_tag_present = iron_tag_value in tags.keys() or int(iron_tag_value) in tags.keys()

    if not iron_tag_present or not bot.automatic_mode: # If not there, assume it is open.
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
