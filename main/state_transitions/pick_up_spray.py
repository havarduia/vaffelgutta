from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

def pick_up_spray(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):

    actions = Actions(bot)
    reader = Jsonreader()
    camera_1 = vision.add_camera(name="cam1")

    reader.pop("camera_readings", Tags.IRON_TAG) # tag id 2 is opened iron
    vision.cam1.run_once(return_image=False, detect_hands=False, detect_markers=True)
    tags = reader.read("camera_readings")

    if Tags.IRON_TAG not in tags.keys(): # If not there, assume it is open.
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
