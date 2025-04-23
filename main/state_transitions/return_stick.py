from main.waffle_states.waffle_states import State, Tags
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

# TODO 
# placeholder = 1

def return_stick(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):
    # TODO finish logic
    camera_1 = vision.add_camera(name="cam1")
    reader = Jsonreader()
    
    
    reader.clear("camera_readings")
    vision.cam1.run_once(show_image=False, detect_markers=True, detect_hands=False)
    tags = reader.read("camera_readings")
    if Tags.IRON_TAG in tags.keys():
        state.set(State.HOME)
    elif Tags.IRON_TAG not in tags.keys():
        state.set(State.OPEN_IRON)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
