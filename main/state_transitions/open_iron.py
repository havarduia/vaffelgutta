from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision

def open_iron(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):

    actions = Actions(bot)
    reader = Jsonreader()
    camera_1 = vision.add_camera(name="cam1")

    # Clear any existing tag data
    reader.clear("camera_readings")
    # Run camera to detect markers
    vision.cam1.run_once(return_image=False, detect_hands=False, detect_markers=True)
    tags = reader.read("camera_readings")

    # Check for spray tag (both as string and integer)
    spray_tag_value = Tags.SPRAY_TAG.value
    if spray_tag_value in tags.keys() or int(spray_tag_value) in tags.keys(): # tag 3 is the lube
        try:
            bot.move("front_of_tool_station")
            actions.pick_up_lube()
            state.set(State.PICK_UP_SPRAY)
        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return
    else:
        state.set(State.HOME)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
