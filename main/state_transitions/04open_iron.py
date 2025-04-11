from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State
from robot.robot_controllers.movements.action_header import Actions
from main.tag_enum.tags import Tags

def open_iron(state: "CurrentState", bot: "Wafflebot", tag: "CurrentTag"):
    
    actions = Actions(bot)
    reader = Jsonreader()
    
    reader.pop("camera_readings", Tags.SPRAY_TAG)
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    if Tags.SPRAY_TAG in tags.keys(): # tag 3 is the lube
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
    from main.tag_enum.tags import CurrentTag
