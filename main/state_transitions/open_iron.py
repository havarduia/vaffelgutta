from robot.tools.file_manipulation import Jsonreader
from ai.timmy_detector import Timmydetector()

def open_iron(state: "State", bot: "Wafflebot"):
    
    timmy_alarm = Timmydetector()
    reader = Jsonreader()
    
    reader.pop("camera_readings", "2")
    bot.camera_start()
    tags = reader.read("camera_readings")
    
    if timmy_alarm == False:
        if 2 in tags.keys():
            iron_open = True
            
        if iron_open == True:
            bot.move(tool_station)
            bot.sequence(pick_up_spray_bottle) 
            state.set(State.PICK_UP_SPRAY)  
        else:
            state.set(State.HOME)
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
