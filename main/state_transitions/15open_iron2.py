from timmy import Timmydetector
from robot.tools.file_manipulation import Jsonreader

def open_iron2(state: "State", bot: "Wafflebot"):
    
    timmy_alarm = Timmydetector()
    reader = Jsonreader()

    bot.camera_start()
    tags = reader.read("camera_readings")
    
    
    if timmy_alarm == False:        
        bot.move(waffle_sticks)
        bot.move(waffle_stick_out)
        bot.move(iron)
        state.set(State.PICK_UP_WAFFLE)
        
    
    if placeholder == 1:
        state.set(State.PICK_UP_WAFFLE)
    elif placeholder == 2:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
