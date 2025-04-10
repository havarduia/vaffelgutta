from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from waffle_states.waffle_states import State
#TODO: Rename accordingly
def return_cup(state: "CurrentState", bot: "Wafflebot"):
    reader = Jsonreader()
    actions = Actions(bot)
    reader.pop("camera_readings","2")
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    if "2" in tags.keys():     
        bot.move("front_of_waffle_iron")
        actions.close_waffle_iron()
        bot.move("front_of_waffle_iron")
        state.set(State.CLOSE_IRON)
    elif "1" in tags.keys():
        state.set(State.CLOSE_IRON) 
    else:
        state.set(State.ERROR)
    


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
