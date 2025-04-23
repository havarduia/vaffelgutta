from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from waffle_states.waffle_states import State, Tags
def return_ladle(state: "CurrentState", bot: "Wafflebot"):
    reader = Jsonreader()
    actions = Actions(bot)
    reader.pop("camera_readings",Tags.OPENED_IRON_TAG)
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    try:
        if Tags.IRON_TAG not in tags.keys():     
            bot.move("front_of_waffle_iron")
            actions.close_waffle_iron()
            bot.move("front_of_waffle_iron")
            state.set(State.CLOSE_IRON)
        elif Tags.IRON_TAG in tags.keys():
            state.set(State.CLOSE_IRON) 
        else:
            state.set(State.ERROR)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return
    
    


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
