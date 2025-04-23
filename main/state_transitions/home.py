from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.waffle_states.waffle_states import State, Tags


def home(state: "CurrentState", bot: "Wafflebot"):
    
    actions = Actions(bot)

    reader = Jsonreader()
    reader.pop("camera_readings", Tags.IRON_TAG)
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    if Tags.IRON_TAG in tags.keys():
        try:
            bot.move("waffle_iron")
            actions.open_waffle_iron() 
            bot.move("waffle_iron")
        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return

    state.set(State.OPEN_IRON)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
