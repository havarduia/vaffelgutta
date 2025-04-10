from waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

def open_iron2(state: "CurrentState", bot: "Wafflebot"):
    
    # TODO finish logic

    actions = Actions(bot)
    reader = Jsonreader()

    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    bot.move("sticks")
    actions.put_away_sticks()
    bot.move("front_of_waffle_iron")
    state.set(State.PICK_UP_WAFFLE)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
