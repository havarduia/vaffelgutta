from waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.tag_enum.tags import Tags

def open_iron2(state: "CurrentState", bot: "Wafflebot", tag: "CurrentTag"):
    
    # TODO finish logic

    actions = Actions(bot)
    reader = Jsonreader()

    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    if Tags.STICKS_TAG in tags.keys(): # Tag 6 is meant to be the sticks in the waffle iron.
        bot.move("sticks")
        actions.put_away_sticks() # Take out sticks from waffle iron with the waffle attached hopefully.
        bot.move("front_of_waffle_iron") # Here for changing state
        state.set(State.PICK_UP_WAFFLE)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
    from main.tag_enum.tags import CurrentTag
