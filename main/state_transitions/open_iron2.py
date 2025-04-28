from rclpy import action
from main.waffle_states.waffle_states import State, Tags
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

def open_iron2(state: "CurrentState", bot: "Wafflebot"):
    
    # TODO finish logic

    actions = Actions(bot)
    
    try:
        #bot.move("sticks")
        #actions.take_out_waffle() # Take out sticks from waffle iron with the waffle attached hopefully.
        bot.move("front_of_waffle_iron") # Here for changing state
        state.set(State.PICK_UP_WAFFLE)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 