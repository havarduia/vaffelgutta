"""
This script outlines main()

It should however be viewed as a template and not funtional
"""

from camera.vision import Vision
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from main.waffle_states.waffle_states import State, CurrentState
from main.state_transitions import *


def init():
    # Inits the camera systems and inserts the instance to Wafflebot.
    vision = Vision()
    bot = Wafflebot(automatic_mode=True, detect_collisions=False)
    userstate_input = input("Input the start state: \n")
    try:
        # Try to match the input to a state enum
        userstate_input = userstate_input.upper()
        # If the input is a function name (like 'sleepstate'), try to match it to the corresponding state
        if userstate_input == 'SLEEPSTATE':
            userstate = State.SLEEP
        elif userstate_input == 'RESTSTATE':
            userstate = State.REST
        elif userstate_input == 'HOMESTATE':
            userstate = State.HOME
        else:
            # Otherwise, try to match it directly to a state enum
            userstate = State[userstate_input]
        state = CurrentState(userstate)
    except KeyError:
        print(f"State '{userstate_input}' not found. Defaulting to SLEEP state.")
        state = CurrentState(State.SLEEP)
    return bot, vision, state


def main():
    """
    Executes the next action in the actions list.
    """
    bot, vision, state = init()
    while True:
        current_state = state.get()
        match current_state:

            case State.REST:
                print("I am resting, dammit. These youngins...")
                rest(state, bot)

            case State.HOME:
                print("Im HOMING AHHH! 👺")
                home(state, bot, vision)

            case State.SLEEP:
                print("Sleeping 😇")
                sleepstate(state, bot)

            case State.OPEN_IRON:
                print("Opening iron!")
                open_iron(state, bot, vision)

            case State.PICK_UP_SPRAY:
                print("Picking up spray 👺")
                pick_up_spray(state, bot, vision)

            case State.SPRAY:
                print("Im spraying AHHHH!")
                spray(state, bot)

            case State.PUT_DOWN_SPRAY:
                print("Sorry i dont have anything left to spray...")
                put_down_spray(state, bot, vision)

            case State.CLOSE_IRON:
                print("Its all going well, closing iron! 👺")
                close_iron(state, bot)

            case State.FUN_TIME:
                print("This is where the fun begins 😏")
                fun_time(state, bot)

            case State.OPEN_IRON2:
                print("Waffle is done opening iron! 🥴")
                open_iron2(state, bot)

            case State.RETURN_STICK:
                print("Stick to IRON!!!")
                return_stick(state, bot, vision)

            case State.ERROR:
                print("Hagle")
                error(state, bot)

            case _:
                print("An unknown state was encountered!")
                bot.safe_stop(slow=True)
                break

    bot.safe_stop()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)
