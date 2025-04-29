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
        elif userstate_input == 'OPENIRON':
            userstate = State.OPEN_IRON
        else:
            # Otherwise, try to match it directly to a state enum
            userstate = State[userstate_input]
        state = CurrentState(userstate)
    except KeyError:
        print(f"State '{userstate_input}' not found. Defaulting to SLEEP state.")
        state = CurrentState(State.SLEEP)
    return vision, state


def main(bot: Wafflebot):
    """
    Executes the next action in the actions list.
    """
    vision, state = init()
    while True:
        current_state = state.get()
        match current_state:

            case State.REST:
                print("I am resting, dammit. These youngins...")
                rest(state, bot)

            case State.HOME:
                print("Opening IRON!!")
                home(state, bot, vision)

            case State.SLEEP:
                print("I am risin up")
                sleepstate(state, bot)

            case State.OPEN_IRON:
                print("Picking up spray 👺")
                open_iron(state, bot, vision)

            case State.PICK_UP_SPRAY:
                print("Im spraying AHHHH!")
                pick_up_spray(state, bot, vision)

            case State.SPRAY:
                print("Sorry i dont have anything left to spray...")
                spray(state, bot)

            case State.PUT_DOWN_SPRAY:
                print("IM PICKING UP THE LADLE AHSAHASDHHAHHHHH")
                put_down_spray(state, bot, vision)
                
            case State.PICK_UP_LADLE:
                print("IM POURING AHHHH")
                pick_up_ladle(state, bot)

            case State.CLOSE_IRON:
                print("Its all going well, closing iron! 👺")
                close_iron(state, bot)

            case State.FUN_TIME:
                print("This is where the fun begins 😏")
                fun_time(state, bot)
            
            case State.POUR_BATTER:
                print("IM RETURNING LADLE AHHH")
                pour_batter(state, bot)
                
            case State.RETURN_LADLE:
                print("I HAVE RETURNED THE LADLE")
                return_ladle(state, bot, vision)

            case State.OPEN_IRON2:
                print("Waffle is done opening iron! 🥴")
                open_iron2(state, bot)

            case State.RETURN_STICK:
                print("Stick to IRON!!!")
                return_stick(state, bot, vision)
                
            case State.PICK_UP_WAFFLE:
                print("123")
                pick_up_waffle(state,bot,vision)

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
        bot = Wafflebot(automatic_mode=False, detect_collisions=True)
        main(bot)
    except Exception as e:
        handle_error(e)
