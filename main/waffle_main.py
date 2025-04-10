"""
This script outlines main()

It should however be viewed as a template and not funtional
"""

from camera.init_camera import initalize_system as init_camera
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot import Wafflebot
from main.waffle_states.waffle_states import State, CurrentState
from main.state_transitions import *


def init():
    # Inits the camera systems and inserts the instance to Wafflebot.
    camera, aruco, coordsys = init_camera()
    bot = Wafflebot(coordsys)
    userstate_input = input("Input the start state: \n")
    try:
        userstate = State[userstate_input.upper()]
        state = CurrentState(userstate)
    except NameError:
        state = CurrentState(State.REST)
    return bot, camera, aruco, coordsys, state


def main():
    """
    Executes the next action in the actions list.
    """
    bot, camera, aruco, coordsys, state = init()
    current_state = state.get()
    while True:
        
        match current_state:

            case State.REST:
                print("I am resting, dammit. These youngins...")
                rest(state, bot)

            case State.HOME:
                print("Im HOMING AHHH! 👺")
                home(state, bot)

            case State.SLEEP:
                print("Sleeping 😇")
                sleepstate(state, bot)

            case State.OPEN_IRON:
                print("Opening iron!")
                open_iron(state, bot)

            case State.PICK_UP_SPRAY:
                print("Picking up spray 👺")
                pick_up_spray(state, bot)

            case State.SPRAY:
                print("Im spraying AHHHH!")
                spray(state, bot)

            case State.PUT_DOWN_SPRAY:
                print("Sorry i dont have anything left to spray...")
                put_down_spray(state, bot)

            case State.PLACE_CUP_TO_STAT:
                print("Give me the cup.")
                place_cup_to_stat(state, bot)

            case State.FILL_CUP:
                print("Im filling AHHH!")
                fill_cup(state, bot)

            case State.CUP_TO_IRON:
                print("Batter to iron!")
                cup_to_iron(state, bot)

            case State.EMPTY_CUP:
                print("im emptying my cup AHHH! 🐬")
                empty_cup(state, bot)

            case State.RETURN_CUP:
                print("Lets put the cup back 🗿")
                return_cup(state, bot)

            case State.CLOSE_IRON:
                print("Its all going well, closing iron! 👺")
                close_iron(state, bot)

            case State.FUN_TIME:
                print("This is where the fun begins 😏")
                fun_time(state, bot)

            case State.OPEN_IRON2:
                print("Waffle is done opening iron! 🥴")
                open_iron2(state, bot)

            case State.PICK_UP_WAFFLE:
                print("Look at my child, its beautiful 👺")
                pick_up_waffle(state, bot)

            case State.SERVE_WAFFLE:
                print("Here is your waffle kind sir 🤵")
                serve_waffle(state, bot)

            case State.RETURN_STICK:
                print("Stick to IRON!!!")
                return_stick(state, bot)

            case State.ERROR:
                print("Hagle")
                error(state, bot)
                
            case State.SHUTDOWN:
                print("Shutting Down... 🚫")
                break

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
