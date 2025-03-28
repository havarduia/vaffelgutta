from main.waffle_states.waffle_states import State, CurrentState

from main.state_transitions import *

def execute(
        bot:"Wafflebot",
        camera: "Camera",
        aruco: "Aruco",
        coordinate_system: "CoordinateSystem"
                        )->bool:
    """
    Executes the next action in the actions list.
    """

    state = CurrentState()
    current_state = state.get()
   
    match current_state:    

        case State.REST:
            print("I am resting, dammit. These youngins...")
            rest(state, bot)

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
            print("Sorry i dont have anything left...")
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
            print("im emptying my cup AHHH!")
            empty_cup(state, bot)
            
        case State.RETURN_CUP:
            print("Lets put the cup back.")
            return_cup(state, bot)
        
        case State.CLOSE_IRON:
            print("Its all going well, closing iron!")
            close_iron(state, bot)
            
        case State.FUN_TIME:
            print("This is where the fun begins.")
            fun_time(state, bot)
            
        case State.OPEN_IRON2:
            print("Waffle is done opening iron!")
            open_iron2(state, bot)
            
        case State.PICK_UP_WAFFLE:
            print("Look at my child, its beautiful")
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

        case _:
            bot.safe_stop(slow=True) 
            print("An unknown state was encountered!")
            return False

    return True

if __name__ == "__main__":
    # For type hinting:
    from robot.robot_controllers.Wafflebot import Wafflebot
    from camera.realsense import Camera
    from camera.aruco import Aruco
    from camera.coordinatesystem import CoordinateSystem
