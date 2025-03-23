from robot.tools.waffle_states.waffle_states import State, CurrentState

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

    match state.get_state():    
        
        case State.SLEEP:
            # named sleepstate instead of sleep to avoid import conflict
            sleepstate(state, bot) 
        case State.REST:
            print("I am resting, dammit. These youngins...")
            rest(state, bot)
        
        case State.state1:
            print("I am a demo state!")

        case _:
            bot.safe_stop(slow = True) 
            print("An unknown state was encountered!")
            return False
    return True




if __name__ == "__main__":
    # For type hinting:
    from robot.robot_controllers.Wafflebot import Wafflebot
    from camera.realsense import Camera
    from camera.aruco import Aruco
    from camera.coordinatesystem import CoordinateSystem
