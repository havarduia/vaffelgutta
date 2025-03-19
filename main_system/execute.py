from (somewhere) import get_state
from robot.assets.waffle_states.waffle_states import State
def execute(
        bot: Wafflebot,
        camera: Camera,
        aruco: Aruco,
        coordinate_system: CoordinateSystem
        )->bool:
    state = get_state(bot,camera,aruco,coordinate_system) # returns a State
    
    
    match state:
        
        case State.REST:
            print("I am resting, dammit. These youngins...")
        
        case State.state1:
            print("I am a demo state!")

        case _:
            bot.safe_stop(slow = True) 
            print("An unknown state was encountered!")
            return False





if __name__ == "__main__":
    # For type hinting:
    from robot.robot_controllers.Wafflebot import Wafflebot
    from camera.realsense import Camera
    from camera.aruco import Aruco
    from camera.coordinatesystem import CoordinateSystem