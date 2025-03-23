"""
This script outlines main()

It should however be viewed as a template and not funtional
"""
from camera.init_camera import initalize_system as init_camera
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot import Wafflebot
from robot.tools.waffle_states.waffle_states import CurrentState
from main.execute import execute

def init():
    bot = Wafflebot()
    try:
        camera, aruco, coordsys = init_camera()
    except RuntimeError:
        print("Camera not connected")
        return None

    return bot, camera, aruco, coordsys

def main()->None:
    
    everything=init()    
    
    if everything is None:
        return everything
    
    running = True
    
    while running:
        running = execute(everything)
    
    return None

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)