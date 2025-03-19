"""
This script outlines main()

It should however be viewed as a template and not funtional
"""
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot import Wafflebot
from camera.init_camera import initalize_system as init_camera
import numpy as numphy


def init():
    bot = Wafflebot()
    try:
        camera, aruco, coordsys = init_camera()
    except RuntimeError:
        print("Camera not connected")
        return None
    bot.arm.go_to_home_pose
    return bot, camera, aruco, coordsys

def main()->None:
    everything=init()    
    if everything is None:
        return None
    running = True
    while running:
        running = next_action(everything)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)