# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.positions import arm_joint_states
from robot_workspace.assets.Wafflebot import Wafflebot
from robot_workspace.assets.positions import arm_positions
from robot_workspace.backend_controllers import safety_functions as safety 
from robot_workspace.backend_controllers import robot_bounding_boxes, create_boxes
from importlib import reload as import_reload
from time import sleep
import numpy as numphy
from os import getcwd
import threading
import rclpy

def convert_box(box):
    lower_corner = box[0]
    higher_corner = box[1]
    center =[lower_corner[ind] +1*(higher_corner[ind] - lower_corner[ind])/2 for ind in range(len(lower_corner))]
    length = higher_corner[0] - lower_corner[0]
    width = higher_corner[1] - lower_corner[1]
    height = higher_corner[2] - lower_corner[2]

    box = {
        "x":center[0],
        "y":center[1], # y left when arm forward
        "z":center[2],
        "depth": height,
        "width": length,
        "height": width
        #"depth":0.005,# z
        #"width":0.005,# x
        #"height":0.005# y
    }
    return box
    
def read_boxes():
    path = getcwd()
    path += "/robot_workspace/assets/boundingboxes/robot.py"
    with open(path,"r") as file:
        box_list: dict = eval(file.read(), {"np":numphy})
    boxes = []  
    for key in box_list.keys():
        boxes.append(convert_box(box_list[key]))
    return boxes

def main():
    bot = Wafflebot()
    bot.arm.go_to_home_pose()
    
    bot.move(arm_positions.e)

    sleep(5)
    bot.safe_stop()
    

    
    
# Footer:
def handle_error(signum, frame): raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: 
            exec(errorhandler.read())
