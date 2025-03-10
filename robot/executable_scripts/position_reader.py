# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath

chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# robot modules
from robot.backend_controllers.robot_controllers.Wafflebot import *
from robot.tools.file_manipulation import Jsonreader
# user libraries: 
from time import sleep
from typing import Literal
import numpy as numphy
import json


def printmenu():
    print("\nPress 1 to record position\n"
          +"Press 2 to play back saved position\n"
          +"Press 3 to play back saved position using joint method\n"
          +"Press 4 to remove a position\n"
          +"Press 5 to quit")
    return  

def print_stored_positions(data: dict) -> None: 
    """print the stored positions."""
    print("The stored positions are:")
    keys = data.keys()
    keys = sorted(keys)
    keylist = ""
    line_length = 50
    tab = "    "
    for key in keys:
        keylist = keylist + key + tab 
        if len(keylist) > line_length:
            print(keylist)
            keylist = ""
    print(keylist+"\n")

def playposition(bot: Wafflebot, data_type: Literal["joints", "matrix"]): 
    # Set up arm
    bot.bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    jsonreader = Jsonreader() 
    data = jsonreader.read("recordings")
    print_stored_positions(data)
    
    # Ask for a name 
    names = input("Enter the name of the position(s) you want to go to,\n"
                  +"separated by a comma (,):\n")
    names = names.split(",")
    
    # Get name and check if it is in name list
    for name in names:
        name = name.strip() # remove whitespace
        if name not in data.keys():
            print(f"position {name} not found in position list")
            sleep(1)
            break
        print(f"Going to {name}")
        position = data[name][data_type]
        if data_type == "matrix":
            bot.move(position,file="recordings")
        elif data_type == "joints":
            bot.arm.set_joint_positions(position)
        sleep(1)
    return    

def recordposition(bot: InterbotixManipulatorXS):
    # detorque arm
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(0.25)    
    # wait for input before retorquing
    input("\nPress enter to record") 
    bot.core.robot_torque_enable("group", "arm", True)
    sleep(0.5)

    # Record position
    bot.arm.capture_joint_positions()
    position_joints = bot.arm.get_joint_positions()
    
    # Test for valid position
    if bot.arm._check_joint_limits(position_joints):
        position_mat = bot.arm.get_ee_pose()
    else:
        print("Joints are not within their limits. Try again bozo.")
        bot.core.robot_torque_enable("group", "arm", False)
        return
    # Get the user to name the positions
    name = input("Write the name of your position:\n"
                + "Press enter to cancel recording\n")
    if name != "":
        # write ee position
        jsonreader = Jsonreader()
        data = jsonreader.read("recordings")
        data.update(
            {
            f"{name}":{
                "matrix": position_mat.tolist(),
                "joints": position_joints
                }},
        )
        jsonreader.write("recordings", data)
        print(f'Successfully written "{name}" to recordings.')
    #Reset before next move    
    return

def pop_item()->None:
    reader = Jsonreader()
    print_stored_positions(reader.read("recordings"))
    key = input("Tell me what position to remove, little boy: ")
    if reader.pop("recordings",key):
        print(f"Thanos snapped {key}. Perfectly balanced, as all things should be.")
    return


def main():
    # boot bot
    bot =  Wafflebot()
    bot.arm.go_to_sleep_pose()
    #print menu and listen for keystrokes:
    while True:
        printmenu()
        userinput = input()
        if userinput == str(1):
            recordposition(bot)
        elif userinput == str(2):
            playposition(bot, "matrix")
        elif userinput == str(3):
            playposition(bot, "joints")
        elif userinput == str(4):
            pop_item()
        elif userinput == str(5):
            break
        else:
            print("invalid input, try again bozo")

    # close bot, close program.
    bot.safe_stop()
    return

# Footer:
def handle_error(signum, frame):raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
    
