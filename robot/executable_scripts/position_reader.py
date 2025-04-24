# robot modules
from os import read
from robot.robot_controllers.Wafflebot.Wafflebot import *
from robot.tools.file_manipulation import Jsonreader, table_print
from robot.tools.errorhandling import handle_error
from robot.tools.update_tagoffsets import create_offset_matrix
from camera.vision import Vision
# user libraries: 
from time import sleep
from typing import Literal
from threading import Thread, Event
from queue import Queue
import numpy as numphy


def printmenu():
    print("\nPress 1 to record position\n"
          +"Press 2 to set a new reference tag\n" 
          +"Press 3 to play back saved position using joint method\n"
          +"Press 4 to remove a position\n"
          +"Press 5 to quit\n"
          +"Press 6 to print menu\n"
          +"Press 7 to toggle torque\n"
          +"Press 8 to toggle gripper\n"
          +"Press 9 to record trajectory\n"
          )
    return  

def print_stored_positions(data: dict) -> None: 
    """print the stored positions."""
    print("The stored positions are:")
    keys = data.keys()
    keys = sorted(keys) # also converts to list[str] as a bonus
    table_print(keys, skip_sort=True) 

def playposition(bot: Wafflebot, data_type: Literal["joints", "basepose"]): 
    # Set up arm
    bot.bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    jsonreader = Jsonreader() 
    data = jsonreader.read("recordings")
    print_stored_positions(data)
    
    # Ask for a name 
    names = input("Enter the name of the position(s) you want to go to,\n"
                  +"separated by a comma (,):\n").lower()
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
        if data_type == "basepose":
            bot.move(position)
        elif data_type == "joints":
            bot.arm.set_joint_positions(position)
        sleep(1)
    return    

def recordposition(bot: Wafflebot, tagid: int, vision: Vision):
    # detorque arm
    
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(0.25)    
    # wait for input before retorquing
    input("\nPress enter to record") 
    bot.core.robot_torque_enable("group", "arm", True)
    sleep(0.5)

    vision.run_once()
    # Record position
    bot.arm.capture_joint_positions()
    position_joints = bot.arm.get_joint_positions()
    
    # Test for valid position
    if bot.arm._check_joint_limits(position_joints):
        position_mat = bot.arm.get_ee_pose().tolist()
        if tagid != 100:
            tag = Jsonreader().read("camera_readings")[tagid]
            position_offset = create_offset_matrix(position_mat, tag)
        else:
            position_offset = 100
    else:
        print("Joints are not within their limits. Try again bozo.")
        bot.core.robot_torque_enable("group", "arm", False)
        return
    # Get the user to name the positions
    name = input("Write the name of your position:\n"
                + "Press enter to cancel recording\n").lower().strip()
    if name != "":
        # write ee position
        jsonreader = Jsonreader()
        data = jsonreader.read("recordings")
        data.update(
            {
            f"{name}":{
                "basepose": position_mat,
                "joints": position_joints,
                "tag" : tagid,
                "offset" : position_offset
                }
            },
        )
        jsonreader.write("recordings", data)
        print(f'Successfully written "{name}" to recordings.')
    
    
    return name


def _recordtrajectory(bot: Wafflebot, tagid: int, pose_name: str, event: Event, queue: Queue):
    # initialize values:
    poses = dict()
    poseindex = 0
    reader = Jsonreader()

    tag = reader.read("camera_readings").get(tagid)
    while True:
        # Record position
        bot.arm.capture_joint_positions()
        position_joints = bot.arm.get_joint_positions()
        # Test for valid position
        if bot.arm._check_joint_limits(position_joints):
            position_mat = bot.arm.get_ee_pose().tolist() 
            position_offset = 100
        else:
            print("Joints are not within their limits. Try again bozo.")
            bot.core.robot_torque_enable("group", "arm", True)
            return
        poses.update({f"{pose_name}_{poseindex}" :
                      {
                          "basepose":position_mat,
                          "joints" : position_joints,
                          "tag" : tagid,
                          "offset" : position_offset 
                      }})
        poseindex+=1
        sleep(0.02)
        if event.is_set():
            queue.put(poses)
            return 
        
def recordtrajectory(bot: Wafflebot, tagid: int):
    bot.core.robot_torque_enable("group", "arm", False)
    pose_name = input("Enter pose name to begin recording. press enter to cancel.\n")
    if pose_name == "":
        return
    q = Queue()
    e = Event()
    t = Thread(target=_recordtrajectory, daemon=True, args=(bot,tagid, pose_name, e, q))
    t.start()
    print("press enter stop recording")
    input()
    e.set()
    bot.core.robot_torque_enable("group", "arm", True)
    poses = q.get()
    t.join()
    if input("press enter to save. Press Q to abort.\n").lower() != "q":
        Jsonreader().write("recordings", poses)
    
        

def pop_item()->None:
    reader = Jsonreader()
    print_stored_positions(reader.read("recordings"))
    key = input("Tell me what position to remove, little boy: ")
    if reader.pop("recordings",key):
        print(f"Thanos snapped {key}. Perfectly balanced, as all things should be.")
    return


def main(bot):
    #print menu and listen for keystrokes:
    vision = Vision()
    tagid = 100
    
    torqued = False
    gripped = False
    
    while True:
        printmenu()
        userinput = input()
        if userinput == str(1):
            vision.run_once()
            recordposition(bot, tagid, vision)
        elif userinput == str(2):
            tagid = int(input("New ID: "))
        elif userinput == str(3):
            playposition(bot, "joints")
        elif userinput == str(4):
            pop_item()
        elif userinput == str(5):
            break
        elif userinput == str(7):
            bot.core.robot_torque_enable("group", "arm", torqued)
            torqued = not torqued
        elif userinput == str(8):
            if gripped:
                bot.grasp()
            else:
                bot.release()
            gripped = not gripped
        elif userinput == str(9):
            print("secret input pog")
            vision.run_once()
            recordtrajectory(bot,tagid)
        else:
            print("invalid input, try again bozo")

    # close bot, close program.
    bot.safe_stop()
    return

if __name__ == '__main__':
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from camera.vision import Vision
    from rclpy.exceptions import InvalidHandle
    from robot.tools.errorhandling import handle_error
    try:
        bot = Wafflebot(automatic_mode=False, detect_collisions=False)
        main(bot=bot)
    # if error detected, run the error handler
    except (InvalidHandle):
        pass
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)
        
