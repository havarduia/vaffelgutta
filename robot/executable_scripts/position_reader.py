# robot modules
from robot.robot_controllers.Wafflebot.Wafflebot import *
from robot.tools.file_manipulation import Jsonreader, table_print
from robot.tools.errorhandling import handle_error
from robot.tools.update_tagoffsets import create_offset_matrix
from camera.coordinatesystem import CoordinateSystem
# user libraries: 
from time import sleep
from typing import Literal
import keyboard
import numpy as numphy



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
    keys = sorted(keys) # also converts to list[str] as a bonus
    table_print(keys, skip_sort=True)

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

def recordposition(bot: Wafflebot):
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
        position_mat = bot.arm.get_ee_pose().tolist()
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
                "matrix": position_mat,
                "joints": position_joints
                }},
        )
        jsonreader.write("recordings", data)
        print(f'Successfully written "{name}" to recordings.')
    return name

def recordtrajectory(bot: Wafflebot):
    # initialize values:
    i = 0
    poses = dict()
    poseindex = 0
    reader = Jsonreader()

    bot.core.robot_torque_enable("group", "arm", False)
    pose_name = input("Enter pose name to begin recording. press enter to cancel.")
    if pose_name == "":
        return
    print("press S to stop recording")
    print("press Q to abort")
    while True:
        if keyboard.is_pressed("s"):
            break;
        if keyboard.is_pressed("q")
            bot.core.robot_torque_enable("group", "arm", True)
            return
        i+=1
        sleep(0.01)
        if i == 100:
            i = 0

            # Record position
            bot.arm.capture_joint_positions()
            position_joints = bot.arm.get_joint_positions()
            # Test for valid position
            if bot.arm._check_joint_limits(position_joints):
                position_mat = bot.arm.get_ee_pose().tolist()
            else:
                print("Joints are not within their limits. Try again bozo.")
                bot.core.robot_torque_enable("group", "arm", True)
                return
            poses.update({f"{pose_name}_{poseindex}" : position_mat})
            poseindex+=1
    bot.core.robot_torque_enable("group", "arm", True)
    reader.write("recordings", poses)





        
        

def pop_item()->None:
    reader = Jsonreader()
    
    print_stored_positions(reader.read("recordings"))
    key = input("Tell me what position to remove, little boy: ")
    if reader.pop("recordings",key):
        print(f"Thanos snapped {key}. Perfectly balanced, as all things should be.")
    return

def record_offset(bot:Wafflebot, cam: CoordinateSystem):
    name = recordposition(bot)
    cam.start("all")
    reader = Jsonreader()
    data = reader.read("recordings")    
    tags = reader.read("camera_readings")

    tagid = 25
    tagid = input(f"Give me a tagid!! default: {tagid}\nInput: ")
    name = recordposition(bot)

    cam.start(tagid)
    reader = Jsonreader()
    robot_positions = reader.read("recordings")
    reader.pop("recordings", name)
    tags = reader.read("camera_readings")

    robot_position = robot_positions[name]["matrix"]    
    robot_joints = robot_positions[name]["joints"]
    tag = tags[tagid]

    robot_position = data[name]["matrix"]    
    robot_joints = data[name]["joints"]

    offset = create_offset_matrix(robot_position, tag)

    newdata = {
        "basepose" : robot_position,
        "joints" : robot_joints,
        "tag" : tagid,
        "offset": offset
    }
    
    reader.write("offsets", {name : newdata})     

    print("successfully recorded offset.")
    return None

def main(bot,cam,aruco,coordsys):
    # boot bot
    bot.go_to_sleep_pose()
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
        elif userinput == str(9):
            print("You used a secret input, you sneaky rascal!")
            record_offset(bot, camera_coordsys)
        else:
            print("invalid input, try again bozo")

    # close bot, close program.
    bot.safe_stop()
    return

if __name__ == '__main__':
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from camera.init_camera import initalize_system as init_camera
    from rclpy.exceptions import InvalidHandle
    from robot.tools.errorhandling import handle_error
    try:
        cam, aruco, coordsys = init_camera()
        bot = Wafflebot(coordsys)
        main(bot=bot,cam=cam,aruco=aruco,coordsys=coordsys)
    # if error detected, run the error handler
    except (InvalidHandle):
        pass
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)
        
