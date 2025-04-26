from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.robot_controllers.path_planner import list_multiply, list_sum
from robot.tools.errorhandling import handle_error
from rclpy._rclpy_pybind11 import RCLError
from time import sleep, time
from robot.tools.file_manipulation import Jsonreader
from robot.tools.update_tagoffsets import create_offset_matrix, abs_position_from_offset
from camera.vision import Vision
from camera.config.configloader import ConfigLoader
from threading import Thread
import cv2
from robot.tools.timekeeper import record_time, read_times
import os
import numpy as numphy
from modern_robotics import FKinSpace


def get_aruco_pose(id: str):
    reader = Jsonreader()
    tags = reader.read("camera_readings")        
    return tags.get(id)

def recordOffset(bot: Wafflebot, tagid: str):
    bot_pos = bot.arm.get_ee_pose()
    reader = Jsonreader()
    tag_pos = reader.read("camera_readings")[tagid]
    offset = create_offset_matrix(bot_pos, tag_pos)
    reader.write("offsets", {"copy_camera": offset})
    print(f"written offset to copy_camera")


def compute_translation_DU(tag_pos: list[list[float]]) -> numphy.array:
    distance = 0.45 
    arm_height = 0.42705
    tag_pos = numphy.array(tag_pos)
    pos = tag_pos[:3,3]        
    pos[2] -= arm_height
    r = numphy.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
    pos = pos * distance / r
    pos[2] += arm_height
    return pos


def compute_rotation_DU(target_position: numphy.array) -> numphy.array:
    x, y, z = target_position
    arm_height = 0.42705 
    z -= arm_height
    
    Rx = numphy.array([x,y,z])
    Rx = Rx/numphy.linalg.norm(Rx)
    Ry = numphy.array([-Rx[1], Rx[0], 0])
    norm = numphy.linalg.norm(Ry)
    Ry = [1,0,0] if norm == 0 else Ry / norm  
    Rz = numphy.cross(Rx,Ry)
    Rz = Rz / numphy.linalg.norm(Rz)

    R = numphy.column_stack([Rx, Ry, Rz])
    return R
        



def follow_DU(bot: Wafflebot, tagid, vision):
    reader = Jsonreader()
    starttime = time()
    endtime = time()
    prev_pos = FKinSpace(
            bot.arm.robot_des.M,
            bot.arm.robot_des.Slist,
            bot.get_joint_positions()
            )
    prev_pos = numphy.array(prev_pos)[:3,3]
    while endtime - starttime <= 30:
        endtime = time()
        vision.run_once()
        tag_pos = reader.read("camera_readings")[tagid]

        pos = compute_translation_DU(tag_pos)
        R = compute_rotation_DU(pos)

        out_pos = numphy.identity(4)
        out_pos[:3,3] = pos 
        out_pos[:3,:3] = R
        out_pos = out_pos.tolist()

        # distance checking:
        target = numphy.array(out_pos)
        pos = target[:3,3]

        if numphy.linalg.norm([pos-prev_pos]) > 0.05:
            prev_pos = pos
            print("Moving robot")
            print(f"movement success? {bot.move(out_pos, speed_scaling=5.0)}")
        
        
def goToTag(bot: Wafflebot, tagid:str, pre_offset, vision):
    reader = Jsonreader()
    starttime = time()
    endtime = time()
    prev_pos = FKinSpace(
            bot.arm.robot_des.M,
            bot.arm.robot_des.Slist,
            bot.get_joint_positions()
            )
    prev_pos = numphy.array(prev_pos)[:3,3]
    while endtime - starttime <= 10:
        endtime = time()
        vision.run_once()
        tag_pos = reader.read("camera_readings")[tagid]
        if pre_offset is not None:
            offset = pre_offset
        else:
            offset = reader.read("offsets")["copy_camera"]
        target = abs_position_from_offset(tag_pos, offset)

        # distance checking:
        target = numphy.array(target)
        pos = target[:3,3]

        if numphy.linalg.norm([pos-prev_pos]) > 0.05:
            prev_pos = pos
            target = target.tolist()
            print("Moving robot")
            print(f"movement success? {bot.move(target, speed_scaling=5.0)}")

def follow_tag(bot, tagid, vision):
    offset =[
    [0.0,0.0,1.0,0.0],
    [0.0,1.0,0.0,0.0],
    [-1.0,0.0,0.0,0.20],
    [0.0,0.0,0.0,1.0]
    ]
    goToTag(bot,tagid, offset, vision)

def printmenu():
    print("Press 1 to record offset")
    print("Press 2 to go to offset")
    print("press 3 to set tag id")
    print("Press 4 to toggle arm torque")
    print("press 5 to exit")
    print("press 6 to follow the tag like a silly lil goose")
    print("press HAGLE to follow DU")
    return 

def main(bot):
    # Init robot  
    vision = Vision()
  
    bot.go_to_home_pose()
    tagid = "25"
    torqed = True
    while True:
        printmenu()
        choice = input("Input: ")
        try:
            choice = int(choice)
        except ValueError:
            print("That was not a number😡") # 😡
        match choice:
            case 1:
                vision.run_once()
                recordOffset(bot, tagid)
            case 2:
                goToTag(bot, tagid, vision)
            case 3: 
                tagid = str(input("Input new ID: "))
            case 4:
                bot.core.robot_torque_enable("group", "arm", not torqed)
                torqed = not torqed
            case 5:
                break
            case 6:
                follow_tag(bot,tagid, vision)
                
            case 42453:
                follow_DU(bot, tagid, vision)
            case _:
                print("invalid input. Try again.")

    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    bot = None
    try:
        
        bot = Wafflebot(automatic_mode=True,detect_collisions=True)
        main(bot)
        bot.exit()
        
    # if error detected, run the error handler
    except (Exception, KeyboardInterrupt, RCLError) as e:
        handle_error(e, bot)
