from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.robot_controllers.path_planner import list_multiply, list_sum
from robot.tools.errorhandling import handle_error
from rclpy._rclpy_pybind11 import RCLError
from time import sleep, time
from robot.tools.file_manipulation import Jsonreader
from robot.tools.visualizers.tf_publisher import TFPublisher
from robot.tools.update_tagoffsets import create_offset_matrix, abs_position_from_offset
from camera.vision import Vision
from camera.config.configloader import ConfigLoader
from threading import Thread
import cv2
from robot.tools.timekeeper import record_time, read_times
import os
import numpy as numphy
from ai.hand_detection import HandDetector

def get_aruco_pose(id: str):
    reader = Jsonreader()
    tags = reader.read("camera_readings")        
    return tags.get(id)

def recordOffset(bot: Wafflebot, tagid: str, visualizer: TFPublisher = None):
    bot_pos = bot.arm.get_ee_pose()
    reader = Jsonreader()
    tag_pos = reader.read("camera_readings")[tagid]
    visualizer.broadcast_transform(tag_pos)
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
        



def follow_DU(bot: Wafflebot, handmatrix):
    reader = Jsonreader()
    starttime = time()
    endtime = time()
    while endtime - starttime <= 30:
        endtime = time()
        bot.hand.start()
        tag_pos = reader.read("hand_position")[handmatrix]

        pos = compute_translation_DU(tag_pos)
        R = compute_rotation_DU(pos)

        out_pos = numphy.identity(4)
        out_pos[:3,3] = pos 
        out_pos[:3,:3] = R
        out_pos = out_pos.tolist()

        print("Moving robot")
        # plan a:
        print(f"movement success? {bot.move(out_pos, speed_scaling=4.0)}")
        
        

        
def goToTag(bot: Wafflebot, tagid:str, pre_offset):
    i = 0
    reader = Jsonreader()
    starttime = time()
    endtime = time()
    while endtime - starttime <= 10:
        endtime = time()
        bot.vision.run_once()
        
        tag_pos = reader.read("camera_readings")[tagid]
        if pre_offset is not None:
            offset = pre_offset
        else:
            offset = reader.read("offsets")["copy_camera"]
        target = abs_position_from_offset(tag_pos, offset)

        print("Moving robot")
        # plan a:
        print(f"movement success? {bot.move(target, speed_scaling=4.0)}")
        # plan b:
        #bot.arm.set_ee_pose_matrix(target, blocking=False)

        sleep(0.2)
    return

def follow_tag(bot, tagid):
    offset =[
    [0.0,0.0,1.0,0.0],
    [0.0,1.0,0.0,0.0],
    [-1.0,0.0,0.0,0.20],
    [0.0,0.0,0.0,1.0]
    ]
    goToTag(bot,tagid, offset)

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
    
    #Thread(target=show_camera,daemon=True, args=[camera_display]).start()
    print("hello world")
    bot.go_to_home_pose()
    pub = TFPublisher()
    tagid = "25"
    handmatrix = "matrix"
    torqed = True
    movement_number = 1
    while True:
        printmenu()
        choice = input("Input: ")
        try:
            choice = int(choice)
        except ValueError:
            print("That was not a number😡") # 😡
        match choice:
            case 1:
                bot.vision.run_once()
                recordOffset(bot, tagid, pub)
            case 2:
                goToTag(bot, tagid, None)
            case 3: 
                tagid = str(input("Input new ID: "))
            case 4:
                bot.core.robot_torque_enable("group", "arm", not torqed)
                torqed = not torqed
            case 5:
                break
            case 6:
                follow_tag(bot,tagid)
            case 42453:
                follow_DU(bot, handmatrix)
            case _:
                print("invalid input. Try again.")

    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    bot = None
    vision = Vision()
    hand = HandDetector(Vision)
    try:
        
        bot = Wafflebot(vision=vision, use_rviz=False, automatic_mode=True, use_hand_detection=True)
        main(bot)
        bot.exit()
        
    # if error detected, run the error handler
    except (Exception, KeyboardInterrupt, RCLError) as e:
        handle_error(e, bot)
