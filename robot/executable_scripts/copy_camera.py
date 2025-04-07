from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
from rclpy._rclpy_pybind11 import RCLError
from time import sleep, time
from robot.tools.file_manipulation import Jsonreader
from robot.tools.visualizers.tf_publisher import TFPublisher
from robot.tools.update_tagoffsets import create_offset_matrix, abs_position_from_offset
from camera.init_camera import initalize_system as init_camera
from threading import Thread
import cv2
from robot.tools.timekeeper import record_time, read_times
import os


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

    return

def show_camera(camera):
    cv2.namedWindow("RealSense Camera", cv2.WINDOW_NORMAL)  # Make window resizable 
    while True:
        image = camera.get_image()
        
        if image is not None:
            cv2.imshow("RealSense Camera", image)
        else:
            print("No image captured")
        
        # Break on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def goToTag(bot: Wafflebot, tagid:str, camera, pre_offset):
    i = 0
    reader = Jsonreader()
    starttime = time()
    endtime = time()
    while endtime - starttime <= 10:
        endtime = time()
        record_time("(buffer)")
        bot.cam.start("all")
        record_time(f"camera_frame_no_{i}")
        tag_pos = reader.read("camera_readings")[tagid]
        if pre_offset is not None:
            offset = pre_offset
        else:
            offset = reader.read("offsets")["copy_camera"]
        target = abs_position_from_offset(tag_pos, offset)

        print("Moving robot")
        # plan a:
        print(f"movement success? {bot.move(target, speed_scaling=4.0)}")
        record_time(f"robot_frame_no_{i}")
        # plan b:
        #bot.arm.set_ee_pose_matrix(target, blocking=False)

        sleep(0.2)
    return

def follow_tag(bot, tagid, camera):
    offset =[
    [0.0,0.0,1.0,0.0],
    [0.0,1.0,0.0,0.0],
    [-1.0,0.0,0.0,0.25],
    [0.0,0.0,0.0,1.0]
    ]
    goToTag(bot,tagid,camera, offset)

def printmenu():
    print("Press 1 to record offset")
    print("Press 2 to go to offset")
    print("press 3 to set tag id")
    print("Press 4 to toggle arm torque")
    print("press 5 to exit")
    print("press 6 to follow the tag like a silly lil goose")
    return 

def main(bot: Wafflebot, camera_display, throwaway2, camera_coordsys):
    # Init robot  
    
    #Thread(target=show_camera,daemon=True, args=[camera_display]).start()
    print("hello worl")
    bot.go_to_home_pose()
    pub = TFPublisher()
    tagid = "25"
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
                camera_coordsys.start("all") 
                recordOffset(bot, tagid, pub)
            case 2:
                os.system("clear")
                record_time("start")
                record_time(f"move_start_{movement_number}")
                goToTag(bot, tagid,camera_coordsys, None)
                record_time(f"move_end_{movement_number}")
                read_times()
                movement_number +=1
            case 3: 
                tagid = str(input("Input new ID: "))
            case 4:
                bot.core.robot_torque_enable("group", "arm", not torqed)
                torqed = not torqed
            case 5:
                break
            case 6:
                os.system("clear")
                record_time("start")
                record_time(f"move_start_{movement_number}")
                follow_tag(bot,tagid,camera_coordsys)
                record_time(f"move_end_{movement_number}")
                movement_number +=1
                read_times()
            case _:
                print("invalid input. Try again.")

    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    bot = None
    try:
        camera_display,throwaway2,camera_coordsys = init_camera()
        bot = Wafflebot(camera_coordsys, use_rviz=False)
        main(bot, camera_display, throwaway2, camera_coordsys)
        bot.exit()
    # if error detected, run the error handler
    except (Exception, KeyboardInterrupt, RCLError) as e:
        handle_error(e, bot)
