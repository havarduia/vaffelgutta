# Change the working directory to the base directory
import directory_fixer
directory_fixer.fix_directiory()
from robot.robot_controllers.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
from time import sleep
from robot.tools.file_manipulation import Jsonreader
from robot.tools.visualizers.tf_publisher import TFPublisher
from robot.tools.update_tagoffsets import create_offset_matrix, abs_position_from_offset
import numpy as numphy
from camera.init_camera import initalize_system as init_camera

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

def goToTag(bot: Wafflebot, tagid:str, camera, visualizer: TFPublisher = None):
    i = 0
    reader = Jsonreader()
    while i<int(10/0.1):
        i+=1
        camera.start(25)
        tag_pos = reader.read("camera_readings")[tagid]
        offset = reader.read("offsets")["copy_camera"]
        target = abs_position_from_offset(tag_pos, offset)

        visualizer.broadcast_transform(target)
        # plan a:
        bot.move(target, speed_scaling=0.2, blocking=False)
        # plan b:
        #bot.arm.set_ee_pose_matrix(target, blocking=False)
        print("Moving robot")
        sleep(0.1)
    return

def printmenu():
    print("Press 1 to record offset")
    print("Press 2 to go to offset")
    print("press 3 to set tag id")
    print("Press 4 to toggle arm torque")
    print("press 5 to exit")
    print("press 6 to toggle rotation offset")
    return 
def main():
    # Init robot
    throwaway,throwaway2,camera = init_camera()
    sleep(3)
    bot = Wafflebot(use_real_robot=False, debug_print=True)    
    bot.arm.go_to_home_pose()
    pub = TFPublisher()
    use_offset = True
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
                camera.start(25) 
                recordOffset(bot, tagid, pub)
            case 2:
                goToTag(bot, tagid,None, pub)
            case 3: 
                tagid = str(input("Input new ID: "))
            case 4:
                bot.core.robot_torque_enable("group", "arm", not torqed)
                torqed = not torqed
            case 5:
                break
            case 6:
                use_offset = not use_offset
            case _:
                print("invalid input. Try again.")
    

    
    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)