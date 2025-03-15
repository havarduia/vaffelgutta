# Change the working directory to the base directory
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
from time import sleep
from robot.tools.file_manipulation import Jsonreader
from robot.tools.visualizers.tf_publisher import TFPublisher
from robot.tools.update_tagoffsets import create_offset_matrix, abs_position_from_offset


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

def goToTag(bot: Wafflebot, tagid:str, visualizer: TFPublisher = None ):
    reader = Jsonreader()
    tag_pos = reader.read("camera_readings")[tagid]
    offset = reader.read("offsets")["copy_camera"]
    target = abs_position_from_offset(tag_pos, offset)
    # plan a:
    bot.move(target)
    # plan b:
    """
    visualizer.broadcast_transform(target)
    bot.arm.set_ee_pose_matrix(target, blocking=False)
    """

def printmenu():
    print("Press 1 to record offset")
    print("Press 2 to go to offset")
    print("press 3 to set tag id")
    print("Press 4 to toggle arm torque")
    print("press 5 to exit")
    return

def main():
    # Init robot
    bot = Wafflebot(use_real_robot=False)    
    bot.arm.go_to_home_pose()
    pub = TFPublisher()
    # Put your code here:
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
                recordOffset(bot, tagid)
            case 2:
                goToTag(bot, tagid, pub)
            case 3: 
                tagid = str(input("Input new ID: "))
            case 4:
                bot.core.robot_torque_enable("group", "arm", not torqed)
                torqed = not torqed
            case 5:
                break
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