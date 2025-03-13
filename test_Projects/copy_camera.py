# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
from threading import Thread
from robot.robot_controllers.Wafflebot import Wafflebot
from robot.executable_scripts.common.errorhandling import handle_error
from time import sleep
from robot.tools.file_manipulation import Jsonreader
from robot.tools.visualizers.tf_publisher import TFPublisher
def get_aruco_pose(id: str):
    reader = Jsonreader()
    tags = reader.read("camera_readings")        
    return tags.get(id)


def main():
    # Init robot
    bot = Wafflebot(use_real_robot=False)    
    bot.arm.go_to_home_pose()
    pub = TFPublisher()
    # Put your code here:
    running = True
    while running:
        try:
            pose = get_aruco_pose("25")
            pub.broadcast_transform(pose)
            bot.move(pose)
            sleep(0.1)
        except KeyboardInterrupt:
            running = False
    # Close bot, close program:
    bot.safe_stop()

if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)