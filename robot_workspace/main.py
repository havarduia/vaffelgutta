# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

from robot_workspace.assets.Wafflebot import Wafflebot
from time import sleep
from robot_workspace.robot_movements import rock_paper_scissors
def main():
    
    bot = Wafflebot()
    
    bot.arm.go_to_home_pose()

    rock_paper_scissors.rock_paper_scissors(bot)
    
    sleep(5)
    bot.safe_stop()


# Footer:
def handle_error(signum, frame):raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
    
