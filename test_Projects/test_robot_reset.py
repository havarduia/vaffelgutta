from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
from time import sleep

def main():
    bot = Wafflebot()
    bot.arm.go_to_home_pose()
    torque = False
    while True:
        choice = str(input("Next: "))
        if choice == "q": break
        if choice == "t":
            bot.core.robot_torque_enable("group","arm",torque)
            torque = not torque
        if choice =="r":
            bot.core.robot_reboot_motors("group","arm",True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        handle_error(e)
