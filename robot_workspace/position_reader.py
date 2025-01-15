
# Change the working directory to the base directory
from os import chdir, getcwd
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# interbotix modules
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# user libraries: 
from robot_workspace.backend_controllers import robot_boot_manager
from time import sleep
from pynput import keyboard

def printmenu():
    print("Press enter to record position\n"
          +"Press q to quit\n"
          +"Press h to show this message again")
    return

def recordposition(bot: InterbotixManipulatorXS):
    if position == "pass":
        return
    bot.core.robot_torque_enable("group", "arm", True)
    position = bot.arm.get_ee_pose()
    bot.core.robot_torque_enable("group", "arm", False)
    name = input("Write the name of your position")    
    with open("robot_workspace/assets/arm_positions", "a") as file:
        file.write(name+":\n"+ position +"\n")
    return

def on_press(key, bot):
    if key.char == "h":
        printmenu()
    elif key.char == "q":
        return False  # Stop listener
    elif key.char == "r":
        recordposition(bot)
        

def main():
    # boot bot
    robot_boot_manager.robot_launch(use_real_robot=False)
    bot = InterbotixManipulatorXS(robot_model="vx300s",
                                  group_name="arm",
                                  gripper_name="gripper",
                                  accel_time=0.05)
    robot_startup()
    
    bot.arm.go_to_sleep_pose()
    bot.core.robot_torque_enable("group", "arm", False)
    
    #print menu:
    printmenu()
    with keyboard.Listener(on_press=on_press(key= bot=bot)) as listener:
        listener.join()





    # close bot, close program.
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    sleep(5)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)
    robot_shutdown()
    robot_boot_manager.robot_close()


    return
if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
