# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# interbotix modules
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# user libraries: 
from time import sleep
from pynput import keyboard
import numpy as numphy
from robot_workspace.backend_controllers import robot_boot_manager


def printmenu():
    print("\nPress 1 to record position\n"
          +"Press 2 to show this message again\n"
          +"Press 3 to quit")
    return

def recordposition(bot: InterbotixManipulatorXS):
    sleep(0.25)
    input("\nPress enter to record") 
    bot.core.robot_torque_enable("group", "arm", True)
    position = bot.arm.get_ee_pose()
    bot.core.robot_torque_enable("group", "arm", False)

    name = input("Press enter to cancel recording\n"
                    + "Write the name of your position:\n")
    
    if name != "":
        with open("robot_workspace/assets/arm_positions.py", "a") as file:
            file.write(f"\ndef {name}():\n")
            file.write("  return ([\n")
            numphy.savetxt(file, position, fmt="  [% .8f, % .8f, % .8f, % .8f],")
            file.write("  ])\n")
        
        print(f'Successfully written "{name}" to arm_positions.py')

    printmenu()
    return


def make_on_press(bot):
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char:  # Check if the key has a 'char' attribute
                if key.char == "2":
                    printmenu()  # This will print the menu when 'h' is pressed
                elif key.char == "3":
                    print("Exiting...")
                    return False  # Stop the listener when 'q' is pressed
                elif key.char == "1":
                    recordposition(bot)  # Call recordposition function when 'r' is pressed
        except AttributeError:
            # Handle special keys like shift, ctrl, etc.
            print("Caught error, no worries")
            pass  # We don't need to worry about special keys
        return True  # Continue listening for other keys

    return on_press  # Return the inner on_press function
   
        

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
    
    #print menu and listen for keystrokes:
    printmenu()
    on_press = make_on_press(bot)
    with keyboard.Listener(on_press=on_press) as listener:
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
