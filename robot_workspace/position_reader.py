# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# robot modules
from robot_workspace.assets import arm_positions
from robot_workspace.assets.Wafflebot import *
# user libraries: 
from time import sleep
from pynput import keyboard
import numpy as numphy
from importlib import reload as import_reload

def printmenu():
    print("\nPress 1 to record position\n"
          +"Press 2 to play back saved position\n"
          +"Press 3 to show this message again\n"
          +"Press 4 to quit")
    return


def playposition(bot: Wafflebot): 
    import_reload(arm_positions)
    bot.bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    bot.arm.go_to_home_pose()
    input("\nPress enter to continue")
    print("The stored positions are:")
    for func in dir(arm_positions):
        if (callable(getattr(arm_positions, func)) 
            and
        not func.startswith("__")
        ):
            print(func)
    #print the stored positions.
    names = input("Enter the name of the position(s) you want to go to,\n"
                  +"separated by a comma (,):\n")
    names = names.split(",")
    for name in names:
        name = name.strip() # remove whitespace
        if not hasattr(arm_positions,name):
            print(f"position {name} not found in position list")
            sleep(5)
            break
        pose = getattr(arm_positions, name)()
        bot.arm.set_ee_pose_matrix(pose)
        sleep(1)
    bot.arm.go_to_home_pose()
    printmenu()
    return


def recordposition(bot: InterbotixManipulatorXS):
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(0.25)
    input("\nPress enter to record") 
    bot.core.robot_torque_enable("group", "arm", True)
    sleep(1)
    bot.arm.capture_joint_positions()
    if bot.arm._check_joint_limits(bot.arm.get_joint_positions()):
        position = bot.arm.get_ee_pose()
        bot.arm.get_ee_pose_command
    else:
        print("Joints are not within their limits. Try again bozo.")
        printmenu()
        return

    name = input("Press enter to cancel recording\n"
                    + "Write the name of your position:\n")
    
    if name != "":
        with open("robot_workspace/assets/arm_positions.py", "a") as file:
            file.write(f"\n{name}=([\n")
            numphy.savetxt(file, position, fmt="  [% .8f, % .8f, % .8f, % .8f],")
            file.write("  ])\n")
        
        print(f'Successfully written "{name}" to arm_positions.py')
    
    printmenu()
    return


def make_on_press(bot):
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char:  # Check if the key has a 'char' attribute
                if key.char == "3":
                    printmenu()  # This will print the menu when '3' is pressed
                elif key.char == "4":
                    print("Exiting...")
                    return False  # Stop the listener when '4' is pressed
                elif key.char == "1":
                    recordposition(bot)  # Call recordposition function when '1' is pressed
                elif key.char == "2":
                    playposition(bot)
        except AttributeError:
            # Handle special keys like shift, ctrl, etc.
            print("Caught error, no worries")
            pass  # We don't need to worry about special keys
        return True  # Continue listening for other keys

    return on_press  # Return the inner on_press function
   
        

def main():
    # boot bot
    bot =  Wafflebot(1)
    bot.arm.go_to_sleep_pose()
    
    #print menu and listen for keystrokes:
    printmenu()
    on_press = make_on_press(bot)
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

    
    # close bot, close program.
    bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    sleep(4)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    sleep(1)
    robot_shutdown()
    robot_boot_manager.robot_close()

    return

# Footer:
def handle_error(signum, frame):raise KeyboardInterrupt
if __name__ == '__main__':
    from signal import signal, SIGINT; signal(SIGINT, handle_error)
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        with open("robot_workspace/backend_controllers/errorhandling.py") as errorhandler: exec(errorhandler.read())
    
