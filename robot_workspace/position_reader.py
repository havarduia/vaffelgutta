# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath

chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))

# robot modules
from robot_workspace.assets.positions import positions
from robot_workspace.assets.positions import joint_states
from robot_workspace.assets.Wafflebot import *
# user libraries: 
from time import sleep
from pynput import keyboard
import numpy as numphy
from importlib import reload as import_reload
import inspect

def printmenu():
    print("\nPress 1 to record position\n"
          +"Press 2 to play back saved position\n"
          +"Press 3 to play back saved position using joint method\n"
          +"Press 4 to show this message again\n"
          +"Press 5 to quit")
    return


def playposition(bot: Wafflebot): 
    # Set up arm
    import_reload(positions)
    bot.bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
    # Clear input buffer
    input("\nPress enter to continue")
    print("The stored positions are:")
    
    #print the stored positions.
    members = inspect.getmembers(positions)
    valid_positions = ([name for name, obj in members if not inspect.isfunction(obj) and not inspect.isclass(obj) and not name.startswith("__")])
    print(valid_positions)

    # Ask for a name 
    names = input("Enter the name of the position(s) you want to go to,\n"
                  +"separated by a comma (,):\n")
    names = names.split(",")
    
    # Get name and check if it is in name list
    for name in names:
        name = name.strip() # remove whitespace
        if name not in valid_positions:
            print(f"position {name} not found in position list")
            sleep(1)
            break
        
        print(f"Going to {name}")
        bot.move(name)
        #bot.move(name,ignore=["robot_cradle"])
        #bot.small_movement(name)
        
        sleep(1)
    
    # Reset
    printmenu()
    return


def playjoints(bot: Wafflebot):
    # Set up arm
    import_reload(joint_states)
    bot.bot.core.robot_torque_enable("group", "arm", True)
    bot.arm.capture_joint_positions()
  
    
    #Clear input buffer
    input("\nPress enter to continue")

    #print the stored positions.
    print("The stored positions are:")
    members = inspect.getmembers(joint_states)
    valid_positions = ([name for name, obj in members if not inspect.isfunction(obj) and not inspect.isclass(obj) and not name.startswith("__")])
    print(valid_positions)

    #Ask for a position and see that it is valid
    names = input("Enter the name of the position(s) you want to go to,\n"
                  +"separated by a comma (,):\n")
    names = names.split(",")
    for name in names:
        name = name.strip() # remove whitespace
        if name not in valid_positions:
            print(f"position {name} not found in position list")
            sleep(1)
            break
        
        # Go to the given position
        pose = getattr(joint_states, name)
        print(f"Going to {name}")
        bot.arm.set_joint_positions(pose)
        sleep(1)
    
    # Reset before next move
    printmenu()
    return
    

def recordposition(bot: InterbotixManipulatorXS):
    # detorque arm
    bot.core.robot_torque_enable("group", "arm", False)
    sleep(0.25)
    
    # wait for input before retorquing
    input("\nPress enter to record") 
    bot.core.robot_torque_enable("group", "arm", True)
    sleep(0.5)

    # Record position
    bot.arm.capture_joint_positions()
    position_joints = bot.arm.get_joint_positions()
    
    # Test for valid position
    if bot.arm._check_joint_limits(position_joints):
        position_mat = bot.arm.get_ee_pose()
    else:
        print("Joints are not within their limits. Try again bozo.")
        bot.core.robot_torque_enable("group", "arm", False)
        printmenu()
        return

    # Get the user to name the positions
    name = input("Press enter to cancel recording\n"
                    + "Write the name of your position:\n")
    if name != "":
        # write ee position
        with open("robot_workspace/assets/positions/positions.py", "a") as file:
            file.write(f"\n{name}=([\n")
            numphy.savetxt(file, position_mat, fmt="  [% .8f, % .8f, % .8f, % .8f],")
            file.write("  ])\n")
        # write joint state:
        with open("robot_workspace/assets/positions/joint_states.py", "a") as file:
            file.write(f"{name}=( ")
            file.write(str(position_joints))
            file.write(" )\n")

        print(f'Successfully written "{name}" to arm_positions.py and arm_joint_states.py')

    #Reset before next move    
    printmenu()
    return


def make_on_press(bot):
    # Helper function to make the robot avaliable to the keyboard listener for future calls
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char:  # Check if the key has a 'char' attribute
                if key.char == "4":
                    printmenu()  
                elif key.char == "5":
                    print("\nExiting...")
                    return False  
                elif key.char == "1":
                    recordposition(bot)  
                elif key.char == "2":
                    playposition(bot)
                elif key.char == "3":
                    playjoints(bot)
        except AttributeError:
            # Handle special keys
            print("Keyboard listener: Caught error, no worries") # We handle it by not handling it 
            return True  # Continue listening for other keys
    return on_press  # Return the inner on_press function
   

def main():
    # boot bot
    bot =  Wafflebot( )
    bot.arm.go_to_sleep_pose()
    
    #print menu and listen for keystrokes:
    printmenu()
    on_press = make_on_press(bot)
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

    # close bot, close program.
    bot.safe_stop()
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
    
