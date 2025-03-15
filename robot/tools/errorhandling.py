# Code to be executed whenever an error is caused
from time import sleep
from interbotix_common_modules.common_robot.robot import robot_shutdown
from robot.robot_controllers import robot_boot_manager
import traceback
from rclpy import ok


def handle_error(error_program_closed_message):
    # Letsgo
    print("Error Caused. Powering down robot.")
    # Countdown before stopping the robot - to create reaction time
    try:
        for i in range(0,11):
            sleep(1)
            print("Powering down in: " + str(10-i) )
    except KeyboardInterrupt:
        print("\nCountdown skipped with ctrl+c")
    finally: 

        # Shut down robot using library
        try:
            if ok():
                robot_shutdown()
        except Exception as error_robot_shutdown_message:
                print("Error raised using robot_shutdown().\nError contents: " + str(error_robot_shutdown_message))
        # shut down robot software
        try:
            if ok:
                robot_boot_manager.robot_close()
        except NameError:
            print("Error: Program closed without valid PID")
        # Print error message  
        print("Program closed by: " + str(error_program_closed_message))
        print(traceback.format_exc())