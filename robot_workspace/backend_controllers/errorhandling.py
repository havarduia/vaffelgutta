# Code to be executed whenever an error is caused
from time import sleep
print("Error Caused. Shutting down robot.")
# Countdown before stopping the robot - to create reaction time
for i in range(0,11):
    sleep(1)
    print("Shutting down in: " + str(10-i) ) 
# Shut down robot using library
try:
    robot_shutdown()
except Exception as error_robot_shutdown_message:
    print("Error raised using robot_shutdown().\nError contents: " + str(error_robot_shutdown_message))
finally:
# shut down robot software
    try:
        robot_boot_manager.robot_close()
    except NameError:
        print("Error: Program closed without valid PID")
    # Catch keyboard interrupt special case:
    if repr(error_program_closed_message) == "KeyboardInterrupt()":
        error_program_closed_message = "Ctrl+C (KeyboardInterrupt)"
    # Print error message  
    print("Program closed by: " + str(error_program_closed_message))