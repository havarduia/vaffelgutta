import cv2
import numpy as np
import mediapipe as mp
import json
import os
from time import sleep
from threading import Thread
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.tools.errorhandling import handle_error
from rclpy._rclpy_pybind11 import RCLError
from robot.tools.file_manipulation import Jsonreader
from camera.realsense import Camera
# Import your new class – adjust the import if the HandDetector class is in a different module
from ai.hand_detection import HandDetector
from camera.Config.misc import ConfigLoader


def move_robot_to_hand(bot: Wafflebot, reader: Jsonreader, json_filename: str):
    """
    Reads the transformation matrix saved by the HandDetector from the JSON file and commands the robot 
    to move to the extracted hand position (translation part of the 4x4 matrix).
    """
    data = reader.read(json_filename)
    if not data or "hand" not in data:
        print("No hand transformation data available. Please run hand detection first.")
        return
    transform = np.array(data["hand"])
    target_position = transform[0:3, 3]
    print(f"Moving robot to hand position: {target_position}")
    # You can use bot.move() or any other movement command as appropriate.
    success = bot.move(target_position, speed_scaling=4.0)
    print(f"Movement command issued. Success? {success}")


def print_menu():
    menu_text = (
        "Menu Options:\n"
        "1. Start hand detection (if not already running)\n"
        "2. Command robot to move to hand position\n"
        "3. Toggle arm torque\n"
        "4. Exit\n"
    )
    print(menu_text)


def main(bot: Wafflebot, camera, json_reader: Jsonreader, hand_detector: HandDetector):
    """
    Main interactive loop. Option 1 will start the hand detection thread,
    Option 2 reads the saved hand transform and moves the robot,
    Option 3 toggles the robot arm torque, and Option 4 exits.
    """
    running_hand_detection = False
    hand_detection_thread = None
    torqued = True

    while True:
        print_menu()
        choice = input("Select an option: ")

        try:
            choice = int(choice)
        except ValueError:
            print("Invalid input. Please enter a number from the menu.")
            continue

        # Use Python’s structural pattern matching (requires Python 3.10+)
        match choice:
            case 1:
                if not running_hand_detection:
                    hand_detection_thread = Thread(target=hand_detector.start, daemon=True)
                    hand_detection_thread.start()
                    running_hand_detection = True
                    print("Started hand detection thread.")
                else:
                    print("Hand detection is already running.")
            case 2:
                move_robot_to_hand(bot, json_reader, "hand_detection")
            case 3:
                bot.core.robot_torque_enable("group", "arm", not torqued)
                torqued = not torqued
                print(f"Arm torque toggled. Now torque enabled: {torqued}")
            case 4:
                print("Exiting program...")
                break
            case _:
                print("Invalid input. Try again.")

    # Clean up and safely stop the robot
    bot.safe_stop()


if __name__ == '__main__':
    bot = None
    try:
        # Initialize the camera system.
        # (Assumes init_camera returns (camera_display, throwaway_variable, camera_coordsys)
        # and that camera_coordsys supports the get_image(), get_depth_image(), and get_calibration() methods.)
        config_loader = ConfigLoader()
        camera = Camera(config_loader)

        # Initialize the robot – here we assume that the robot uses the camera with depth capabilities.
        bot = Wafflebot(camera, use_rviz=False)

        # Create a JSON reader instance for reading/writing transformation matrices.
        json_reader = Jsonreader()

        # Instantiate the HandDetector with the camera and JSON writer information.
        # The JSON data will be stored in a file identified by "hand_detection".
        hand_detector = HandDetector(camera, json_reader, "hand_detection")

        # Start the main control loop.
        main(bot, camera, json_reader, hand_detector)
        bot.exit()

    except (Exception, KeyboardInterrupt, RCLError) as e:
        handle_error(e, bot)
