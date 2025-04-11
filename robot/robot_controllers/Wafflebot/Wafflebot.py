from enum import Enum
from types import NoneType
from typing import Optional
from robot.robot_controllers.Wafflebot import *
from robot.robot_controllers import robot_boot_manager
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import rclpy
from time import sleep
from robot.robot_controllers.path_planner import list_multiply, list_sum
from robot.robot_controllers.Wafflebot.moveit.MotionPlanner import MotionPlanner
from camera.coordinatesystem import CoordinateSystem
from robot.robot_controllers.Wafflebot.add_collisionobjects import add_collisionobjects
from robot.robot_controllers.Wafflebot.moveit.create_collisionobjects import CollisionObjectPublisher
from rclpy.logging import LoggingSeverity
from threading import Event

from robot.tools.file_manipulation import Jsonreader
import numpy as numphy

class Wafflebot:
    def __init__(
        self,
        cam: Optional[CoordinateSystem] = None,
        automatic_mode: bool = False,
        use_rviz: bool = True,
        debug_print: bool = False,
    ):
        # Initialize robot:
        interbotix_process = robot_boot_manager.robot_launch(use_rviz=use_rviz, use_moveit = automatic_mode)

        self.bot = InterbotixManipulatorXS(
            robot_model="vx300s",
            group_name="arm",
            gripper_name="gripper",
            logging_level=LoggingSeverity.ERROR # may or may not do something
        )
        
        robot_startup()

        # Keep a look out for the emergency stop
        self.emergency_stop = Event()
        emergency_stop.run_emergency_stop_monitor(self.emergency_stop)
        # Define shorthands to call bot functions intuitively
        self.arm = self.bot.arm
        self.gripper = self.bot.gripper
        self.core = self.bot.core
        # misc inits
        self.home_pose = self.arm.robot_des.M if automatic_mode else [0]*6
        self.debug_print = debug_print
        self.speed = 1.0
        self.automatic_mode = automatic_mode

        self.motionplanner = MotionPlanner(interbotix_process)
        self.motionplanner.update_joint_states()

        if self.automatic_mode:
            if cam is None:
                raise RuntimeError("Camera is not provided for automatic mode")
            else:
                self.cam = cam
            self.collision_publisher = CollisionObjectPublisher()
    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)


    def go_to_home_pose(self):
        if self.automatic_mode:
            self.move(self.home_pose)
            start_joints = self.motionplanner.update_joint_states()
            for i in range(1, 51):
                self.arm.set_joint_positions(list_multiply(start_joints, (1 - i / 50)), blocking=False)
                sleep(0.02)
            sleep(0.2)
        else:
            self.arm.set_joint_positions(self.home_pose)

    def go_to_sleep_pose(self):
        sleep_joints = [0.0, -1.80, 1.59, 0.0, 0.5959, 0.0]
        if not self.automatic_mode:
            self.arm.set_joint_positions(sleep_joints)
            return
        start_joints = self.motionplanner.update_joint_states()
        for i in range(1, 51):
            # sleep = start + (start-sleep)*T/dt
            self.arm.set_joint_positions(
                list_sum(
                    start_joints,
                    list_multiply(
                        list_sum(sleep_joints, list_multiply(start_joints, -1)),
                        (i/50),
                    ),
                ),
            blocking=False
            )
            sleep(0.012)
        sleep(0.4)

    
    def exit(self):
        if rclpy.ok():
            self.collision_publisher.destroy_node()
            self.motionplanner.destroy_node()
            robot_shutdown()
            robot_boot_manager.robot_close()
            if rclpy.ok():
                rclpy.shutdown()

    def safe_stop(self, slow=False):
        if slow:
            self.move(self.home_pose, speed_scaling=0.1)
            self.go_to_home_pose()
            self.go_to_sleep_pose()
        else:
            self.go_to_home_pose()
            self.go_to_sleep_pose()
        self.exit()

    def move(self, target, ignore: Optional[list[str]] = None, speed_scaling: float = 1.0):
        """
        checks the input type and moves to a position.
        input can be either of:
        pose matrix - 4x4
        string - name of the pose in the recordings folder
        joints - joint states.
        """
        if self.emergency_stop.isSet():
            raise FloatingPointError # unused error used as signal.
        use_joints = not self.automatic_mode
        if self.automatic_mode:
            self.cam.start("all")
        (target, returncode) = interpret_target_command.interpret_target_command(
                target,
                use_joints,
                self.debug_print
                )
        if returncode == -1:
            raise RuntimeError("Invalid pose passed")
        elif returncode == 0:
            self.move_to_joints(None)
        elif returncode == 1:
            self.move_to_matrix(target, ignore, speed_scaling) 
        else:
            raise RuntimeError("I f-ed up. check for invalid returns in interpret_target_command.")


    def move_to_matrix(self, target: list[list[float]], ignore: Optional[list[str]], speed_scaling: float = 1.0) -> bool: 
        """
        moves the bot to a given pose matrix.
        the "move" function should be used instend for robustness.
        """
        if self.emergency_stop.isSet():
            raise FloatingPointError # unused error used as signal.
        if self.automatic_mode:
            self.cam.start("all")
            add_collisionobjects(ignore)
            success = self.collision_publisher.publish_collisionobjects() 
            if success:
                self.motionplanner.move(target, speed_scaling*self.speed)
                return self.motionplanner.movement_success
            elif self.debug_print:
                print("Wafflebot: Collision publishing failed")
            return False
        else:
            raise RuntimeError("This feature is only avaliable in automatic mode")


    def move_to_joints(self, target):
        if self.emergency_stop.isSet():
            raise FloatingPointError # unused error used as signal.
        assert (not self.automatic_mode), "This function is intended for manual mode only"
        self.arm.set_joint_positions(target)

    def clear_error(self):
        if self.emergency_stop.is_set():
            self.emergency_stop.clear()
            self.emergency_stop_pressed = False
            emergency_stop.run_emergency_stop_monitor(self.emergency_stop)

