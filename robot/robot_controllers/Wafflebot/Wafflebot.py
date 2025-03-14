# Import only if if running on Jetson
from sys import modules as sysmodules
if "Jetson.GPIO" in sysmodules:
    import Jetson.GPIO as GPIO

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from robot.robot_controllers import robot_boot_manager, path_planner, safety_functions
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.Wafflebot.emergency_stop import run_emergency_stop_monitor
from robot.robot_controllers.Wafflebot import pose_to_joints
from robot.tools.visualizers.tf_publisher import TFPublisher

from argparse import ArgumentParser
from functools import partial
from rclpy import ok as rclpyok
from time import sleep
import numpy as numphy

def read_input_args():
    parser = ArgumentParser(description="Runs a wafflebot")
    parser.add_argument("-r", required=False, type=int, default=0, help="1 to use real robot, 0 to simulate")
    args = parser.parse_args() 
    return bool(args.r)

class Wafflebot:
    def __init__(self, use_real_robot : bool = False, debug_print: bool = False):
        # Include launch arguments 
        use_real_robot = use_real_robot or read_input_args()
        # Initialize robot:
        robot_boot_manager.robot_launch(use_real_robot)
        self.bot = InterbotixManipulatorXS(
            robot_model= "vx300s",
            group_name="arm",
            gripper_name="gripper",
            )
        robot_startup()
        # Keep a look out for the emergency stop
        self.launch_emergency_stop_monitor()
        # Set print level:
        self.debug_print = debug_print
        # Define shorthands to call bot functions intuitively 
        self.arm        = self.bot.arm
        self.gripper    = self.bot.gripper
        self.core       = self.bot.core
    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)


    def get_joint_command_from_matrix(self,
                                target: any,
                                file: str = "None",
                                ) -> tuple[list[float] | None, bool]:
        """
        Future replacement to merge interpret_target_command and refine_guess
        """
        raise NotImplementedError

    def refine_guess(self,target) -> tuple[list[float] | None , bool]:
        """
        Refines the guessed position into a position that is hopefully less awkward for the arm to reach.
        """    
        target_joints, success = pose_to_joints.refine_guess(self.bot, target, self.debug_print)   
        return (target_joints, True) if success else (None, False)
    
    def _interpret_target_command(self,
                                target: any,
                                file: str = "None",
                                ) -> tuple[list[float] | None, bool]:
        """
        Converts a guess ino
        """
        if (isinstance(target, list)) and (len(target) == 6):
            return (target, True)
        elif isinstance(target, numphy.matrix):
            target = target.tolist()
        elif isinstance(target, str):
            if file.lower() == "none":
                if self.debug_print:
                    print("Wafflebot: tried to interpret name without file input")
                return (None, False)
            positions = Jsonreader().read(file)
            try: 
                return (positions[target]["matrix"], True)
            except KeyError:
                if self.debug_print:
                    print(f"{target} not in {file}")
                return(None, False)
        else:
            if self.debug_print:
                print("Wafflebot: Unsupported command type.")
            return (None, False)
        target_joints, success = self.refine_guess(target)
        return (target_joints, success)
    
    def exit(self):
        if rclpyok():
            robot_shutdown()  
            robot_boot_manager.robot_close()    
    
    def safe_stop(self, slow = False):
        self.arm.set_trajectory_time(moving_time=(8.0 if slow else 2.0)) # reset moving time if changed elsewhere
        self.bot.core.robot_torque_enable("group", "arm", True)
        sleep(2)
        self.arm.go_to_home_pose()
        sleep_joints =  [0.0, -1.80, 1.6, 0.0, 0.5859, 0.0]
        self.arm.set_joint_positions(sleep_joints)
        sleep(0.5)
        self.exit()
    
    def cancel_movement(self):
        current_pose = self.arm.get_ee_pose()
        self.arm.set_ee_pose_matrix(current_pose)

    def move(self,
            target, 
            ignore: list[str] = [],
            blocking: bool = True, 
            file: str = "None",
            speed_scaling: float = 1.0, 
            ) -> None:
        deleteme = TFPublisher()
        try:
            target2 = target.tolist()
            deleteme.broadcast_transform(list(target2))
        except:
            pass
        # Todo? add blocking = False?
        start_joints = self.arm.get_joint_positions()
        target_joints, success = self._interpret_target_command(target, file)
        if not success:
            if self.debug_print:
                print("Wafflebot:\nCould not plan movement.")
            return False    
        waypoints, success = (path_planner.plan_path(self, start_joints, target_joints, ignore, []))

        if not success: 
            if self.debug_print:
                print("Wafflebot: move failed after path planner")
            return False
        if not isinstance(waypoints, list):
            return False
        
        if len(waypoints) == 1:
            try: 
                if len(waypoints[0]) == 1:
                    return False
            except TypeError:
                if self.debug_print:
                    print ("TypeError for testing len(waypoints[0])==1")
        
        speedconstant = 0.42066638
        prev_waypoint = start_joints
        from robot.robot_controllers.path_planner import _list_sum, _list_multiply
        for waypoint in waypoints:

            joint_travel_distance =_list_sum(waypoint, _list_multiply(prev_waypoint,-1)) 
            wp_path_length = max(path_planner.calculate_biggest_joint(joint_travel_distance), 1e-8)
            speed = (speedconstant * speed_scaling / wp_path_length) 
            min_move_time = 0.314159265
            move_time = max(1.0/speed, min_move_time)
            self.arm.set_joint_positions(waypoint,moving_time=move_time,blocking=False)
            prev_waypoint = waypoint
            if blocking:
                # todo make interruptible. (e. stop etc)
                sleep(move_time)

    def launch_emergency_stop_monitor(self):
        run_emergency_stop_monitor(self.safe_stop)


if __name__ == "__main__":
    b = Wafflebot(0)
    b.launch_emergency_stop_monitor()()
    for i in range(20):
        print("main is running")
        sleep(2)

    """
    def run_emergency_stop_monitor(self):
        if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
            # **Start GPIO monitoring in a separate thread**
            self.gpio_thread = Thread(
                target=self.monitor_emergency_stop, daemon=True)
            self.gpio_thread.start()
        return
    
    def monitor_emergency_stop(self):
        "#"" Function to monitor GPIO button in a separate thread. "#""
        # Set the GPIO mode
        GPIO.setmode(GPIO.BOARD)
        button_pin = 18  # Define button pin
        # Set the pin as an input
        GPIO.setup(button_pin, GPIO.IN)
              
        while True:
            pin_state = GPIO.input(button_pin)
            if pin_state == GPIO.LOW:
                self.safe_stop(slow = True)
                break
            sleep(0.1)  # Prevent CPU overuse
    """