# Import only if if running on Jetson
from sys import modules as sysmodules
if "Jetson.GPIO" in sysmodules:
    import Jetson.GPIO as GPIO

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from robot.robot_controllers import robot_boot_manager, path_planner, safety_functions
from robot.tools.file_manipulation import Jsonreader

from argparse import ArgumentParser
from threading import Thread
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
        self.run_emergency_stop_monitor()
        # Set print level:
        self.debug_print = debug_print
        # Define shorthands to call bot functions intuitively 
        self.arm        = self.bot.arm
        self.gripper    = self.bot.gripper
        self.core       = self.bot.core

    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)

    def __del__(self):
        self.exit()
    
    def _refine_guess(self,target):
        """
        Refines the guessed position into a position that is hopefully less awward for the arm to reach.
        """
        bot = self.bot
        self.arm.capture_joint_positions()
        current_pose = self.arm.get_joint_positions()
        # Try using current position as seed for target joints. Else retry with vanilla guesses
        (target_joints, success) = path_planner.plan_matrix(bot, target, current_pose)
        if not success:
            target_joints, success = path_planner.plan_matrix(bot, target)
        # error checking
        if not success:
            if self.debug_print:
                print("Wafflebot: move failed after first fix joints - aborting")
            return (None, False)
        # For positions that have made over a quarter rotation: 
        # Might cause double rotation --> awkward position.
        # So retry with forced upright position. Waist ([0]) is exempt this problem.
        prev_target = target_joints.copy()
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0        
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints) 
        if not success:
            if self.debug_print:
                print("Wafflebot: move failed after second fix joints - using previous guess")
            target_joints = prev_target
        # if the shoulder is bent back and the elbow is pointing up,
        # it is likely another unnatural position that should be fixed
        prev_target = target_joints.copy()
        if (
            target_joints[1] < -(numphy.pi/6)   # if shoulder is bent back 
            and not target_joints[2] > 0.1      # and elbow is not pointing somewhat foreward
        ):   
            if self.debug_print:
                print(f"Adjusted from state:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
            # turn around waist (joint overflow will be handled down the line)
            target_joints[0] +=numphy.pi           
            # reset shoulder and elbow
            target_joints[1] = 1e-6  
            target_joints[2] = 1e-6
            if self.debug_print:
                print(f"to:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
            adjusted = True
        else:
            adjusted = False
        # refine adjusted target pose        
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints) 
        if not success:
            if self.debug_print:
                print("Wafflebot: move failed after third fix joints - using previous guess")
            target_joints = prev_target
        # since the position has changed, redo the first uprightness test.
        prev_target = target_joints.copy()
        for i in range(1,6):
            if abs(target_joints[i]) > numphy.pi/2:
                target_joints[i] = 0.0001
        # refine final target pose
        target_joints, success = path_planner.plan_matrix(bot, target, target_joints)        
        if not success:
            if self.debug_print:
                print("Wafflebot: move failed after fourth fix joints - using previous guess")
            target_joints = prev_target
        if adjusted:
            if self.debug_print:
                print(f"the adjusted joints are:\n{target_joints[0]}\n{target_joints[1]}\n{target_joints[2]}")
        return target_joints, True
    
    def _interpret_target_command(self,
                                target: any,
                                file: str = "None",
                                ) -> tuple[list[float] | None, bool]:
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
        target_joints, success = self._refine_guess(target)
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
        # Todo? add blocking = False?
        start_joints = self.arm.get_joint_positions()
        target_joints, success = self._interpret_target_command(target, file)

        if not success:
            if self.debug_print:
                print("Wafflebot:\Could not plan movement.")
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



    def run_emergency_stop_monitor(self):
        if "Jetson.GPIO" in sysmodules: # Check if running on Jetson
            # **Start GPIO monitoring in a separate thread**
            self.gpio_thread = Thread(
                target=self.monitor_emergency_stop, daemon=True)
            self.gpio_thread.start()
        return
    
    def monitor_emergency_stop(self):
        """ Function to monitor GPIO button in a separate thread. """
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
    












    '''
    # Deprecated functions
    def big_movement(self, target: str, target_position_matrix = None): # todo: add support to convert variables to string

        """
        moves the bot to a faraway place. requires a preset waypoint in joint space
        A list of joint states are stored in assets/arm_joint_states.py
        :input: joint_state_target: The joint state to go to
        :input: target_position_matrix: optional parameter to adjust waist position to point towards the given direction  
        """
        print("Warning: big_movement is deprecated. please use move() instead.")
        import_reload(joint_states)
        joint_name = target # save name for future use
        target = self._interpret_target_command(target)
        
        # If a target matrix is given, adjust the joint first  
        if target_position_matrix != None:
            
            # To ensure compatibility - cast to a numphy matrix
            target_position_matrix = numphy.matrix(target_position_matrix)             
            # extract the angle of the position
            target_x = target_position_matrix[3,0]
            target_y = target_position_matrix[3,1]
            target_rad = numphy.atan2(target_y, target_x)
            # commit the position    
            target[0] = target_rad
        
        # Check the feasability of the movement 
        target = safety_functions.fix_joint_limits(target)
        # If an error was raised, abort  """movement
        if target[0] == False: 
            print("big_movement: joint positions not reachable - aborting movement")
        # Else, move.
        else:
            self.arm.set_joint_positions(target)
            self.small_movement(joint_name)
    
    
    def small_movement(self, target): # todo: add support to convert variables to string
        print("warning: small_movement is depreceated. please use move() instead.")
        if isinstance(target, str):
            target = getattr(positions,target)
        if not isinstance (target,list):
            print ("Wafflebot - Small movement: target position is not a valid type")
            return False # error

        self.arm.capture_joint_positions() # in hopes of reminding the bot not to kill itself with its next move

        waypoints = self.arm.set_ee_pose_matrix(target)        


        for waypoint in waypoints:
            joints = self.arm.get_joint_positions()
                        
            joints = self.arm.set_ee_pose_matrix(
                waypoint,
                execute=False,
                custom_guess=joints
                )[0]            
            joints = safety_functions.fix_joint_limits(joints=joints)

  
            if joints[0] == False:
                print("small_movement failed.")
                return
                #todo error handling

            
            # if joint values are "out of wack", retry with upright-er positions 
            for i in range(1,6):
                if abs(joints[i]) > numphy.pi/2:
                    joints[i] = 0.0
            
            joints = self.arm.set_ee_pose_matrix(
                waypoint,
                execute=False,
                custom_guess=joints
                )[0]
            joints = safety_functions.fix_joint_limits(joints=joints)            

            if joints[0] == False:
                print("Small_movement failed.")
                return
                #todo error handling
            
            self.arm.set_joint_positions(joints)
        return
    '''