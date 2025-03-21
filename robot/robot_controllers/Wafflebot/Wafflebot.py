from robot.robot_controllers.Wafflebot import *
from robot.robot_controllers import robot_boot_manager
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from rclpy import ok as rclpyok
from robot.tools.timekeeper import read_times, record_time

class Wafflebot:
    def __init__(self, use_real_robot : bool = False, debug_print: bool = False):
        # Include launch arguments 
        use_real_robot = use_real_robot or argumentparser.read_input_args()
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
        self.debug_print = debug_print
        self.speed = 1.0
        # Define shorthands to call bot functions intuitively 
        self.arm        = self.bot.arm
        self.gripper    = self.bot.gripper
        self.core       = self.bot.core
    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)


    def get_joint_commands(self,target: any,file: str = "None",
                           ) -> tuple[list[float] | None, bool]:
        """
        ### Converts a guess from a arbitrary input to either a set of joints or a matrix.

        :param target: The position to go to. Either of:
            \njoints
            \npose matrix
            \nposition name.

        if a position name is given, the "file" parameter must also be given.

        :param file: The name of the file to read the pose from. 
        must be in the position_data folder.

        :returns target: List of joints to get to the target position,
        or "None" if no target was found. 
        """
        target, target_type = interpret_target_command.interpret_target_command(
            target, file, self.debug_print
            )
    
        if target_type == 0:
            return None
    
        if target_type == 1:
            target = interpret_target_command.refine_guess(
                self.bot, target, self.debug_print
                ) 
        return target

    
    def exit(self):
        if rclpyok():
            robot_shutdown()  
            robot_boot_manager.robot_close()    

    
    def safe_stop(self, slow = False):
        safe_stop.safe_stop(self.bot, slow)
        self.exit()


    def move(self, target, ignore = None, blocking=True, file = None) -> bool:
        if ignore is None: ignore = []
        record_time("move_entered") 
        target_joints, return_type = interpret_target_command.interpret_target_command(
            target, file, self.debug_print
        )
        record_time("interpret_target_command")
        if return_type == 1:
            target_joints, return_type = pose_to_joints.refine_guess(
                self.bot, target_joints, self.debug_print
                )
            record_time("refine_guess")
        if return_type == 0:
            if self.debug_print:
                print("Wafflebot: Invalid movement detected")
            record_time("movement_failed")
            return False
        
        move.move(
            self.bot, target_joints,ignore,blocking,self.speed,self.debug_print
            )
        record_time("move_command")
        return None
    

    def launch_emergency_stop_monitor(self):
        emergency_stop.run_emergency_stop_monitor(self.safe_stop)