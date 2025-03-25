from robot.robot_controllers.Wafflebot import *
from robot.robot_controllers import robot_boot_manager
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from rclpy import ok as rclpyok
from robot.tools.timekeeper import read_times, record_time
from robot.robot_controllers.Wafflebot.moveit.MotionPlanner import MotionPlanner

class Wafflebot:
    def __init__(self,
                  use_real_robot : bool = False,
                  debug_print: bool = False,
                  use_rviz : bool = True,
                  ):
        # Include launch arguments 
        use_real_robot = use_real_robot or argumentparser.read_input_args()
        # Initialize robot:
        interbotix_process = robot_boot_manager.robot_launch(use_real_robot,use_rviz)
        self.bot = InterbotixManipulatorXS(
            robot_model= "vx300s",
            group_name="arm",
            gripper_name="gripper",
        )
        self.motionplanner = MotionPlanner(interbotix_process)
        robot_startup()
        
        # Keep a look out for the emergency stop
        self.launch_emergency_stop_monitor()
        self.debug_print = debug_print
        self.speed = 1.0
        # Define shorthands to call bot functions intuitively 
        self.arm        = self.bot.arm
        self.gripper    = self.bot.gripper
        self.core       = self.bot.core
        self.home_pose = self.arm.robot_des.M
        self.sleep_pose = (
        [[ 0.83094068, 0.,          0.55636102,  0.13459297],
        [ 0.,          1.,          0.,          0.,        ],
        [-0.55636102,  0.,          0.83094068,  0.09320746],
        [ 0.,          0.,          0.,          1.        ]]) 

    # return the methods of the child class (interbotixmanipulatorxs)
    def __getattr__(self, name):
        return getattr(self.bot, name)

    def exit(self):
        if rclpyok():
            self.motionplanner.destroy_node()
            robot_shutdown()  
            robot_boot_manager.robot_close()    
    
    def safe_stop(self, slow = False):
        if slow:
            self.move(self.home_pose, speed_scaling = 0.1)
        else:
            self.move(self.home_pose)
        self.exit()

    def move(self,target, ignore = None, speed_scaling: float = 1.0):
        if ignore is None: ignore = []
        # TODO add collision objects to path
        self.motionplanner.move(target,speed_scaling)

    def launch_emergency_stop_monitor(self):
        emergency_stop.run_emergency_stop_monitor(self.safe_stop)