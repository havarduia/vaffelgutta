from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot_workspace.assets import arm_offsets
def openwaffleiron(bot):
    bot = InterbotixManipulatorXS(bot)
    targetpos = arm_offsets.waffleiron() # + camera.get_position(waffleiron)
    bot.arm.set_ee_pose_matrix(
        
    )