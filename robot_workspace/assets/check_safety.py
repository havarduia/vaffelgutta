from robot_workspace.backend_controllers import find_pose_from_matrix
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
def check_safety(bot: InterbotixManipulatorXS, delta_matrix):
    
    # extract pose parameters
    current_pose_matrix = bot.arm.get_ee_pose()
    target_position_matrix = delta_matrix + bot.arm.get_ee_pose()
    
    # test for floor collision
    while target_position_matrix[2][3] <= 0.03: # If z is below floor height
        # Reset to a taller place
        delta_matrix[2][3] += 0.001 
        target_position_matrix = delta_matrix + bot.arm.get_ee_pose()
        
    return delta_matrix
