"""
This script is used to convert a 4x4 pose matrix
into its components.
This conversion allows use of the built-in
set_cartesian_trajectory function that is recommended
by the interbotix website
https://docs.trossenrobotics.com/interbotix_xsarms_docs/python_ros_interface.html#tips-best-practices

This approach was scrapped due to frequent runtime errors
causing the robot to cancel movement.
It is kept as legacy code,
as a surprise tool to help us later 
"""
import numpy as numphy
class Coordinatesystem:
    x = None,
    y = None,
    z = None,
    roll = None,
    pitch = None,
    yaw = None
    all = None 

from interbotix_common_modules import angle_manipulation
#  Greg code to find the Delta matrix given two matrices

def compute_relative_pose(T_start, T_target):
    T_delta = angle_manipulation.trans_inv(T_start) @ T_target - numphy.identity(4)
    return T_delta

def find_pose_from_matrix(delta_matrix: list):
    target_positions = Coordinatesystem()
    
    rotation_matrix = numphy.array([
     [   delta_matrix[0][0], delta_matrix[0][1], delta_matrix[0][2]],
     [   delta_matrix[1][0], delta_matrix[1][1], delta_matrix[1][2]],
     [   delta_matrix[2][0], delta_matrix[2][1], delta_matrix[2][2]],
    ]) 
    print("debug:")
    print(angle_manipulation.rotation_matrix_to_euler_angles(rotation_matrix))
    
    print(angle_manipulation.rotation_matrix_to_euler_angles(
            angle_manipulation.euler_angles_to_rotation_matrix(
                angle_manipulation.rotation_matrix_to_euler_angles(
                    rotation_matrix))
        )
    )
    # convert rotation matrix to its roll/pitch/yaw components
    (roll, pitch, yaw) = angle_manipulation.rotation_matrix_to_euler_angles(rotation_matrix)
    print(([roll,pitch,yaw]))
    print("")
    # Enter the components in a struct    
    target_positions.x =float(delta_matrix[0][3])
    target_positions.y =float(delta_matrix[1][3])
    target_positions.z =float(delta_matrix[2][3])
    target_positions.roll = roll 
    target_positions.pitch = pitch 
    target_positions.yaw = yaw
    target_positions.all = numphy.array([
        target_positions.x,
        target_positions.y,
        target_positions.z,
        target_positions.roll,
        target_positions.yaw,
        target_positions.pitch,
        ])
    print("debug 2")
    print(numphy.matrix(angle_manipulation.pose_to_transformation_matrix(target_positions.all)))
    print(delta_matrix)    
    return target_positions


