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

from interbotix_common_modules import angle_manipulation
#  Greg code to find the Delta matrix given two matrices

def compute_relative_pose(T_start, T_target):
    T_delta = numphy.linalg.inv(T_start) @ T_target
    return T_delta

def find_pose_from_matrix(trajectory_matrix: list):
    
    target_positions = Coordinatesystem()
    
    rotation_matrix = numphy.array([
     [   trajectory_matrix[0][0], trajectory_matrix[0][1], trajectory_matrix[0][2]],
     [   trajectory_matrix[1][0], trajectory_matrix[1][1], trajectory_matrix[1][2]],
     [   trajectory_matrix[2][0], trajectory_matrix[2][1], trajectory_matrix[2][2]],
    ]) 

    # convert rotation matrix to its roll/pitch/yaw components
    (roll, pitch, yaw) = angle_manipulation.rotation_matrix_to_euler_angles(rotation_matrix)

    # Enter the components in a struct    
    target_positions.x = trajectory_matrix[0][3]
    target_positions.y = trajectory_matrix[1][3]
    target_positions.z = trajectory_matrix[2][3]
    target_positions.roll = yaw 
    target_positions.pitch = pitch 
    target_positions.yaw = roll

    target_positions = [target_positions.x, target_positions.y, target_positions.z, target_positions.yaw, target_positions.pitch, target_positions.roll]
    return target_positions


