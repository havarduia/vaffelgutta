""" functions to generate bounding boxes"""
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as numphy

def _bb_from_endpoints(min, max):
    (xm,ym,zm) = min
    (xp,yp,zp) = max
    return[
    [xm,ym,zm], 
    [xm,ym,zp],
    [xm,yp,zm],
    [xm,yp,zp],
    [xp,ym,zm],
    [xp,ym,zp],
    [xp,yp,zm],
    [xp,yp,zp]
    ]

def _endpoints_from_bb(corners):
    min = [1e4, 1e4, 1e4]
    max = [0,0,0]
    # make sure the endpoints are the max of the rotated endpoints for every iteration and axis
    for corner in corners:
        if corner[0] < min[0]: min[0] = corner[0]
        if corner[1] < min[1]: min[1] = corner[1]
        if corner[2] < min[2]: min[2] = corner[2]
        if corner[0] > max[0]: max[0] = corner[0]
        if corner[1] > max[1]: max[1] = corner[1]
        if corner[2] > max[1]: max[2] = corner[2]
    return(min, max)

def _rotate_cube_by_waist_angle(corners, waist_direction):
    for corner in corners:
        corner[0] = corner[0]*numphy.cos(waist_direction)
        corner[1] = corner[1]*numphy.sin(waist_direction)
        #corner[2] = corner[2]
    return corners

def update_robot_bounding_box(
        bot: InterbotixManipulatorXS,
        joints: list
        ) -> None:
    base_origin = [0,0,0]

    if len(joints) != 6:
        print("Error in bounding boxes -> robot_bounding box:\n"
              +"Please input a list of joint positions")
        return
    
    joint_lengths = [
        4.5, # base to shoulder
        30,  # shoulder to arm_link z
        6.5, # shoulder to arm_link x
        30,  # arm_link to wrist
        6.5  # wrist to gripper
        ]/100 # convert cm to m

    "Waist direction:"
    waist_direction = joints[0]
        

    "Shoulder:"
    # Set shoulder position
    shoulder_origin = base_origin
    shoulder_origin[2] += joint_lengths[0]
    shoulder_dimensions = [9,5,6]/100 # measurements of shoulder fixture,shoulder_min,shoulder_max
    shoulder_offset = - shoulder_dimensions/2 # initialize with origin in center
    shoulder_offset[2] = -4.5e-2 # z component measured by hand
    # Determine corners
    shoulder_min = shoulder_origin+shoulder_offset # minimum point is equal to offset
    shoulder_max = shoulder_min + shoulder_dimensions # define max point of bocxs

    #calculate bounding box
    shoulder_corners = _bb_from_endpoints(shoulder_min, shoulder_max) #todo: make this function
    shoulder_corners = _rotate_cube_by_waist_angle(shoulder_corners,waist_direction)
    (shoulder_min,shoulder_max) = _endpoints_from_bb(shoulder_corners)

    # TODO write bounding box to file...
    

    "First arm joint"
    # Declare arm dimensions
    arm_1_origin = shoulder_origin
    arm_1_dimensions = [2, 4, 33]/100 # hand measurements
    arm_1_offset = -arm_1_dimensions/2 # x, y offsets are half of the arm size
    arm_1_offset[2] = 0 # the motor is the origin from the z axis
    # Determine corners
    arm_1_min = arm_1_origin + arm_1_offset
    arm_1_max = arm_1_min + arm_1_dimensions
    # locate endpoints
    arm_1_radius = numphy.cos(joints[1])*joint_lengths[1] # projection of the arm radius on the xy-plane - orientation agnostic
    arm_1_height = numphy.sin(joints[1])*joint_lengths[1] # z component
    #arm_1_angle = waist_direction

    # Iterate over multiple segments of the arm to reduce error.
    arm_subdivisions = 10
    DT = arm_subdivisions
    for dt in range(arm_subdivisions): 
        #initialize arm positions
        subdivision_start = arm_1_min; subdivision_end = arm_1_min
        subdivision_end[1] = arm_1_dimensions[1] #the width (y) of arm1 is always 4cm
        #create subdivisions
        subdivision_start[1] += arm_1_radius*dt/DT     # start x
        subdivision_start[2] += arm_1_height[2]*dt/DT  # start y
        subdivision_end[1] += arm_1_radius*(dt+1)/DT   # end x
        subdivision_end[2] += arm_1_height[2]*(dt+1)/DT# end y
        # create a cube and apply rotation
        subdivision_corners = _bb_from_endpoints(subdivision_start,subdivision_end)
        subdivision_corners = _rotate_cube_by_waist_angle(subdivision_corners)
        (subdivision_start, subdivision_end) = _endpoints_from_bb(subdivision_corners)

        #todo save positions

    "Arm link"
    # origin set at the dynamixel servo to arm 2
    arm_link_origin = shoulder_origin
    arm_link_origin[0] += joint_lengths[2]         

    
