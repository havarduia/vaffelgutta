from os import getcwd
import numpy as numphy


def convert_box(box):
    lower_corner = box[0]
    higher_corner = box[1]
    center =[lower_corner[ind] +1*(higher_corner[ind] - lower_corner[ind])/2 for ind in range(len(lower_corner))]
    length = higher_corner[0] - lower_corner[0]
    width = higher_corner[1] - lower_corner[1]
    height = higher_corner[2] - lower_corner[2]

    box = {
        "x":center[0],
        "y":center[1], # y left when arm forward
        "z":center[2],
        "depth": height,
        "width": length,
        "height": width
        #"depth":0.005,# z
        #"width":0.005,# x
        #"height":0.005# y
    }
    return box
    
def read_boxes():
    path = getcwd()
    path += "/robot/assets/boundingboxes/robot.py"
    with open(path,"r") as file:
        box_list: dict = eval(file.read(), {"np":numphy})
    boxes = []  
    for key in box_list.keys():
        boxes.append(convert_box(box_list[key]))
    return boxes



