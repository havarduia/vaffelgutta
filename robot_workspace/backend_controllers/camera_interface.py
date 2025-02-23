# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
# import camera
import numpy as numphy

def get_tag_from_camera(tag: str):
    return numphy.matrix(numphy.identity(4))