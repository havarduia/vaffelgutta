# set working directory:
from executable_scripts.common.directory_fixer import fix_directiory
fix_directiory()
###
from robot.robot_controllers.Wafflebot import Wafflebot
from robot.robot_controllers import movements

class Waffle_making_system:
    def __init__(self):
        pass
def open_waffle_iron(bot: Wafflebot):
    raise NotImplementedError("this function not yet made")

def close_waffle_iron(bot:Wafflebot):
    raise NotImplementedError("this function not yet made")