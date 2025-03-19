from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from robot.robot_controllers import movements

from time import sleep
import numpy as numphy

def main():
    bot = Wafflebot()

    bot.arm.go_to_home_pose()

    """    
    movements.waffle_iron.open_waffle_iron(bot) 
    movements.waffle_iron.insert_sticks(bot)
    movements.lubrication.pick_up_lube(bot)
    movements.lubrication.spray_lube(bot)
    movements.lubrication.pick_up_lube(bot, reverse=True)

    sleep(3)

    movements.batter.place_cup_at_filling_station(bot,True)    
    sleep(2)
    """
    movements.batter.pick_up_cup_from_filling_station(bot)
    movements.batter.pour_batter(bot)
    movements.batter.place_cup_at_filling_station(bot, is_holding_cup=True)    
    movements.waffle_iron.open_waffle_iron(bot, reverse=True)

    sleep(5)    

    movements.waffle_iron.open_waffle_iron(bot)
    movements.waffle_iron.serve_waffle(bot)
 
 
    bot.safe_stop()
 
    
    
    
if __name__ == '__main__':
    try:
        main()
    # if error detected, run the error handler
    except (KeyboardInterrupt, Exception) as error_program_closed_message:
        handle_error(error_program_closed_message)
