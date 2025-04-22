
from robot.tools import maleman
from screen.screen_main import TouchScreenApp
from robot.tools.maleman import MaleMan
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from time import sleep

def main():
    maleman = MaleMan()
    bot = Wafflebot(maleman, automatic_mode=False)
    screen = TouchScreenApp(maleman)
    maleman.register("robot",bot)
    maleman.register("screen", screen)
    screen.mainloop()
    sleep(5)
    bot.test_male()
    

if __name__=="__main__":
    main()
