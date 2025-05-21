
from robot.tools import maleman
from screen.screen_main import TouchScreenApp
from robot.tools.maleman import MaleMan
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from time import sleep
from threading import Thread

def main():
    maleman = MaleMan()
    bot = Wafflebot(maleman, automatic_mode=False)
    screen = TouchScreenApp(maleman)
    maleman.register("robot",bot)
    maleman.register("screen", screen)
    st = Thread(target =screen.mainloop, daemon=True).start()

    for i in range(6):
        print("starting in"+str( (5-i)))
    bot.test_male()
    

if __name__=="__main__":
    main()
