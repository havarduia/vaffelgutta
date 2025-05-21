
from robot.tools import maleman
from screen.screen_main import TouchScreenApp
from robot.tools.maleman import MaleMan
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from time import sleep
from threading import Thread

class  Bot:
    def __init__(self, maleman) -> None:
        self.maleman = maleman
        self.male = None

    def rxmsg(self, operation: str, msg: any):
        match operation:
            case "collision_detected_response":
                self.male = msg
            case _:
                print(f"no operation found for {operation}")
                print(f"message contents: {str(msg)}")

    def empty_malebox(self):
        self.male = None
    def read_male(self):
        while self.male is None:
            print("waiting...")
            sleep(0.5)
        male = self.male
        self.empty_malebox()
        return male 

    def test_male(self):
        for i in range(6):
            sleep(1)
            print("starting in "+str( (5-i)))
        self.maleman.send_male("screen", "manual_mode_collision", ["boxbox", "objectivebox"])
        execute_movement = self.read_male() 
        sleep(1)
        self.maleman.send_male("screen", "show_message", f"You pressed {execute_movement}")

def main():
    maleman = MaleMan()
    bot = Bot(maleman)
    screen = TouchScreenApp(maleman)
    maleman.register("robot",bot)
    maleman.register("screen", screen)
    st = Thread(target = bot.test_male, daemon=True).start()
    screen.mainloop()

    

if __name__=="__main__":
    main()
