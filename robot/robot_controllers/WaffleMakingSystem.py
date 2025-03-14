from robot.tools.errorhandling import handle_error


class WaffleMakingSystem:
    def __init__(self, bot: "Wafflebot"):
        self.state = 0 # todo make an enum 
        self.waffles_to_be_made: int = 0
        self.waffles_made: int = 0

    def add_waffles(self, count: int):
        max_waffles = 5
        new_waffle_count = self.waffles_to_be_made + count
        if new_waffle_count <= max_waffles: 
            self.waffles_to_be_made = new_waffle_count
        else:
            print("Too many waffles. Please ask for less.\n"+
                  f"Max: {max_waffles}\n"+
                  f"Requested total:{new_waffle_count}")
    

def main():
    pass
    # test functionalities

if __name__ == "__main__":
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    try:
        main()
    except Exception as error:
        handle_error(error)    