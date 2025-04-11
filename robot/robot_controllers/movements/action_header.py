from robot.robot_controllers.movements import *
from time import sleep

class Actions:
    def __init__(self, bot: "Wafflebot"):
        self.bot = bot

    def do_everything_and_just_make_a_waffle_for_me(self, delay: float = 0, print_actions: bool = False):
        if print_actions: print("Opening waffle iron")
        self.open_waffle_iron()
        if print_actions: print("Applying lube")
        self.apply_lube()
        if print_actions: print("Inserting sticks")
        self.insert_sticks()
        if print_actions: print("Adding batter")
        self.fill_batter_on_iron()
        if print_actions: print("Closing waffle iron")
        self.close_waffle_iron()
        if print_actions: print(f"Waiting for {delay} seconds")
        sleep(delay)     
        if print_actions: print("Opening waffle iron")
        self.open_waffle_iron()
        if print_actions: print("Serving waffle")
        self.serve_waffle()
        if print_actions: print("Putting away sticks.")
        self.put_away_sticks()   
        if print_actions: print("Closing waffle iron")
        self.close_waffle_iron()

    def open_waffle_iron(self):
        waffle_iron.open_waffle_iron(self.bot, reverse=False)    

    def close_waffle_iron(self):
        waffle_iron.open_waffle_iron(self.bot, reverse=True)
    
    def apply_lube(self):
        self.pick_up_lube()
        self.spray_lube()
        self.put_down_lube()

    def pick_up_lube(self):
        lubrication.pick_up_lube(self.bot, reverse=False)

    def spray_lube(self):
        lubrication.spray_lube(self.bot) 

    def put_down_lube(self):
        lubrication.pick_up_lube(self.bot, reverse=True)

    def insert_sticks(self):
        waffle_iron.insert_sticks(self.bot)
    
    def serve_waffle(self):
        self.take_out_waffle()
        self.serve_waffle()
    
    def take_out_waffle(self):
        waffle_iron.take_out_waffle(self.bot)
    
    def serve_waffle(self):
        waffle_iron.serve_waffle(self.bot)

    def put_away_sticks(self):
        waffle_iron.put_away_sticks(self.bot) 
    
    def fill_batter_on_iron(self):
        self.put_ladle_in_bowl()
        self.fill_cup()
        self.pick_up_ladle()
        self.pour_batter()
        self.put_ladle_in_bowl(True)

    def put_ladle_in_bowl(self, is_holding_cup):
        batter.put_ladle_in_bowl(self.bot)
    
    def fill_cup(self):
        try:
            batter.fill_cup(self.bot)
        except NotImplementedError:
            print("Not implemented yet.")

    def pick_up_ladle(self):
        batter.pick_up_ladle(self.bot)

    def pour_batter(self):
        batter.pour_batter(self.bot)

    def get_actions(self)->list[str]:
        """returns all the possible actions that have been implemented"""
        blacklist = ["bot", "get_actions"]
        methods = dir(self)
        valid_methods = []
        for method in methods:
            if not method.startswith("__"):
                valid_methods.append(method)
        [valid_methods.remove(name) for name in blacklist]
        valid_methods.sort()
        return valid_methods 

if __name__ == "__main__":
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    b = Wafflebot()
    a = Actions(b)
    from robot.tools.file_manipulation import table_print
    table_print(a.get_actions())
    b.safe_stop()


    
