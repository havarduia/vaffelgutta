from waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

def error(state: "CurrentState", bot: "Wafflebot"):
    # bot.stop #TODO not implented 
    # This is very hard to implement on a per movement basis.
    # Should be repeatedly checked between movements instead.
    user_input = input("Do you want to continue where you stopped? (y/n) \n")
    bot.clear_error()
    
    if user_input == "y":
        ... # Gå til previous state
    elif user_input == "n":
        while True:
            try:
                state_user_input = input("Which state do you want to go to? \n")
                userstate = State[state_user_input.upper()]
                state.set(userstate)
                break
            except NameError:
                print("Invalid state, try again!")
    else:   
        bot.safe_stop(slow=True)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
