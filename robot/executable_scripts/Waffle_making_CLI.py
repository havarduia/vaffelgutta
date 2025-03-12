from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot import Wafflebot
from robot.robot_controllers.movements.action_header import Actions
from robot.tools.file_manipulation import table_print

def printmenu():
    print("")
    print("Welcome to the actions tester.")
    print("Press 1 to perform an action")
    print("Press 2 to go to home pose")
    print("Press 3 to exit.")
    return None

def do_an_action(bot: Wafflebot):
    actions = Actions(bot)
    list_of_actions = actions.get_actions()
    print("The possible actions are:")
    table_print(list_of_actions)
    user_action = input("input the action you want to test\nInput: ")    
    if user_action in list_of_actions:
        action_as_function = "actions."+user_action+"()"
        try:
            eval(action_as_function)
        except TypeError:
            print("This function takes arguments. Please input True/False:")
            arguments = input("Input: ")
            if arguments.lower() == "false":
                arguments = False
            else: 
                arguments = True
            action_as_function = action_as_function[:-1] + str(arguments) + ")"
            eval(action_as_function)
    else:
        print(f"No action {user_action}.\n" +
              "Maybe try something more Real™️ next time.")
 
def main():
    bot = Wafflebot()
    bot.arm.go_to_home_pose(blocking=False)
    while True:
        printmenu()
        choice = input("Input: ")
        try:
            choice = int(choice)
        except ValueError:
            print("That was not a number😡")
        match choice:
            case 1:
                do_an_action(bot)
            case 2:
                bot.arm.go_to_home_pose()
            case 3:
                print("hello")
                break
            case _:
                print(f"invalid input {choice}. Try again.")
    bot.safe_stop()
   
if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        handle_error(error) 