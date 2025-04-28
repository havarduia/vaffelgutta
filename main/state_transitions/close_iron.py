
from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision


def close_iron(state: "CurrentState", bot: "Wafflebot"):
    bot.go_to_home_pose()
    # if waffle_count > 0:
    #   ...

    """
        waffle_ready = Wafflereadier() #Har en bool
        
        if waffle_ready == True:
            state.set(State.FUN_TIME) 
        else:
            do_funtime() #ask for prompt and do prompt
    """       
    state.set(State.FUN_TIME)  


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
