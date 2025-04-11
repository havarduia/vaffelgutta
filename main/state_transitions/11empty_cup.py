from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def empty_cup(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)
    try:
        bot.move("front_of_bowl") #TODO several tags, go to closest
        actions.put_ladle_in_bowl() #TODO Reprogram to remove ladle
        state.set(State.RETURN_CUP)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return



if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
