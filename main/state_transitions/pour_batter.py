from main.waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def pour_batter(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)
    try:
        bot.move("prep_ladle") 
        actions.put_ladle_in_bowl() 
        bot.move("prep_ladle")
        state.set(State.RETURN_LADLE)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return



if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
