from main.waffle_states.waffle_states import State 

from robot.robot_controllers.movements.action_header import Actions

def fun_time(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)

    try:
        bot.move("front_of_waffle_iron")
        actions.open_waffle_iron()
        bot.move("front_of_waffle_iron")
        #bot.go_to_home_pose() #Make it easier for the camera to see the tag for the sticks
        state.set(State.OPEN_IRON2)
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState 
