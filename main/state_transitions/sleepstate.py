
from waffle_states.waffle_states import State

def sleepstate(state: "CurrentState", bot: "Wafflebot"):

    try:
        bot.go_to_home_pose()
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return

    state.set(State.HOME)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState
