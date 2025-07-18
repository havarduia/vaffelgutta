
from main.waffle_states.waffle_states import State
from camera.vision import Vision

def sleepstate(state: "CurrentState", bot: "Wafflebot"):

    try:
        bot.release()
        bot.go_to_home_pose()
    except FloatingPointError: # unused error used as signal.
        state.set(State.ERROR)
        return

    state.set(State.HOME)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.state_transitions.waffle_states.waffle_states import CurrentState
