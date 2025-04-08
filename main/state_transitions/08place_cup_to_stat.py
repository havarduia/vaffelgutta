from ai.timmy_detector import Timmydetector


def place_cup_to_stat(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()
    if timmy_alarm == False:
        fill_cup()
        state.set(State.FILL_CUP)
    else:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
