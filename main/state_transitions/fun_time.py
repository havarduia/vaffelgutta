placeholder = 1


def fun_time(state: "State", bot: "Wafflebot"):
    if placeholder == 1:
        state.set(State.OPEN_IRON2)
    elif placeholder == 2:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
