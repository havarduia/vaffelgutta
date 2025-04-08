from timmy import Timmydetector


def close_iron(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()
    
    if timmy_alarm == False:
        do_funtime()
        state.set(State.FUN_TIME)
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
