from waffle_states.waffle_states import State


def close_iron(state: "CurrentState", bot: "Wafflebot"):

    bot.go_to_home_pose() 
    do_funtime()
    state.set(State.FUN_TIME)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
