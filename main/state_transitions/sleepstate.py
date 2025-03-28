def sleepstate(state: "State", bot: "Wafflebot"):
    if not False:# TODO add some test for if the robot is closing up shop.
        pass 
    bot.arm.go_to_home_pose()
    state.set_state(State.HOME)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
