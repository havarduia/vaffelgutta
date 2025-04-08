from timmy import Timmydetector


def return_cup(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()
    
    if timmy_alarm == False:
        bot.move(iron)
        bot.move(close_iron)
        bot.move(home)
        state.set(State.CLOSE_IRON)
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
