from timmy import Timmydetector

def fun_time(state: "State", bot: "Wafflebot"):
    
    timmy_alarm = Timmydetector()
    
    if timmy_alarm == False:
        bot.move(iron)
        bot.move(open_iron)
        bot.move(iron)
        state.set(State.OPEN_IRON2)
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
