from timmy import Timmydetector

def empty_cup(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()
    
    if timmy_alarm == False:
        bot.move(filling_station)
        bot.move(place_cup_fill_station)
        state.set(State.RETURN_CUP)
    else:
        state.set(State.ERROR)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
