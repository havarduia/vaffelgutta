from ai.timmy_detector import Timmydetector


def fill_cup(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()

    if timmy_alarm == False:
        bot.move(bottle)
        bot.move(pick_up_bottle)
        bot.move


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
