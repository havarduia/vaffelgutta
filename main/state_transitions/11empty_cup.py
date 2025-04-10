from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def empty_cup(state: "State", bot: "Wafflebot"):

    actions = Actions(bot)
    bot.move("front_of_bowl") #TODO several tags, go to closest
    actions.place_cup() #TODO Reprogram to remove ladle

    state.set(State.RETURN_CUP)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
