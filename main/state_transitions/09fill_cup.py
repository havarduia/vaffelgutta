#TODO kill me 👿
from waffle_states.waffle_states import State 
from robot.robot_controllers.movements.action_header import Actions

def fill_cup(state: "CurrentState", bot: "Wafflebot"):


        except FloatingPointError: # unused error used as signal.
            state.set(State.ERROR)
            return
    actions = Actions(bot)
    bot.move("front_of_filling_station")
    actions.pick_up_cup()
    bot.move("front_of_waffle_iron")
    state.set(State.CUP_TO_IRON)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
