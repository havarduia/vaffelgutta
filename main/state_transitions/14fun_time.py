from waffle_states.waffle_states import State 

from robot.robot_controllers.movements.action_header import Actions

def fun_time(state: "CurrentState", bot: "Wafflebot"):

    actions = Actions(bot)
    
    bot.move("front_of_waffle_iron")
    actions.open_waffle_iron()
    bot.move("front_of_waffle_iron")
    bot.go_to_home_pose() #Make it easier for the camera to see the tag for the sticks
    state.set(State.OPEN_IRON2)

if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
