from robot.tools.file_manipulation import Jsonreader
from main.waffle_states.waffle_states import State, Tags
from robot.robot_controllers.movements.action_header import Actions
from camera.vision import Vision


def pick_up_waffle(state: "CurrentState", bot: "Wafflebot", vision: "Vision"):

    #gjør noe for å servere vaffel
    actions = Actions(bot)
    actions.take_waffle_off_sticks()
    actions.insert_sticks()
    state.set(State.RETURN_STICK)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from main.waffle_states.waffle_states import CurrentState
