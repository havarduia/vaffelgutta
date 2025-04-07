from robot.tools.file_manipulation import Jsonreader
from ai.timmy_detector import Timmydetector


def open_iron(state: "State", bot: "Wafflebot"):
    timmy_alarm = Timmydetector()
    reader = Jsonreader()

    reader.pop("camera_readings", "1")
    bot.camera_start()
    tags = reader.read("camera_readings")

    if timmy_alarm == False:

        if 1 in tags.keys():
            jern_lukket = True

        if jern_lukket == True:
            bot.move(jern)
            bot.sequence(
                open_iron_plan
            )  # Trenger ikke å være en function, bare en lettere måte å si at
            # du må gjøre flere bevegelser.
            jern_lukket = False
            state.set(State.OPEN_IRON)
        else:
            state.set(State.OPEN_IRON)
    else:
        state.set(State.ERROR)


if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot import Wafflebot
    from waffle_states.waffle_states import State
