from tkinter import Event
from typing import Callable
from robot.tools.maleman import MaleMan


class screen:
    def __init__(self, maleman: MaleMan) -> None:
        self.maleman = maleman
    # placeholder used to give other objects access to MaleMan.
    # fix this in whatever way is most convenient. (pass maleman as parameter?)

    def popup_manual_movement_choice(self,msg):
        # msg = [robotbox, targetbox]
        # create a popup on the screen
        # transmit response to robot
        botbox, objbox = msg
        ######
        #create_popup for this:
        print(f"""
        Collision detected!
        {botbox}
        has collided with
        {objbox}.
        Proceed anyway?
        """)
        action: bool = input()# yes (True) or no (False)
        #####

        self.maleman.txmsg("robot", "collision_detected_response", action)    


    def rxmsg(self, operation, msg):
        match operation:
            case "manual_mode_collision":
                self.popup_manual_movement_choice(msg)



    
