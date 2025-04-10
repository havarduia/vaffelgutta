from waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions

# TODO 
# placeholder = 1


def serve_waffle(state: "CurrentState", bot: "Wafflebot"):
    actions = Actions(bot)
    reader = Jsonreader()
    reader.clear("camera_readings")
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    
    success = False
    
    if "2" in tags.keys():
        success = True

    else:   
        bot.go_to_home_pose() # May have waypoints for this. moveit might save us.
        bot.cam.start("all")
        tags = reader.read("camera_readings")
        if "2" in tags.keys():    
            success = True      
            
    if success == True:
        bot.go_to_home_pose() # May have waypoints for this. moveit might save us.
        bot.move("front_of_waffle_iron")
        actions.insert_sticks()
        bot.move("front_of_waffle_iron")
        bot.go_to_home_pose()
        state.set(State.RETURN_STICK)
    else:   
        state.set(State.ERROR)
        
if __name__ == "__main__":
    # to resolve type annotation
    from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
    from waffle_states.waffle_states import CurrentState 
