from main.waffle_states.waffle_states import State 
from robot.tools.file_manipulation import Jsonreader
from robot.robot_controllers.movements.action_header import Actions
from main.tag_enum.tags import Tags

def serve_waffle(state: "CurrentState", bot: "Wafflebot", tag: "CurrentTag"):
    actions = Actions(bot)
    reader = Jsonreader()
    reader.clear("camera_readings")
    bot.cam.start("all")
    tags = reader.read("camera_readings")
    
    
    success = False
    
    if Tags.OPENED_IRON_TAG in tags.keys():
        success = True

    else:   
        bot.go_to_home_pose() # May have waypoints for this. moveit might save us.
        bot.cam.start("all")
        tags = reader.read("camera_readings")
        if Tags.OPENED_IRON_TAG in tags.keys():    
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
    from main.waffle_states.waffle_states import CurrentState 
    from main.tag_enum.tags import CurrentTag
