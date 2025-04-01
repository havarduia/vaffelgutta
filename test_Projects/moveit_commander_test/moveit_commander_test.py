from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import Pose
from time import sleep
def main():
    scene = PlanningSceneInterface("move_group_interface")
    pose = Pose()
    pose.position.x = 0.5
    pose.position.x = 0.5
    pose.position.x = 0.5
    pose.orientation.w = 1 
    scene.add_box("testbox", pose, size = (0.4, 0.4, 0.4) )
    
    scene.apply_planning_scene()
    sleep(20)
    scene.clear()

if __name__ == "__main__":
    main()