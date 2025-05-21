#Todo
from typing import Optional
from robot.robot_controllers.Wafflebot.read_collisionobjects import read_collisionobjects
from robot.tools.file_manipulation import Jsonreader

def add_collisionobjects(ignore: Optional[list[str]] = None)-> tuple[dict, dict]:
    
    dynamic_boxes = read_collisionobjects()  

    reader = Jsonreader()
    reader.update_filedirectory("robot/assets/boundingboxes/")
    static_boxes = reader.read("static")

    all_boxes = dynamic_boxes
    all_boxes.update(static_boxes)

    add_objects = dict()
    rm_objects = dict()
    if ignore is None: ignore = list()
    
    for key, val in dynamic_boxes.items():
        if key in ignore:
            rm_objects.update({key : val})
        else:
            add_objects.update({key : val})
    
    reader.update_filedirectory("robot/assets/boundingboxes/publish")
    reader.clear("add")
    reader.write("add", add_objects)
    reader.clear("remove")
    reader.write("remove", rm_objects)
    return (add_objects, rm_objects)

if __name__ == "__main__":
    add_collisionobjects("test")
