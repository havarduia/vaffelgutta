from enum import Enum


class Tags(Enum):
    CLOSED_IRON_TAG = "1"
    OPENED_IRON_TAG = "2"
    SPRAY_TAG = "3"
    LADLE_TAG = "4"
    STICKS_TAG = "5"
    JAM_TAG = "6"
    FOLLOW_ME_TAG = "7"
    ORIGIN_TAG = "0"
    
    # ...fill in tags here

class CurrentTag:
    def __init__(self, starttag):
        
        self.tag = starttag
        self.tagtarget = None
    
    def set(self, new_tag: Tags) -> None:
        self.tag = new_tag

    def get(self) -> Tags:
        return self.tag