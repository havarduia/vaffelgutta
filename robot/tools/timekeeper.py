from time import time, sleep
from robot.tools.file_manipulation import *

def record_time(name: str) -> None:
    reader = Jsonreader()
    reader.update_filedirectory("robot/assets/misc/")
    data = {name:time()}
    if name.lower() == "start":
        reader.clear("time_recordings")
    reader.write("time_recordings",data)
    return

def read_times(): 
    reader = Jsonreader()
    reader.update_filedirectory("robot/assets/misc/")
    data = reader.read("time_recordings")
    list_of_items = []
    prev_time = data["start"]
    for k,v in data.items():
        list_of_items.append(k)
        deltaT =round( v - prev_time, 3)
        list_of_items.append(deltaT)
        prev_time = v

    table_print(list_of_items, 2,True)

if __name__ == "__main__":
    read_times()
