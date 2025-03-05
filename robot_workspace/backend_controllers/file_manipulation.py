import json
import numpy as numphy

# verify proper working directory
from os import getcwd, getenv
username = (getenv("USERNAME"))
if not getcwd()==f"/home/{username}/git/vaffelgutta":
    raise RuntimeError(
                       "run your script from the git/vaffelgutta/ directory instead.\n"
                       +"the current directory is\n"
                       +getcwd()
                       )

def _get_filepath(filename: str, name_is_full_file_path:bool)->str:
    if name_is_full_file_path:
        return filename
    else:
        return f"robot_workspace/assets/positions/{filename}.json"


def read_json(filename: str, name_is_full_file_path: bool = False)->dict:
    """TODO write documentation"""    
    filepath = _get_filepath(filename, name_is_full_file_path)
    try:
        with open(filepath, "r+") as file:
            if file.read() == "":
                file.seek(0)
                file.write("{}")
            file.seek(0)
            return json.load(file)
    except FileNotFoundError:
        print(f'file_manipulation: File \"{filename}\" not found.')
        return False
    
def write_json(data: dict, filename: str, name_is_full_file_path:bool = False) -> None:
    """TODO write documentation"""
    all_data = read_json(filename, name_is_full_file_path)
    all_data.update(data)
    filepath = _get_filepath(filename, name_is_full_file_path)
    with open(filepath, "w") as file:
        json.dump(all_data, file, indent=4)
    return

def pop_json(key: str, filename: str, name_is_full_file_path:bool = False)-> None:
    """todo write documentation"""
    data = read_json(filename, name_is_full_file_path)
    try:
        if not data: raise KeyError
        data.pop(key)
        filepath = _get_filepath(filename, name_is_full_file_path)
        with open(filepath,"w") as file:
            json.dump(data, file, indent=4)
    except KeyError:
        print(f"Key \"{key}\" not found in \"{filename}\". No action taken.")
    return


