from typing import Literal
import numpy as numphy
from robot.tools.file_manipulation import Jsonreader
def interpret_target_command(
                            target: str | list[list[float]] |list[float],
                            file: str = "None",
                            debug_print: bool = False,
                            ) -> tuple[list[float] | int]:
    """
    ### Converts a guess from a arbitrary input to either a set of joints or a matrix.

    :param target: The position to go to. Either of:
        \njoints
        \npose matrix
        \nposition name.

    if a position name is given, the "file" parameter must also be given.

    :param file: The name of the file to read the pose from. 
    must be in the position_data folder.

    :returns target: None (error), pose matrix or list of joints. 

    :returns datatype: 0 if error (None), 1 for pose matrix or 2 for list of joints. 
    """
    # test for list of joints
    if (isinstance(target, list)): 
        if (len(target) == 6):
            return (target, 2)
    # test for numpy matrix
    elif isinstance(target, numphy.matrix):
        target = target.tolist()
    elif isinstance(target, str):
        if file is None:
            if debug_print:
                print("Wafflebot: tried to interpret name without file input")
            return (None, 0)
        positions = Jsonreader().read(file)
        try: 
            return (positions[target]["matrix"], 1)
        except KeyError:
            if debug_print:
                print(f"{target} not in {file}")
            return(None, 0)
    else:
        if debug_print:
            print("Wafflebot: Unsupported command type.")
        return (None, 0)
    
    return (target, 1)