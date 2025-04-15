from robot.tools.file_manipulation import Jsonreader
from robot.tools.update_tagoffsets import abs_position_from_offset

def interpret_target_command(
                            target: str | list[float] | list[list[float]],
                            return_joints: bool = False,
                            debug_print: bool = False
                            ) -> tuple[ list[list[float]] | list[float] | None , int]:
    """
    ### Converts a guess from a arbitrary input to either a set of joints or a matrix.

    :param target: The position to go to. Either of:
        \njoints
        \npose matrix
        \nposition name.

    :returns target: None (error), pose matrix or list of joints. 

    :returns datatype: -1 if error (None), 0 for list of joints, or 1 for matrix 
    """

    #handle string positions:
    if isinstance(target, str):
        reader = Jsonreader()
        positions = reader.read("recordings")
        #joints:
        if return_joints:
            try: 
                return (positions[target]["joints"], 0)
            except KeyError:
                if debug_print:
                    print(f"{target} not in recordings")
                return(None, -1)
        else:
        #matrices:
                tagid = int(positions[target]["tag"]) 
                if tagid == 100: 
                    return (positions[target]["basepose"], 1)
                else:
                    tags = reader.read("camera_readings")
                    tag_matrix = tags[tagid]
                    offset = positions[target]["offset"]
                    absolute_matrix = abs_position_from_offset(tag_matrix, offset)
                    return (absolute_matrix, 1)
        

    # now handle list positions:
    # convert numpy array, if applicable:
    try:
        target = target.tolist()
    except AttributeError:
        pass
    # test for list of joints
    if isinstance(target, list): 
        if (len(target) == 6):
            if return_joints:
                return (target, 0)
            else:
                return (None, -1)
        return (target, 1)
    
    raise RuntimeError("invalid datatype passed to move()")




