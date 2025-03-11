def print_blue(message):
    BLUE = "\033[94m"   # ANSI code for bright blue text
    RESET = "\033[0m"   # ANSI code to reset the text color to default
    print(f"{BLUE}{message}{RESET}")

def print_error(message):
    RED = "\033[91m"   # ANSI code for bright red text
    RESET = "\033[0m"  # ANSI code to reset text color to default
    print(f"{RED}{message}{RESET}")
    
    
def save_matrices_to_json(tags, filepath):
        data = {}
        for tag_id, matrix in tags.items():
            matrix_list = matrix.tolist()  # Convert numpy array to list
            # IDK WHAT TO DO WITH JOINTS
            data[str(tag_id)] = {
                "matrix": matrix_list,
                "joints": []
            }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4)