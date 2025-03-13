def print_blue(message):
    BLUE = "\033[94m"   # ANSI code for bright blue text
    RESET = "\033[0m"   # ANSI code to reset the text color to default
    print(f"{BLUE}{message}{RESET}")

def print_error(message):
    RED = "\033[91m"   # ANSI code for bright red text
    RESET = "\033[0m"  # ANSI code to reset text color to default
    print(f"{RED}{message}{RESET}")
    
