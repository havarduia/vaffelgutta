import yaml
import os
class ConfigLoader:
    def __init__(self, config_path=os.path.expanduser('~/git/vaffelgutta/camera/config/config.yaml')):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
    
    def get(self, key, default=None):
        return self.config.get(key, default)
    
def print_blue(message):
    BLUE = "\033[94m"   # ANSI code for bright blue text
    RESET = "\033[0m"   # ANSI code to reset the text color to default
    print(f"{BLUE}{message}{RESET}")

def print_error(message):
    RED = "\033[91m"   # ANSI code for bright red text
    RESET = "\033[0m"  # ANSI code to reset text color to default
    print(f"{RED}{message}{RESET}")
    


