import yaml
import os
class ConfigLoader:
    def __init__(self, config_path=os.path.expanduser('~/git/vaffelgutta/camera/config.yaml')):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
    
    def get(self, key, default=None):
        return self.config.get(key, default)
