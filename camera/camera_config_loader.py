import yaml

class ConfigLoader:
    def __init__(self, config_path="config.yaml"):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
    
    def get(self, key, default=None):
        return self.config.get(key, default)