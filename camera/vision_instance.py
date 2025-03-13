class InstanceRegistry:
    _instances = {}

    @classmethod
    def register(cls, key, instance):
        cls._instances[key] = instance

    @classmethod
    def get(cls, key):
        return cls._instances.get(key)