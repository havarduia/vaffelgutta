from robot.tools.file_manipulation import Jsonreader


reader = Jsonreader()

data = reader.read("recordings")

target = data["waffle_iron_open_<>"]

for key, value in data.items()
    if key.startswith("waffle_iron_open_"):
        value = target

reader.write("recordings", data)
