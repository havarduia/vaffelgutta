
# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath
chdir(ospath.expanduser("~/git/vaffelgutta"))
syspath.append(ospath.abspath(ospath.expanduser("~/git/vaffelgutta")))
from robot_workspace.backend_controllers.file_manipulation import Jsonreader
import inspect

from robot_workspace.assets.position_data import tools

data_var = inspect.getmembers(tools)
keys = []
values =  []
for key, value in data_var:
    if not key.startswith("__"):
        keys.append(key)
        values.append(value)

data = dict()
for key, value in zip(keys, values):
    data[str(key)] = value
reader = Jsonreader()
reader.write("static_objects",data)