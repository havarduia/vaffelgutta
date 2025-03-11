# verify proper working directory
from os import getcwd, getenv
from os import path as os_path
username = (getenv("USERNAME"))
assert getcwd()==f"/home/{username}/git/vaffelgutta", (
                    "run your script from the git/vaffelgutta/ directory instead.\n"
                    +"the current directory is\n"
                    +getcwd())
    
import json

class Jsonreader:
    def __init__(self, 
            directory_path: str = "robot/assets/position_data/"
        ):
        self.directory_path = directory_path
        return

    def update_filedirectory(self, directory: str):
        """### sets the file directory for reading.
        
        :param directory: full path of the direcory to change to. Starting at vaffelgutta/.
        """
        self.directory_path  = directory

    def read(self, filename: str)->dict | None:
        """reads the specified file contents as .json"""    
        try:
            with open((self.directory_path+filename+".json"), "r+") as file:
                if file.read() == "":
                    file.seek(0)
                    file.write("{}")
                file.seek(0)
                return json.load(file)
        except FileNotFoundError:
            print(f"""file manipulation:
                 \rfile \"{filename}.json\" not found in \"{self.directory_path}\". 
                \rNo action taken.""")
            return None
        
    def write(self, filename: str, data: dict) -> None:
        """
        ### Writes data to a .json file.
        
        :param file: The file to write to

        :param data: Dictionary containing the data to be stored. Can be nested dict.
        """
        all_data = self.read(filename)
        try:
            all_data.update(data)
        except AttributeError:
            return None
        with open((self.directory_path+filename+".json"), "w") as file:
            try:
                json.dump(all_data, file, indent=4)
            except FileNotFoundError:
                print(f"""file manipulation:
                  \rfile \"{filename}.json\" not found in \"{self.directory_path}\". 
                  \rNo action taken.""")
        return None
    def clear(self, filename: str) -> None:
        filepath = self.directory_path + filename + ".json"
        if os_path.exists(filepath):
            with open(filepath, "w") as file:
                file.write("{}")
        else:
            print(f"file {filename} not found")
        return None
        
    def pop(self, filename: str, key: str)-> bool:
        """
        ### Pops a key from the given file
        
        :param file: the name of the file to change
        
        :param key: the key to pop
        """
        data = self.read(filename)
        try:
            if not data: raise KeyError
            data.pop(key)
            with open((self.directory_path+filename+".json"),"w") as file:
                json.dump(data, file, indent=4)
            return True
        except KeyError:
            print(f"""file manipulation:
                  \rKey \"{key}\" not found in \"{self.directory_path}/{filename}.json\". 
                  \rNo action taken.""")
            return False
        
if __name__ == "__main__":
    reader = Jsonreader()
    