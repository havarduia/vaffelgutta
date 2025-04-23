import json
from os import path as os_path
from robot.executable_scripts.common import directory_fixer
directory_fixer.fix_directiory()
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
        if not directory.endswith("/"):
            directory += "/"
        self.directory_path  = directory

    def read(self, filename: str)->dict:
        """reads the specified file contents as .json"""    
        if filename.endswith(".json"): filename=filename[:-5]
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
            return dict()
        
    def write(self, filename: str, data: dict) -> None:
        """
        ### Writes data to a .json file.
        
        :param file: The file to write to

        :param data: Dictionary containing the data to be stored. Can be nested dict.
        """
        if filename.endswith(".json"): filename=filename[:-5]
        all_data = self.read(filename)
        updated_data = {}
        for key, value in data.items():
            updated_data.update({str(key):value})
        try:
            all_data.update(updated_data)
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
        if filename.endswith(".json"): filename=filename[:-5]
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
        if filename.endswith(".json"): filename=filename[:-5]
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

def format_word(word: str, wordlength: int):
    return f"{f'{word}': ^{wordlength}}"

def print_line(line_items: list[str], wordlength: int):
    """helper function to print a single line of a table"""
    line = ""
    if isinstance(line_items, str):
        line_items = [line_items]
    for word in line_items:
        formatted_word = format_word(word, wordlength-1)+"|"
        line+=(formatted_word)
    print(f"|{line}")

def table_print(text_items: list[str], words_per_line: int = 3, skip_sort: bool = False):
    """function to print a list of items as a pretty princess table"""
    text_items = [str(item) for item in text_items]
    if not skip_sort:
        text_items.sort()
    line_length: int = 70
    word_length = int(line_length/words_per_line)
    line = []
    big_words = []
    print("-"*(line_length))
    for word in text_items:
        if len(word) >= word_length:
            big_words.append(word)
        else:
            line.append(word)
            if len(line) == words_per_line:
                print_line(line, word_length)
                line = []
    if len(line) != 0:
        for _ in range(words_per_line-len(line)):
            line.append("")
        print_line(line, word_length)
    for word in big_words:
        print("-"*line_length)
        print_line(word, word_length*words_per_line)
    print("-"*(line_length))

if __name__ == "__main__":
    pass 
