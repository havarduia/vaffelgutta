# Change the working directory to the base directory
from os import chdir
from os import path as ospath 
from sys import path as syspath

    

def fix_directiory() -> None:
    
    repo_name = "vaffelgutta/"
    path_to_repo = __file__
    
    while path_to_repo[-len(repo_name):] != repo_name:
        path_to_repo = path_to_repo[:-1]

    chdir(path_to_repo)
    syspath.append(ospath.abspath(path_to_repo))
    

if __name__ == "__main__":
    fix_directiory()