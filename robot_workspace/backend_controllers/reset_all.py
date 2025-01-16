"""
This script is intended to clear all instances of ros, rviz, etc.. being run
This should never be used while running a program, 
but rather as a cleanup when another script has
gone out of control
"""
import psutil
import os
from signal import SIGINT as killsig

def return_this_script_pid():
    return 

def get_pids_by_name(process_name):
    #Returns a list of PIDs for processes matching the given name.
    return [proc.pid for proc in psutil.process_iter(attrs=['pid', 'name']) if proc.info['name'] == process_name]


def kill_processes_by_name(process_name, killsig):
    #Sends the given signal (default: SIGINT) to all processes matching the given name.
    pids = get_pids_by_name(process_name)
    for pid in pids:
        if pid!=os.getpid():
            try:
                # try kill
                psutil.Process(pid).send_signal(killsig)
                print(f"Killed {process_name}:{pid} successfully")
            except psutil.NoSuchProcess:
                print(f"Process {pid} no longer exists.")
            except psutil.AccessDenied:
                print(f"Permission denied to kill process {pid}. Try running as sudo.")

def main():
    processes =[
        "rviz2",
        "gzclient",
        "gzserver",
        "gazebo",
        "python3",
        "robot_state_publisher",
        "ros2",
    ]
    for proc in processes:
        kill_processes_by_name(proc, killsig)




if __name__=="__main__":
    main()