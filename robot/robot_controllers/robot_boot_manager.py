from os import killpg, getpgid, setpgrp, wait
from subprocess import Popen, DEVNULL
from signal import SIGTERM
from time import sleep


def robot_launch(use_rviz = True, use_moveit = True): 
    use_rviz = "true" if use_rviz else "false"
    controltype = 'moveit' if use_moveit else 'control'

    command1 = ('ros2 launch'
               +' interbotix_xsarm_' + controltype
               +f' xsarm_{controltype}.launch.py'
               +' robot_model:=vx300s'
               +' hardware_type:=actual'
               +f' use_{"moveit" if use_moveit else ""}_rviz:='+use_rviz
               )

    command2 = "ros2 run collision_publisher collision_publisher"

    process1 = Popen(["bash", "-c", command1],
                    stderr=DEVNULL,
                    stdout=DEVNULL,
                    preexec_fn=setpgrp
                    )
    global boot_manager_pid; boot_manager_pid = process1.pid

    if use_moveit:
        process2 = Popen(["bash", "-c", command2],
                         stderr=DEVNULL,
                         stdout=DEVNULL,
                         preexec_fn=setpgrp
                         )
        global boot_manager_pid2; boot_manager_pid2 = process2.pid
    sleep(0.5)  
    print("Boot Manager: Process started with pid: " + str(process1.pid))
    return process1

def robot_close():
    try:
        killpg(getpgid(boot_manager_pid), SIGTERM)
        killpg(getpgid(boot_manager_pid2), SIGTERM)
        print("Boot manager: Process killed") 
    except NameError:
        print("Boot manager: PID not found")

