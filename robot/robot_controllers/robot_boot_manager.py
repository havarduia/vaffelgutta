import subprocess
from os import killpg, getpgid, setpgrp
from signal import SIGTERM
from time import sleep

def robot_launch(use_rviz=True):
    hardwaretype = "actual" if True else "fake"
    use_sim = "true" if not True else "false"
    use_rviz = "true" if use_rviz else "false"

    command = ('ros2 launch'
               ' interbotix_xsarm_moveit'
               ' xsarm_moveit.launch.py'
               ' robot_model:=vx300s'
               f' hardware_type:={hardwaretype}'
               # f' use_sim:={use_sim}'
               f' use_moveit_rviz:={use_rviz}'
               )

    process = subprocess.Popen(["bash", "-c", command],
                               stderr=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL,
                               preexec_fn=setpgrp
                               )
    process2 = subprocess.Popen(["bash", "-c", "ros2 run collision_publisher collision_publisher"],
                                stderr=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL,
                                preexec_fn=setpgrp
                                )
    global boot_manager_pid
    boot_manager_pid = process.pid
    global boot_manager_pid2
    boot_manager_pid2 = process2.pid
    sleep(0.5)
    print("Boot Manager: Process started with pid: " + str(process.pid))
    return process

def robot_close():
    try:
        killpg(getpgid(boot_manager_pid), SIGTERM)
        killpg(getpgid(boot_manager_pid2), SIGTERM)
        print("Boot manager: Process killed")
    except NameError:
        print("Boot manager: PID not found")
