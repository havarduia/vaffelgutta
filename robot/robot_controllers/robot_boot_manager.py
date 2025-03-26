from os import killpg, getpgid, setpgrp
from subprocess import Popen 
from signal import SIGTERM
from time import sleep

def robot_launch(use_moveit_robot, use_rviz = True): 
  hardwaretype = "actual" if True else "fake"
  use_sim = "true" if (not True) else "false"
  use_rviz = "true" if use_rviz else "false"
  
  command = ('ros2 launch'
  +' interbotix_xsarm_moveit'
  +f' xsarm_moveit.launch.py'
  +' robot_model:=vx300s'
  +' hardware_type:=' + hardwaretype
  #+' use_sim:=' + use_sim
  +f' use_moveit_rviz:='+use_rviz
  )
  
  process = Popen(["bash", "-c", command],
                   preexec_fn=setpgrp
                   )
  global boot_manager_pid; boot_manager_pid = process.pid
  sleep(0.5)  
  print("Boot Manager: Process started with pid: " + str(process.pid))
  return process  

def robot_close():
    try:
      killpg(getpgid(boot_manager_pid), SIGTERM)
      print("Boot manager: Process killed") 
    except NameError:
       print("Boot manager: PID not found")
