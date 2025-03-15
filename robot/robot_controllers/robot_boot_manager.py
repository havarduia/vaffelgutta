from os import killpg, getpgid, setpgrp
from subprocess import Popen 
from signal import SIGTERM
from time import sleep

def robot_launch(use_real_robot): 
  #run shell script
   
  hardwaretype = "actual" if use_real_robot else "fake"
  use_sim = "true" if (not use_real_robot) else "false"
  
  command = ('ros2 launch interbotix_xsarm_control'
  +' xsarm_control.launch.py'
  +' robot_model:=vx300s'
  +' hardware_type:=' + hardwaretype
  +' use_sim:=' + use_sim
  )
  
  process = Popen(["bash", "-c", command],
                   preexec_fn=setpgrp
                   )
  global boot_manager_pid; boot_manager_pid = process.pid
  sleep(0.5)  
  print("Boot Manager: Process started with pid: " + str(process.pid))
  return()

def robot_close():
    try:
      killpg(getpgid(boot_manager_pid), SIGTERM)
      print("Boot manager: Process killed") 
    except NameError:
       print("Boot manager: PID not found")
