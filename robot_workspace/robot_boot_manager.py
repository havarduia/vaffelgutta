from os import killpg, getpgid, setpgrp
from subprocess import Popen 
from signal import SIGTERM
from time import sleep

def robot_launch(): 
  #run shell script
  process = Popen(["bash", "-c", 
      + 'ros2 launch interbotix_xsarm_control'
      +' xsarm_control.launch.py'
      +' robot_model:=vx300s'
      +' hardware_type:=fake'
      +' use_sim:=true'
        	], preexec_fn=setpgrp
        )
  sleep(1)  
  print("Boot Manager: Process started with pid: " + str(process.pid))
  return(process.pid)

def robot_close(pid):
    killpg(getpgid(pid), SIGTERM)
    print("Boot manager: Process killed") 
