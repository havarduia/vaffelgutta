import subprocess
import os
import signal
import sys

class ArucoPoseLauncher:
    def __init__(self, script_path):
        self.script_path = script_path
        self.process = None

    def launch(self):
        if self.process is None:
            self.process = subprocess.Popen(["/bin/bash", self.script_path], preexec_fn=os.setsid)
            print("Aruco pose estimation launched.")
        else:
            print("Process is already running.")

    def kill(self, signum=None, frame=None):
        if self.process is not None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process = None
            print("Aruco pose estimation stopped.")
        else:
            print("No process is running.")

if __name__ == "__main__":
    launcher = ArucoPoseLauncher("/home/havard/git/vaffelgutta/camera/run_camera.sh")
    
    # Handle Ctrl + C
    def signal_handler(signum, frame):
        launcher.kill()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    launcher.launch()
    print("Press Ctrl + C to stop...")
    signal.pause()
