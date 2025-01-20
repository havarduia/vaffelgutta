import os
import subprocess

WORKSPACE = "test_Projects"
VENV_PATH = os.path.join(WORKSPACE, "{WORKSPACE}_env")
PYTHON_EXEC = os.path.join(VENV_PATH, "bin", "python")

def run_command(command):
    """Helper function to run shell commands."""
    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        print(f"Error running command: {command}")
        exit(1)

# Step 1: Check if virtual environment exists, create it if missing
if not os.path.exists(VENV_PATH):
    print("Virtual environment not found.")
    
# Step 2: Install dependencies
run_command(f"{PYTHON_EXEC} -m pip install --upgrade pip")
run_command(f"{PYTHON_EXEC} -m pip install -r {WORKSPACE}/requirements.txt")

# Step 3: Run the script
run_command(f"{PYTHON_EXEC} {WORKSPACE}/run_copy_camera.py")
