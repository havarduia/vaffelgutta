#!/bin/bash
current_dir=$(pwd)

# set python path
if echo "$USER" | grep -q "vaffelgutta" 
    then
    validinput=0
    while ! [[ $validinput == 1 ]]
    do
        sudo echo "Are you running this script for the first time? y/n"
        read myinput
        if [[ $myinput == "y" ]] || [[ $myinput == "n" ]]
        then
            validinput=1 
            if [ $myinput == "y" ]
            then
                append_path=true
            else
                append_path=false
            fi
        else
            echo "invalid input." 
        fi
    done
elif echo "$PYTHONPATH" | grep -q "vaffelgutta" 
    then
    append_path=false
else
    append_path=true
fi
if [[ $append_path == "true" ]]
then
    echo "export PYTHONPATH=$PYTHONPATH:$current_dir" >> ~/.bashrc 
fi
# install interbotix library
cd 
# neccesary files for moveit and gazebo respectively
echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:' >> ~/.bashrc
sudo apt install curl -y
sudo apt update && sudo apt install -y dbus-x11
architecture=$(dpkg --print-architecture)
TERMINAL=$(command -v gnome-terminal || command -v konsole || command -v x-terminal-emulator || command -v xterm || command -v lxterminal || echo "")

if [[ $architecture == "arm64" ]]
then
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh' > xsarm_rpi4_install.sh
    chmod +x xsarm_rpi4_install.sh 
else
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
    chmod +x xsarm_amd64_install.sh
fi

# install
sudo apt-get update -y
sudo apt-get upgrade -y
# install python requirements
cd $current_dir 

sudo apt install python3 -y
pip install -r requirements.txt



# realsense
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
sudo apt-get install git wget cmake build-essential -y
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y
cd
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.55.1
mkdir build
cd build
cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=$(which python3) 
make -j$(nproc)
sudo make install -j$(nproc)
