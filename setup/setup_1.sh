#!/bin/bash
current_dir=$(pwd)

read -p "Do you only want to add PYTHON PATHS and not install? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    grep -qxF "export PYTHONPATH=\$PYTHONPATH:$current_dir" ~/.bashrc || echo "export PYTHONPATH=\$PYTHONPATH:$current_dir" >> ~/.bashrc
    exec bash
    exit 0
fi

# Set python path logic
if [[ "$USER" == "vaffelgutta" ]]; then
    validinput=0
    while [[ $validinput -eq 0 ]]; do
        echo "Are you running this script for the first time? (y/n)"
        read myinput
        if [[ "$myinput" == "y" || "$myinput" == "n" ]]; then
            validinput=1
            append_path=$([[ "$myinput" == "y" ]] && echo "true" || echo "false")
        else
            echo "Invalid input."
        fi
    done
elif [[ "$PYTHONPATH" =~ "vaffelgutta" ]]; then
    append_path="false"
else
    append_path="true"
fi

if [[ "$append_path" == "true" ]]; then
    grep -qxF "export PYTHONPATH=\$PYTHONPATH:$current_dir" ~/.bashrc || echo "export PYTHONPATH=\$PYTHONPATH:$current_dir" >> ~/.bashrc
fi

# Install interbotix library
echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:' >> ~/.bashrc

sudo apt update && sudo apt install -y curl dbus-x11 python3

architecture=$(dpkg --print-architecture)
if [[ "$architecture" == "arm64" ]]; then
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh' -o xsarm_rpi4_install.sh
    chmod +x xsarm_rpi4_install.sh
    sudo mv xsarm_rpi4_install.sh setup_2.sh
else
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' -o xsarm_amd64_install.sh
    chmod +x xsarm_amd64_install.sh
    sudo mv xsarm_amd64_install.sh setup_2.sh
fi

# Install required dependencies
sudo apt-get upgrade -y
sudo -H pip install -r requirements.txt

# Install RealSense dependencies
sudo apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev git wget cmake build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

cd
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.55.1
mkdir -p build
cd build
cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=$(which python3)
make -j$(nproc)
sudo make install -j$(nproc)
echo 'export PYTHONPATH=$PYTHONPATH:~/librealsense/build/release' >> ~/.bashrc
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
