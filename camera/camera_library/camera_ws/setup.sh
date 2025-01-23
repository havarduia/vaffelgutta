#!/bin/bash

# Update git submodules recursively
git submodule update --init --recursive

# Build the workspace with colcon
colcon build --symlink-install

# Path to the setup file
setup_file="~/git/vaffelgutta/camera/camera_library/camera_ws/install/setup.bash"

# Add the source command to .bashrc if not already there
if ! grep -q "$setup_file" ~/.bashrc; then
    echo "source $setup_file" >> ~/.bashrc
    echo "Added source command to .bashrc"
else
    echo "Source command already present in .bashrc"
fi

echo "Workspace setup completed!"

