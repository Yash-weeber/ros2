#!/bin/bash

# 1. System-level ROS 2 Jazzy Packages
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ur \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2controlcli \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-ur-simulation-gz \
  python3-colcon-common-extensions \
  python3-rosdep

# 2. Python Dependencies
pip install numpy

# 3. Setup Workspace
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y

# 4. Build and Source
colcon build --allow-overriding ur_description --packages-select ur_description
source install/setup.bash

# 5. Gazebo Path Configuration
# This ensures Gazebo finds your mop and table meshes automatically
if ! grep -q "GZ_SIM_RESOURCE_PATH" ~/.bashrc; then
  echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/ur_description/share' >> ~/.bashrc
  echo "Added GZ_SIM_RESOURCE_PATH to ~/.bashrc"
fi

echo "Environment Setup Complete. Please restart your terminal."