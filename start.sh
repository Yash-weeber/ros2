#!/bin/bash

# 1. Ensure we are in the workspace directory (Change this if your path is different)
cd ~/ros2_ws || { echo "Workspace not found! Make sure you are in ~/ros2_ws"; exit 1; }


# We build once here to prevent 'Lock File' errors
colcon build --packages-select ur_description --allow-overriding ur_description

if [ $? -ne 0 ]; then
    echo "[ERROR] Build failed. Stopping."
    exit 1
fi


# T1: SIMULATION (cleaning_session)
# We open a new tab/window, source, launch, and keep open on exit ('exec bash')
gnome-terminal --title="T1: Simulation" -- bash -c "source install/setup.bash; \
export ROS_DOMAIN_ID=42
ros2 launch ur_description cleaning_session.launch.py; \
exec bash"



sleep 8

# T2: MOVEIT
gnome-terminal --title="T2: MoveIt" -- bash -c "source install/setup.bash; \
export ROS_DOMAIN_ID=42
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e; \
exec bash"



sleep 10

# T3: PYTHON SCRIPT
gnome-terminal --title="T3: Python Script" -- bash -c "source install/setup.bash; \
export ROS_DOMAIN_ID=42
python3 finnal.py; \
exec bash"

