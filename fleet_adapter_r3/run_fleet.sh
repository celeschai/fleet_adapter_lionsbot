#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "=== 1. Cleaning up old build artifacts ==="
rm -rf build/ install/ log/ .pytest_cache/ __pycache__/ *.egg-info/ src/*.egg-info

echo "=== 2. Sourcing ROS 2 environment ==="
echo "
Reminder: Please set the LIONSBOT_USER, LIONSBOT_PASSWORD, ROBOT_ID environment variables. 
export LIONSBOT_USER="your_email@lionsbot.com" 
export LIONSBOT_PASSWORD="your_password"
export LIONSBOT_API="hostname"
export ROBOT_ID="robotid"
"

# Adjust the path below if your ROS 2 installation is under an underlay like /opt/ros/humble/setup.bash
# Assuming 'ros2' was a custom alias/script, using standard sourcing template here:
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    # Fallback to your custom local 'ros2' trigger if it's an alias or binary in path
    source ros2 2>/dev/null || echo "Using system default ROS 2 environment..."
fi

echo "=== 3. Updating rosdep and installing dependencies ==="
# rosdep update 
rosdep install --from-paths . --ignore-src -r -y

echo "=== 4. Building the workspace ==="
colcon build --symlink-install

echo "=== 5. Sourcing the local workspace ==="
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Build might have failed."
    exit 1
fi

echo "=== 6. Launching fleet_adapter_r3 ==="
ros2 run fleet_adapter_r3 fleet_adapter \
    -c configs/config.yaml \
    -n maps/0.yaml \
    -d maps/dock_summary.yaml