#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/humble/setup.bash"
# export PYTHONPATH="${PYTHONPATH}:/workspace/smores_drone_software/include"
# echo "Installing MoGE dependencies..."
# pip install -r "/workspace/smores_drone_software/include/MoGe/requirements.txt"
# echo "Installing Madpose dependencies..."
# pip install "/workspace/smores_drone_software/include/madpose/"
# Execute the command passed to the docker run
echo $PATH
exec "$@"
