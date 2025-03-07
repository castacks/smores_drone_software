#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/humble/setup.bash"
export PYTHONPATH="${PYTHONPATH}:/workspace/smores_drone_software/include"
# Execute the command passed to the docker run
exec "$@"
