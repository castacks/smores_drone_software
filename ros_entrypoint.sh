#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/humble/setup.bash"
export PYTHONPATH="${PYTHONPATH}:/mast3r"
# Execute the command passed to the docker run
exec "$@"
