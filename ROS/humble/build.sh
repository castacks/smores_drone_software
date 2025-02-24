#! /bin/sh

# Build docker image
podman build -f Dockerfile_ROS1 -t ros_noetic .
