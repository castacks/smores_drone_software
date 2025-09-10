#! /bin/sh

# Build docker image
# Syntax docker -f <name to Dockerfile> -t <What name to assign> <root dir for docker build (build context)>
docker build -f ROS/humble/Dockerfile \
             -t smores_dev_container \
             --build-arg HOST_UID=$(id -u) \
             --build-arg HOST_GID=$(id -g) \
             $(git rev-parse --show-toplevel)
