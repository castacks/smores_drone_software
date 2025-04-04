#! /bin/sh

# Build docker image
docker build -f Dockerfile \
             -t smores_dev /external/smores_drone_software

