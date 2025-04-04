#! /bin/sh

# Build docker image
podman build -f Dockerfile \
             -t smores_dev /home/smores/smores_drone_software \
             --build-arg ssh_prv_key="$(cat ~/.ssh/ranais_ssh)"
