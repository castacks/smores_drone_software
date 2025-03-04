podman run --gpus all -it -v /mnt/storage:/storage -v ~/thermal-preprocessing/workspace:/workspace --privileged \
        -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --net host --replace \
        --security-opt label=type:container_runtime_t \
        --name thermal-preprocessing  thermal-preprocessing /bin/bash