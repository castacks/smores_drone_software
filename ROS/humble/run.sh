#! /bin/bash
container_name="smores_macvo"

podman container rm $container_name -f

podman run -it \
	--privileged \
	--volume ~/.Xauthority:/root/.Xauthority:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/smores/pc_compare:/workspace/pc_compare:z \
	-v /home/smores/MAC-VO-ROS2:/workspace/macvo \
	-v /mnt/storage:/storage/:z \
	--device=/dev/dri:/dev/dri \
	-e DISPLAY=$DISPLAY \
	--gpus all \
	--net host \
	--name $container_name \
      	smores_dev tail -f /dev/null
	# --name smores_dev
      	# localhost/smores_dev:latest /bin/bash

# -v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


