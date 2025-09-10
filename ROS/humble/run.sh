#! /bin/bash
container_name="smores_dev_image"

docker container rm $container_name -f

docker run -it \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--gpus all \
	--net host \
	--privileged \
	--device=/dev/dri:/dev/dri \
	--user "$(id -u):$(id -g)" \
	--volume ~/.Xauthority:/root/.Xauthority:rw \
	--name $container_name \
      	smores_dev_container tail -f /dev/null

# VOLUME MOUNT SYNTAX 
# 	-v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


