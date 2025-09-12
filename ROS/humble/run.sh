#! /bin/bash
container_name="smores_dev_image"

docker rm -f $container_name

docker run -it \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ~/.Xauthority:/home/drone/.Xauthority:ro \
	-v ~/.Xauthority:/root/.Xauthority:ro \
	\
	-v ~/.ssh:/home/drone/.ssh:ro \
	-v "$SSH_AUTH_SOCK:/tmp/ssh-agent.sock" \
	-e SSH_AUTH_SOCK="/tmp/ssh-agent.sock" \
	\
	-v ~/MRSD/smores/smores_drone_software:/home/drone/smores_drone_software:rw \
	-v ~/MRSD/smores/smores_drone_drivers:/home/drone/smores_drone_drivers:rw \
	\
	--gpus all \
	--net host \
	--privileged \
	--device=/dev/dri:/dev/dri \
	--user "$(id -u):$(id -g)" \
	--name smores_dev_container \
      	$container_name tail -f /dev/null

# VOLUME MOUNT SYNTAX 
# 	-v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


