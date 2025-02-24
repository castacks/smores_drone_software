#! /bin/bash

docker build -t ros_noetic .

docker run -it \
	--volume ~/.Xauthority:/home/user/.Xauthority:rw \
	--volume ~/.Xauthority:/root/.Xauthority:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--device=/dev/dri:/dev/dri \
	-e DISPLAY=$DISPLAY \
	--gpus all \
      	ros_noetic /bin/bash

# -v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


