#! /bin/bash

docker build -t ros_noetic -f ~/MRSD/smores/SLAM/ROSNoetic_Focal/Dockerfile ~/MRSD/smores

docker run -it \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/ranai/Downloads:/tmp/downloads \
	-v /home/ranai/MRSD/smores/SLAM:/tmp/slam \
	--device=/dev/dri:/dev/dri \
	-e DISPLAY=$DISPLAY \
	--gpus all \
      	ros_noetic /bin/bash

# -v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


