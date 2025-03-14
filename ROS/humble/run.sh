#! /bin/bash

docker run -it \
	--volume ~/.Xauthority:/root/.Xauthority:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /external/smores_drone_software:/workspace/smores_drone_software \
	--net host \
      	smores_dev:latest /bin/bash
	# -v /home/smores/smores_drone_software:/workspace/smores_drone_software:z \
	# -v /mnt/storage:/workspace/smores_drone_software/data \
	# --device=/dev/dri:/dev/dri \
	# -e DISPLAY=$DISPLAY \
	# --gpus all \
	# --net host \
	# --name smores_dev
      	# smores_dev:latest /bin/bash

# -v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


