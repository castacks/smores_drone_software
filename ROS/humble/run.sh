#! /bin/bash

podman run -it \
	--volume ~/.Xauthority:/root/.Xauthority:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/smores/smores_drone_software:/workspace/smores_drone_software:z \
	-v /home/smores/triangluate/workspace:/workspace/triangulate:z \
	-v /mnt/storage:/storage/:z \
	--device=/dev/dri:/dev/dri \
	-e DISPLAY=$DISPLAY \
	--gpus all \
	--net host \
	--name smores_dev_all \
      	localhost/smores_dev:latest tail -f /dev/null
	# --name smores_dev
      	# localhost/smores_dev:latest /bin/bash

# -v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


