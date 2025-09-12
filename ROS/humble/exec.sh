#! /bin/bash
container_name="smores_dev_container"

docker exec -it \
	-e DISPLAY=$DISPLAY \
	--user "$(id -u):$(id -g)" \
      	smores_dev_container /bin/bash

# VOLUME MOUNT SYNTAX 
# 	-v <host pc absolute path>:<absolute container path>

# LIBGL_ALWAYS_SOFTWARE=1


