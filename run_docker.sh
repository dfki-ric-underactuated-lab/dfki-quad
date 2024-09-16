#!/bin/bash

touch -a /home/$USER/.ros_docker_bash_history

# check, if a container with dfki_quad label is already there
container_id=$(docker ps -aq --filter "label=dfki_quad")

if [[ $container_id = "" ]]; then
    # start new container if there is none
    xhost +local:
    docker_options=(-it \
        --label="dfki_quad" \
        --net="host" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${PWD}/ws/src:/root/ros2_ws/src" \
        --volume="/home/$USER/.ros_docker_bash_history:/root/.bash_history" \
	--device="/dev/input:/dev/input" \
	--device="/dev/ttyACM0:/dev/ttyACM0" \
	--privileged)

    # when on arm, try to mount the external ssd for data logging
    if [[ $(uname -i) = "aarch64" ]]; then
	docker_options+=(--mount type=bind,src=/media/external-drive,dst=/media/external-drive)
	docker_options+=(--mount type=bind,src=/usr/bin/tegrastats,dst=/usr/bin/tegrastats)
    elif [[ $(uname -i ) = "x86_64" ]]; then
	docker_options+=(--mount type=bind,source=/sys/class/powercap,target=/sys/class/powercap)
    fi

    docker run "${docker_options[@]}" dfki_quad:latest
else
    # restart existing container
    docker start -i $(docker ps -n1 -aq --filter "label=dfki_quad") 
fi

exit 0


#export containerId=$(docker ps -l -q)

