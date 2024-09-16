#!/bin/bash

echo "This will remove all containers with \"dfki_quad\" label!"
echo "Please make sure to save all relevant data."
echo "After that, a new image will be build."
echo "Press ENTER, if you want to continue:"

read -s -n 1 key
if [[ $key != "" ]]; then 
	echo "Aborted."
	exit 0
fi

# get ids from all containers having the dfki_quad label
container_id=$(docker ps -aq --filter "label=dfki_quad")

# remove all containers with dfki_quad label
if [[ $container_id != "" ]]; then
	docker container rm ${container_id}
fi

# determine hardware architecture and then pass an appropriate build flag to docker
hw_arch=$(uname -i)

if [[ $hw_arch == "x86_64" ]]; then
  arch_build_arg=$hw_arch
  network_interface="enp0s31f6"
elif [[ $hw_arch == "aarch64" ]]; then
  arch_build_arg=$hw_arch
  network_interface="eth0"
else
  echo "Unknown hardware architecture, aborting..."
  exit 0
fi

# build image
docker build -t dfki_quad docker/ --build-arg="HW_ARCH=${arch_build_arg}" --build-arg="GO2_NETWORK_INTERFACE=${network_interface}"

exit 0




