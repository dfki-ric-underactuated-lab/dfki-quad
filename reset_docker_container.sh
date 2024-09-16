#!/bin/bash

echo "This will remove all containers with the \"dfki_quad\" label!"
echo "Make sure to backup all relevant data before."
echo "To continue, please press ENTER:"

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

