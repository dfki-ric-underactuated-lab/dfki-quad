#!/bin/bash

echo "Launching Hardware"

if [[ $(uname -i) = "aarch64" ]]
then
    mkdir /media/external-drive/$1
    echo "Git Hash:" >> /media/external-drive/$1/hardware_launch.txt
    git rev-parse HEAD >> /media/external-drive/$1/hardware_launch.txt
    ros2 launch common hardware.launch.py real:=$2 onboard:=true 2>&1 | tee -a /media/external-drive/$1/hardware_launch.txt
    exit 0
else
    mkdir $1
    echo "Git Hash:" >> $1/hardware_launch.txt
    git rev-parse HEAD >> $1/hardware_launch.txt
    ros2 launch common hardware.launch.py real:=$2 2>&1 | tee -a $1/hardware_launch.txt
    exit 0
fi

