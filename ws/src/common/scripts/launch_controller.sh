#!/bin/bash

echo "Launching Controller"

if [[ $(uname -i) = "aarch64" ]]
then
    mkdir /media/external-drive/$1
    echo "Git Hash:" >> /media/external-drive/$1/standup.txt
    git rev-parse HEAD >> /media/external-drive/$1/standup.txt
    ros2 launch controllers quad_stand_up.launch.py real:=$2 2>&1 | tee -a /media/external-drive/$1/standup.txt
    echo "Git Hash:" >> /media/external-drive/$1/mit_controller.txt
    git rev-parse HEAD >> /media/external-drive/$1/mit_controller.txt
    ros2 launch controllers mit_controller.launch.py real:=$2 2>&1 | tee -a /media/external-drive/$1/mit_controller.txt
    exit 0
else
    mkdir $1
    echo "Git Hash:" >> $1/standup.txt
    git rev-parse HEAD >> $1/standup.txt
    ros2 launch controllers quad_stand_up.launch.py real:=$2 2>&1 | tee -a $1/standup.txt
    echo "Git Hash:" >> $1/mit_controller.txt
    git rev-parse HEAD >> $1/mit_controller.txt
    ros2 launch controllers mit_controller.launch.py real:=$2 2>&1 | tee -a $1/mit_controller.txt
    exit 0
fi