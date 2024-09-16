#!/bin/sh

# perform a binary install on x86 machines
# on ARM, drake has to be built from source

if [ "${HW_ARCH}" = "x86_64" ]; then
  # Drake Setup (similar as they do in their docker containers)
  cd /opt
  wget -nc -O drake-latest-jammy.tar.gz https://github.com/RobotLocomotion/drake/releases/download/v1.24.0/drake-1.24.0-jammy.tar.gz
  tar -xf drake-latest-jammy.tar.gz
  apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy $(cat /opt/drake/share/drake/setup/packages-jammy.txt)
  echo 'export PATH="/opt/drake/bin:${PATH}"' >> ~/.bashrc
  echo 'export PYTHONPATH="/opt/drake/lib/python3.10/site-packages:${PYTHONPATH}"' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH="/opt/drake/lib:${LD_LIBRARY_PATH}"' >> ~/.bashrc
  # the following is needed for drake to find meshes in ROS style urdfs (i.e. when using package://path/to/meshes/)
  echo 'export ROS_PACKAGE_PATH="/root/ros2_ws/src"' >> ~/.bashrc
  
elif [ "${HW_ARCH}" = "aarch64" ]; then
  echo 'Skipping drake on ARM'
  # build and install Drake from source: https://drake.mit.edu/from_source.html
  #  cd /root
  # git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git
  # sed -i "/with_asking=1/c\with_asking=0" /root/drake/setup/ubuntu/binary_distribution/install_prereqs.sh
  # sed -i "/with_asking=1/c\with_asking=0" /root/drake/setup/ubuntu/source_distribution/install_prereqs.sh
  # /bin/bash -c "/root/drake/setup/ubuntu/install_prereqs.sh"
  # mkdir drake-build
  # cd /root/drake-build
  # cmake ../drake
  # make install
  # echo 'export PYTHONPATH="${PWD}/install/lib/python3.10/site-packages:${PYTHONPATH}"' >> ~/.bashrc
  # echo 'export PATH="/root/drake-build/install/bin:${PATH}"' >> ~/.bashrc
  # echo 'export LD_LIBRARY_PATH="/root/drake-build/install/lib:${LD_LIBRARY_PATH}"' >> ~/.bashrc
  # the following is needed for drake to find meshes in ROS style urdfs (i.e. when using package://path/to/meshes/)
  # echo 'export ROS_PACKAGE_PATH="/root/ros2_ws/src"' >> ~/.bashrc
fi


exit 0
