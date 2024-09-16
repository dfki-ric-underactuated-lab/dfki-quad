#!/bin/sh

# install Vicon on x86 machines
# skip on ARM, since the DataStream SDK is not available there

if [ "${HW_ARCH}" = "x86_64" ]; then
	cd /root
	git clone https://github.com/CatInTheRain/ros2-vicon-receiver && cd ros2-vicon-receiver && git switch humble && ./install_libs.sh
	/bin/bash -c "source /opt/ros/humble/setup.bash && cd vicon_receiver && colcon build --symlink-install"
	echo 'source /root/ros2-vicon-receiver/vicon_receiver/install/setup.bash' >> ~/.bashrc

elif [ "${HW_ARCH}" = "aarch64" ]; then
	echo "Skipping Vicon Setup on ARM..."
fi

exit 0
