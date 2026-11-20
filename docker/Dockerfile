# syntax=docker/dockerfile:1
FROM ros:humble-ros-base-jammy

# basic tooling & packages needed
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    ca-certificates \
    gnupg \
    gnuplot \
    lsb-release \
    wget \
    curl \
    git \
    python3-pip \
    python-is-python3 \
    ros-humble-rosbridge-server \
    ros-humble-joy-linux \
    ros-humble-teleop-tools \
    nano \
    vim \
    libserial-dev \
    fontconfig

RUN echo "ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true" | debconf-set-selections

ARG HW_ARCH
ARG GO2_NETWORK_INTERFACE

# install drake - drake_setup.sh determines the underlying hardware architecture
WORKDIR /root
COPY drake_setup.sh /drake_setup.sh
RUN /drake_setup.sh
RUN rm /drake_setup.sh

# vicon requires libboost
RUN apt-get update && apt-get install -y libboost1.74-all-dev
WORKDIR /root
COPY vicon_setup.sh /vicon_setup.sh
RUN /vicon_setup.sh
RUN rm /vicon_setup.sh

# Install additional pip packages from custom requirements.txt
COPY requirements.txt /tmp/
#RUN pip3 install qpsolvers[open_source_solvers] # somehow the [] option is ignored when installed from requirements.txt
RUN pip3 install --requirement /tmp/requirements.txt

# simde provides the SIMD kernels of ProxQP in acados (ACADOS_PROXQP_VECTORIZE), as recommended by ProxSuite.
# ProxQP itself comes with acados, for the MPC and for wbc.
RUN apt-get update && apt-get install -y libsimde-dev

# Install invariant-ekf library from src: https://github.com/RossHartley/invariant-ekf/blob/master/README.md (use franeks fork due to backwards compatibility issues with modern cpp and missing features, see changelog in franeks fork)
WORKDIR /root
RUN git clone https://github.com/FranekStark/invariant-ekf.git --recursive && cd invariant-ekf && git checkout devel
WORKDIR /root/invariant-ekf
RUN mkdir build
WORKDIR /root/invariant-ekf/build
RUN apt-get update && apt-get install -y libboost1.74-all-dev
RUN cmake ..
RUN make -j$(nproc)
RUN make install
# add to ld library path the inkef install dixr
ENV LD_LIBRARY_PATH /usr/local/lib:${LD_LIBRARY_PATH}

# add adapted acados library and install it
WORKDIR /root
# Clarabel is a Rust library, so acados needs a cargo toolchain to build it
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"
# ADD of a git ref rebuilds this layer whenever the branch moves (no GitHub API call, so no rate limit)
ADD --keep-git-dir=true https://github.com/FranekStark/acados.git#comparison_paper_V2 /root/acados
RUN cd acados && git submodule update --recursive --init && mkdir build
WORKDIR /root/acados/build
ENV RUSTFLAGS="-C target-cpu=native"
# acados installs its QP solvers into /opt/acados. wbc calls HPIPM, DAQP, ProxQP, qpOASES (qpOASES_e), OSQP and
# Clarabel through acados, so only one version of each solver ends up in the controller process
RUN cmake ..    -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS="-march=native" -DCMAKE_CXX_FLAGS="-march=native" \
                -DACADOS_WITH_QPOASES=ON \
                -DACADOS_WITH_OSQP=ON -DOSQP_USE_LONG=OFF -DOSQP_ENABLE_PRINTING=OFF \
                -DACADOS_WITH_CLARABEL=ON \
                -DACADOS_WITH_DAQP=ON -DNATIVE_OPTIMIZATION=ON \
                -DACADOS_WITH_PROXQP=ON -DACADOS_PROXQP_VECTORIZE=ON \
                -DACADOS_INSTALL_DIR=/opt/acados -DACADOS_EXAMPLES=OFF && make -j$(nproc) install
ENV LD_LIBRARY_PATH /opt/acados/lib:${LD_LIBRARY_PATH}
ENV ACADOS_SOURCE_DIR "/root/acados"

# Install Intel MKL
WORKDIR /root
COPY mkl_setup.sh /mkl_setup.sh
RUN /mkl_setup.sh
RUN rm /mkl_setup.sh

# Install unitree_sdk2
RUN apt-get update && apt-get install net-tools
WORKDIR /root
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git && cd unitree_sdk2 && git checkout bb983e1 && mkdir build && cd build && cmake .. && make -j$(nproc) install

# Install cyclone dds
WORKDIR /root
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x && cd cyclonedds && mkdir build && cd build && cmake .. && cmake --build . && cmake --build . --target install
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git -b releases/0.10.x && cd cyclonedds-cxx && mkdir build && cd build && cmake .. && cmake --build . && cmake --build . --target install

# Install unitree_ros2
WORKDIR /root
RUN git clone https://github.com/unitreerobotics/unitree_ros2 && cd unitree_ros2 && git checkout 055e476
RUN apt-get update && apt-get install ros-humble-rmw-cyclonedds-cpp -y && apt-get install ros-humble-rosidl-generator-dds-idl -y
WORKDIR /root/unitree_ros2/cyclonedds_ws/src
RUN git clone https://github.com/ros2/rmw_cyclonedds -b humble
WORKDIR /root/unitree_ros2/cyclonedds_ws
# RUN colcon build --cmake-args -DBUILD_DDSPERF=NO --packages-select cyclonedds
RUN colcon build --packages-select cyclonedds
WORKDIR /root/unitree_ros2/
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DBUILD_DDSPERF=NO"
RUN sed -i '$ d' /root/unitree_ros2/example/src/CMakeLists.txt # remove last line
RUN echo " add_library(common src/common/motor_crc.cpp src/common/ros2_sport_client.cpp) \n \
           ament_target_dependencies(common PUBLIC unitree_go unitree_api rclcpp) \n \
           ament_export_targets(common HAS_LIBRARY_TARGET) \n \
           install(DIRECTORY include/ DESTINATION include) \n \
           install( \n \
           TARGETS common \n \
           EXPORT common \n \
           LIBRARY DESTINATION lib \n \
           ARCHIVE DESTINATION lib \n \
           RUNTIME DESTINATION bin \n \
           INCLUDES DESTINATION include) \n \
           ament_package()" >> /root/unitree_ros2/example/src/CMakeLists.txt
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build" # replace last line to install needed functions

# install matplot++ \
WORKDIR /root
RUN git clone --branch v1.2.1 https://github.com/alandefreitas/matplotplusplus/
RUN cd matplotplusplus && cmake --preset=system && cmake --build --preset=system && cmake --install build/system

# install fmt lib
WORKDIR /root
RUN git clone --recursive https://github.com/fmtlib/fmt.git && cd fmt && git checkout 8.1.1 && mkdir build && cd build && cmake .. && make install

# install arc-opt
RUN apt-get update && apt-get -y install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev
RUN apt-get update && apt-get -y install liburdfdom-headers-dev liburdfdom-dev
ADD --keep-git-dir=true https://github.com/jmiddelberg/wbc.git#comparison_paper_V2 /root/wbc
RUN git clone --branch v2.6.8 --recurse-submodules https://github.com/stack-of-tasks/pinocchio.git && cd pinocchio && mkdir build && cd build && cmake .. -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install
# HPIPM, DAQP, ProxQP, qpOASES, OSQP and Clarabel come from acados (wbc calls them through acados). eiquadprog and
# qpSWIFT are not in acados, so they are installed here.
RUN git clone --recurse-submodules https://github.com/stack-of-tasks/eiquadprog.git -b v1.2.5 && cd eiquadprog && cp ../wbc/patches/eiquadprog.patch . && git apply eiquadprog.patch && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install
RUN git clone https://github.com/qpSWIFT/qpSWIFT.git && cd qpSWIFT && git checkout 24608b6 && cp ../wbc/patches/qpSWIFT.patch . && git apply qpSWIFT.patch && mkdir build && cd build && cmake .. && make -j$(nproc) && make install

RUN mkdir wbc/build && cd wbc/build && cmake .. -DROBOT_MODEL_RBDL=OFF -DROBOT_MODEL_KDL=OFF -DSOLVER_DAQP=ON -DSOLVER_CLARABEL=ON -DSOLVER_PROXQP=ON -DSOLVER_EIQUADPROG=ON -DSOLVER_QPSWIFT=ON -DSOLVER_OSQP=ON -DSOLVER_HPIPM=ON -DCMAKE_BUILD_TYPE=RELEASE && make -j$(nproc) && make install
RUN ldconfig

# Install NMEA GPS driver
RUN apt-get update && apt-get -y install ros-humble-nmea-navsat-driver && pip3 install transforms3d

RUN echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /root/unitree_ros2/install/setup.bash' >> ~/.bashrc

# Install Vicon driver
#WORKDIR /root
#RUN git clone https://github.com/CatInTheRain/ros2-vicon-receiver && cd ros2-vicon-receiver && git switch humble && ./install_libs.sh
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ros2-vicon-receiver/vicon_receiver && colcon build --symlink-install "
#RUN echo 'source /root/ros2-vicon-receiver/vicon_receiver/install/setup.bash' >> ~/.bashrc

# Install plotjuggler
RUN apt-get update && apt-get -y install ros-humble-plotjuggler-ros

# create environments for both robots
WORKDIR /root
RUN touch setup_go2_workspace.bash && touch setup_ulab_workspace.bash
RUN echo 'source /root/ros2_ws/install/setup.bash' >> setup_go2_workspace.bash && echo 'source /root/ros2_ws/install/setup.bash' >> setup_ulab_workspace.bash
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> setup_go2_workspace.bash
# build argument can be exchanged during build with --build-arg GO2_NETWORK_INTERFACE=<network_name>
RUN echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>'" >> setup_go2_workspace.bash
#Add ROS_DOMAIN_ID same as hardware
RUN echo 'export ROS_DOMAIN_ID=0' >> setup_go2_workspace.bash
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> setup_ulab_workspace.bash
RUN echo 'export ROS_DOMAIN_ID=100' >> setup_ulab_workspace.bash

RUN echo 'alias sr="source /root/setup_ulab_workspace.bash"' >> ~/.bashrc
RUN echo 'alias sg="source /root/setup_go2_workspace.bash"' >> ~/.bashrc
RUN echo 'alias cbr="colcon build --symlink-install --cmake-args -DROBOT_NAME=ulab"' >> ~/.bashrc
RUN echo 'alias cbg="colcon build --symlink-install --cmake-args -DROBOT_NAME=go2"' >> ~/.bashrc
RUN echo 'alias cbg_onboard="colcon build --symlink-install --cmake-args -DROBOT_NAME=go2 -DWITHOUT_DRAKE=ON -DWITH_VICON=OFF && sg"' >> ~/.bashrc

RUN export ROS_DOMAIN_ID=24

# default workdir
WORKDIR /root/ros2_ws
