#!/bin/bash

echo "Starting Experiment $1."

echo "Creating folder for $1"

if [[ $(uname -i) = "aarch64" ]]
then
    mkdir /media/external-drive/$1
    git rev-parse HEAD > /media/external-drive/$1/git-hash.txt
    cd /media/external-drive/$1
else
    mkdir $1
    git rev-parse HEAD > $1/git-hash.txt
    cd $1
fi

echo "Dumping ROS parameters."
mkdir param-files && cd param-files
ros2 param dump /mit_controller_node > mit_controller_node.yaml
ros2 param dump /state_estimation_node > state_estimation_node.yaml
ros2 param dump /leg_driver > leg_driver.yaml
ros2 param dump /disturbance_node > disturbance_node.yaml
cd ..

echo "Which topics would you like to record? Please choose a preset."
echo "1: State Estimation test - /quad_state, /joint_states, /imu_measurement, /contact_state, /tcpfix, /tcpvel, /tcptime, /gait_state"
echo "2: Gait Sequencer test - /quad_state, /quad_control_target, /battery_state, /gait_sequence, /gait_state, /tcpfix, /tcpvel, /tcptime"
echo "3: Adaptive MPC test - /quad_state, /quad_model_update, quad_control_target,..."
echo "4: TOTAL RECORDING "

read preset

if [[ $preset == 1 ]]
then
    echo "Recording State Estimation test..."
    ros2 bag record -d 60 /quad_state /joint_states /imu_measurement /contact_state /tcpfix /tcpvel /tcptime /gait_state
elif [[ $preset == 2 ]]
then
    echo "Recording Gait Sequencer test..."
    ros2 bag record -d 60 /quad_state /quad_control_target /battery_state /gait_sequence /gait_state /tcpfix /tcpvel /tcptime
elif [[ $preset == 3 ]]
then
    echo "Recording Adaptive MPC test..."
    ros2 bag record -d 60 /quad_state /quad_model_update /quad_control_target
elif [[ $preset == 4 ]]
then
    echo "TOAL RECORDING..."
    ros2 bag record -d 60 /quad_state /quad_model_update /quad_control_target /quad_state /joint_states /imu_measurement /contact_state /fix /vel /time_reference /joint_cmd /wbc_target /battery_state /gait_sequence /gait_state /solve_time /wbc_solve_time /cpu_power_consumption /controller_hartbeat /open_loop_trajectory /vicon/unitree_go2/unitree_go2
else
    echo "Invalid Input"
    exit 1
fi
echo "Experiment Finished!"
exit 0

