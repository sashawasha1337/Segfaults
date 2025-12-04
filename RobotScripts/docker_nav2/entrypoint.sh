#!/bin/bash

#source /opt/ros/foxy/setup.bash

#source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
#source /root/.bashrc

# Environment variables
alias python=python3
export ROS_DOMAIN_ID=0
export ROBOT_TYPE=x3   # r2, x1, x3
export RPLIDAR_TYPE=a1 # a1, s2
export CAMERA_TYPE=astraplus # astrapro, astraplus

echo "--------------------------------------------------------"
echo -e "ROS_DOMAIN_ID: \033[32m$ROS_DOMAIN_ID\033[0m"
echo -e "my_robot_type: \033[32m$ROBOT_TYPE\033[0m | my_lidar: \033[32m$RPLIDAR_TYPE\033[0m | my_camera: \033[32m$CAMERA_TYPE\033[0m"
echo "--------------------------------------------------------"

# Colcon helpers
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/root/yahboomcar_ros2_ws/yahboomcar_ws
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS2
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash

# ORB-SLAM2
export ORB_SLAM2_ROOT_DIR=/root/yahboomcar_ros2_ws/software/orbslam2/ORB_SLAM2-master
export LD_LIBRARY_PATH=/root/yahboomcar_ros2_ws/software/orbslam2/Pangolin-0.6/build/src/:/root/yahboomcar_ros2_ws/software/orbslam2/ORB_SLAM2-master/Thirdparty/DBoW2/lib:/root/yahboomcar_ros2_ws/software/orbslam2/ORB_SLAM2-master/Thirdparty/g2o/lib:/root/yahboomcar_ros2_ws/software/orbslam2/ORB_SLAM2-master/lib:$LD_LIBRARY_PATH


if [ $# -eq 0 ]; then
    echo "[Entrypoint] Launching Nav2"
    exec ros2 launch yahboomcar_nav nav2_launch.py
else
    exec "$@"
fi
