#!/bin/bash
set -e

cleanup() {
    echo
    echo "=== Stopping Docker containers ==="
    [ -n "$NAV2_ID" ] && sudo docker stop "$NAV2_ID"
    [ -n "$YOLO_ID" ] && sudo docker stop "$YOLO_ID"

    echo "=== Stopping host ROS2 processes ==="
    [ -n "$ROS2_PID" ] && kill $ROS2_PID
    [ -n "$ROSMASTER_PID" ] && kill $ROSMASTER_PID

    echo "=== Exiting ==="
    exit 0
}

trap cleanup INT

echo "=== Starting NAV2 Docker ==="
cd ~/ros2_ws/docker_nav2 || exit 1

NAV2_ID=$(sudo docker run -d \
  --memory=1g \
  --memory-swap=4g \
  --net=host \
  --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/jetson/temp:/root/yahboomcar_ros2_ws/temp \
  -v /home/jetson/rosboard:/root/rosboard \
  -v /home/jetson/maps:/root/maps \
  -v /dev/bus/usb/001/010:/dev/bus/usb/001/010 \
  -v /dev/bus/usb/001/011:/dev/bus/usb/001/011 \
  --device=/dev/myserial \
  --device=/dev/rplidar \
  --device=/dev/input \
  segfaults_yahboom_nav2:1.0)

echo "NAV2 container ID: $NAV2_ID"
sleep 3

echo "=== Starting YOLO / Robot Docker Container ==="
cd ~/ros2_ws || exit 1

YOLO_ID=$(sudo docker run -d \
  --memory=1g \
  --memory-swap=4g \
  --runtime nvidia \
  --network host \
  --ipc=host \
  --device /dev/video0:/dev/video0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -e FIREBASE_STORAGE_BUCKET="segfaults-database.firebasestorage.app" \
  -e ROBOT_ID="$(hostname)" \
  -e ROBOT_UID="$(hostname)" \
  -v "$PWD/src:/ros2_ws/src:rw" \
  -v "$PWD/robot-service-account.json:/ros2_ws/secrets/robot-service-account.json:ro" \
  -v "$PWD/Model:/Model:ro" \
  -v ros2_build:/ros2_ws/build \
  -v ros2_install:/ros2_ws/install \
  -v ros2_log:/ros2_ws/log \
  nano-ros-foxy-yolo:latest)

echo "YOLO container ID: $YOLO_ID"
sleep 5

echo "=== Launching ROS2 full launch on host ==="
source /opt/ros/eloquent/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_launch full_launch_eloquent.py &
ROS2_PID=$!
sleep 2

echo "=== Running Rosmaster Motor Node on host ==="
cd ~/ros2_ws
python3 RosmasterMotorNode.py &
ROSMASTER_PID=$!

wait

