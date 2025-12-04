#!/bin/bash
set -e

cleanup() {
    echo
    echo "=== Cleaning up ==="

    echo "Stopping YOLO Docker…"
    [ -n "$YOLO_ID" ] && sudo docker stop "$YOLO_ID" >/dev/null 2>&1

    echo "Stopping host ROS2 launch…"
    if [ -n "$ROS2_PID" ]; then
        kill "$ROS2_PID" >/dev/null 2>&1
        sleep 1
        if ps -p "$ROS2_PID" > /dev/null; then
            kill -9 "$ROS2_PID" >/dev/null 2>&1
        fi
        wait "$ROS2_PID" 2>/dev/null || true
    fi

    echo "Stopping RosmasterMotorNode.py…"
    if [ -n "$ROSMASTER_PID" ]; then
        kill "$ROSMASTER_PID" >/dev/null 2>&1
        sleep 1
        if ps -p "$ROSMASTER_PID" > /dev/null; then
            kill -9 "$ROSMASTER_PID" >/dev/null 2>&1
        fi
        wait "$ROSMASTER_PID" 2>/dev/null || true
    fi

    echo "Killing any leftover Rosmaster processes locking serial…"
    sudo pkill -f "RosmasterMotorNode.py" >/dev/null 2>&1 || true
    sudo pkill -f "python3.*ttyUSB" >/dev/null 2>&1 || true

    echo "=== Cleanup complete ==="
    exit 0
}

trap cleanup INT

if lsof /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "Error: /dev/ttyUSB0 is busy. Kill leftover processes before starting."
    lsof /dev/ttyUSB0
    exit 1
fi

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

echo "=== Launching full ROS2 stack ==="
source /opt/ros/eloquent/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_launch yolo_control_launch.py &
ROS2_PID=$!
sleep 5  # wait for ROS2 launch to initialize

echo "=== Running Rosmaster Motor Node ==="
cd ~/ros2_ws
python3 RosmasterMotorNode.py &
ROSMASTER_PID=$!

wait

