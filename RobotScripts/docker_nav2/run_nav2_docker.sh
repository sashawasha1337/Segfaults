#!/bin/bash
#xhost +

docker run -it \
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
segfaults_yahboom_nav2:1.0
