# From Docker Hub yahboomtechnology/ros-foxy tag: 3.5.3
FROM yahboomtechnology/ros-foxy:3.5.3

# Set environment variable (this ensures docker image topics can communicate with host)
ENV ROS_DOMAIN_ID=0

# Dummy map files for slam-toolbox in maps dir
COPY maps /root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/maps
# Param files for slam-toolbox and navigation_dwa_launch.py in params dir
COPY params /root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/params
# Modify launch file in laucnh dir
COPY launch /root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/launch

# Working directory /root/yahboomcar_ros2_ws/yahboomcar_ws
WORKDIR /root/yahboomcar_ros2_ws/yahboomcar_ws

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select yahboomcar_nav --symlink-install"

# .bashrc change for ROS_DOMAIN_ID=0
COPY .bashrc /root/.bashrc

# Entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh
ENTRYPOINT ["/root/entrypoint.sh"]
