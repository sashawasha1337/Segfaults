#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="foxy"

# Source ROS 2 setup
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/install/setup.bash"
else
  echo "ERROR: ROS $ROS_DISTRO not found under /opt/ros/$ROS_DISTRO" >&2
  exit 1
fi

# Install system dependencies for PCL
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# Navigate back to the workspace root
cd /root/ros2_ws

# Install ROS2 dependencies for all packages
echo "Installing ROS 2 dependencies..."
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

cd /root/ros2_ws

# Fix PCL warning - this needs to come after rosdep install
echo "Fixing PCL warnings..."
find /usr/include/pcl* -path "*/sample_consensus/impl/sac_model_plane.hpp" -exec sed -i 's/^\(\s*\)PCL_ERROR ("\[pcl::SampleConsensusModelPlane::isSampleGood\] Sample points too similar or collinear!\\n");/\1\/\/ PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\\n");/' {} \;

# Final build of everything
colcon build
source install/setup.bash

echo "Workspace setup completed!"
