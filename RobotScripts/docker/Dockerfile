# Use the official ROS 2 Jazzy core image (Ubuntu Noble)
FROM ros:jazzy-ros-core-noble

# Install essential build tools and ROS utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    curl \
    gnupg \
    libpcap-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update

# Set up colcon mixins and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Create and set workspace directory
WORKDIR /ros2_ws

# Copy the source packages and scripts into the workspace
COPY src ./src

COPY docker/workspace.sh ./workspace.sh
COPY docker/entrypoint.sh ./entrypoint.sh

# Make sure scripts are executable
RUN chmod +x /ros2_ws/*.sh

# Install dependencies and build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source the workspace on container start
ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
