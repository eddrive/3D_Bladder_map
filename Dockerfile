FROM osrf/ros:humble-desktop

# Install general ROS2 dependencies and development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-desktop \
    ros-humble-ros2controlcli \
    build-essential \
    cmake \
    ros-humble-ros-workspace \
    nano \
    tree \
    x11-xserver-utils \
    && apt-get clean

# Create ROS2 workspace
ENV COLCON_WS=/workspace/ros_ur_driver
RUN mkdir -p $COLCON_WS/src

# Clone the Universal Robots ROS2 Driver repository
WORKDIR $COLCON_WS
RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver \
    && vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos

# Install ROS2 dependencies
RUN rosdep update && \
    rosdep install --ignore-src --from-paths src -y --rosdistro humble

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy the entrypoint script from the local directory to the container
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command
CMD ["bash"]