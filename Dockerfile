FROM osrf/ros:humble-desktop

# Install general ROS2 dependencies and development
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
    ros-humble-moveit-setup-assistant \
    ros-humble-v4l2-camera \
    ros-humble-image-tools \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rqt-tf-tree \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-msgs \
    graphviz \
    && apt-get clean

# Install pip and Python dependencies
RUN apt-get update && apt-get install -y python3-pip git libgl1 libglib2.0-0
RUN pip install torch torchvision opencv-python

# Clone MiDaS repository
RUN git clone https://github.com/isl-org/MiDaS.git /midas \
    && mkdir -p /midas/weights \
    && curl -L -o /midas/weights/dpt_hybrid-midas-501f0c75.pt https://github.com/isl-org/MiDaS/releases/download/v3_1/dpt_hybrid-midas-501f0c75.pt

# Create ROS2 workspace
ENV COLCON_WS=/workspace/bladder_mapper
RUN mkdir -p $COLCON_WS/src

# Clone the Universal Robots ROS2 Driver repository
WORKDIR $COLCON_WS
RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver \
    && vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos

# Install ROS2 dependencies
RUN rosdep update && \
    rosdep install --ignore-src --from-paths src -y --rosdistro humble

# Copy the UR3 endoscope description files
COPY resources/ur3_endoscope_description $COLCON_WS/src/ur3_endoscope_description
COPY resources/ur3_endoscope_moveit_config $COLCON_WS/src/ur3_endoscope_moveit_config
COPY resources/endoscope_calibration.yaml /root/endoscope_calibration.yaml
COPY resources/bladder_rtabmapper $COLCON_WS/src/bladder_rtabmapper
COPY resources/midas_depth_ros $COLCON_WS/src/midas_depth_ros


# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy the entrypoint script
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command
CMD ["bash"]