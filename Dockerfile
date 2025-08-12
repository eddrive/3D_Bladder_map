# CHANGE: Use CUDA base image instead of ROS-only base
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# Prevent interactive prompts during package installation

ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Europe/Rome

# Configure timezone non-interactively

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS2 Humble from source
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble and development tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
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
    wget \
    curl \
    python3-pip \
    git \
    libgl1 \
    libglib2.0-0 \
    && apt-get clean

# Install PyTorch with CUDA 11.8 support (compatible with base image)
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install Python dependencies for MiDaS depth estimation
RUN pip install opencv-python timm einops matplotlib pillow "numpy<2"

# Clone MiDaS repository
RUN git clone https://github.com/isl-org/MiDaS.git /midas
WORKDIR /midas

# Pre-download MiDaS models during build to avoid runtime delays
RUN python3 -c "import torch; torch.hub.load('intel-isl/MiDaS', 'DPT_Hybrid', pretrained=True, trust_repo=True)" || echo "DPT_Hybrid download failed, will retry at runtime"
RUN python3 -c "import torch; torch.hub.load('intel-isl/MiDaS', 'DPT_Large', pretrained=True, trust_repo=True)" || echo "DPT_Large download failed, will retry at runtime"
RUN python3 -c "import torch; torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', pretrained=True, trust_repo=True)" || echo "MiDaS_small download failed, will retry at runtime"

# Create ROS2 workspace
ENV COLCON_WS=/workspace/bladder_mapper
RUN mkdir -p $COLCON_WS/src

# Clone Universal Robots ROS2 Driver
WORKDIR $COLCON_WS
RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver \
    && vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos

# Initialize rosdep (package dependency manager)
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Install ROS2 package dependencies
RUN rosdep install --ignore-src --from-paths src -y --rosdistro humble

# Copy project-specific packages
COPY resources/ur3_endoscope_description $COLCON_WS/src/ur3_endoscope_description
COPY resources/ur3_endoscope_moveit_config $COLCON_WS/src/ur3_endoscope_moveit_config
COPY resources/endoscope_calibration.yaml /root/endoscope_calibration.yaml
COPY resources/endoscope_calibration.yaml /root/endoscope_mask.png
COPY resources/bladder_rtabmapper $COLCON_WS/src/bladder_rtabmapper
COPY resources/midas_depth_ros $COLCON_WS/src/midas_depth_ros

# Set environment variables for MiDaS
ENV MIDAS_PATH=/midas
ENV PYTHONPATH="/midas"

# Source ROS2 setup automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy and setup entrypoint script
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command when container starts
CMD ["bash"]
