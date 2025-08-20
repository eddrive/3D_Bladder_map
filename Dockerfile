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

# CHANGE: Install Python dependencies for Depth-Anything V2 instead of MiDaS
RUN pip install opencv-python timm einops matplotlib pillow "numpy<2" transformers huggingface-hub

# CHANGE: Clone Depth-Anything V2 repository instead of MiDaS
RUN git clone https://github.com/DepthAnything/Depth-Anything-V2.git /depth_anything_v2
WORKDIR /depth_anything_v2

# CHANGE: Create checkpoints directory and download Depth-Anything V2 models
RUN mkdir -p /depth_anything_v2/checkpoints
RUN cd /depth_anything_v2/checkpoints && \
    wget -O depth_anything_v2_vits.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth || echo "vits download failed" && \
    wget -O depth_anything_v2_vitb.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Base/resolve/main/depth_anything_v2_vitb.pth || echo "vitb download failed" && \
    wget -O depth_anything_v2_vitl.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth || echo "vitl download failed"

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
RUN apt-get update && rosdep install --ignore-src --from-paths src -y --rosdistro humble


# Copy project-specific packages
COPY resources/ur3_endoscope_description $COLCON_WS/src/ur3_endoscope_description
COPY resources/ur3_endoscope_moveit_config $COLCON_WS/src/ur3_endoscope_moveit_config
COPY resources/endoscope_calibration.yaml /root/endoscope_calibration.yaml
COPY resources/endoscope_mask.png /root/endoscope_mask.png
COPY resources/bladder_rtabmapper $COLCON_WS/src/bladder_rtabmapper
# CHANGE: Copy depth_anything package instead of midas_depth_ros
COPY resources/depth_anything $COLCON_WS/src/depth_anything

# CHANGE: Set environment variables for Depth-Anything V2 instead of MiDaS
ENV DEPTH_ANYTHING_V2_PATH=/depth_anything_v2
ENV PYTHONPATH="/depth_anything_v2"

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