# Usa l'immagine completa di ROS Noetic con desktop e Gazebo Classic, MoveIt, RViz, ecc
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LIBGL_ALWAYS_SOFTWARE=1

# Aggiorna e installa driver UR, pacchetti Gazebo, sensori virtuali, strumenti di test
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-ur-robot-driver \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-plugins \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-moveit \
    ros-noetic-rviz \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-warehouse-ros-mongo \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-industrial-robot-status-interface \
    ros-noetic-effort-controllers \
    python3-catkin-tools \
    python3-vcstool \
    x11-xserver-utils \
    iputils-ping \
    mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Solo update di rosdep
RUN rosdep update

# Crea la catkin workspace e imposta la working directory
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clona i repository essenziali Universal Robots (driver e meta-package)
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
RUN git clone https://github.com/ros-industrial/universal_robot.git

# Torna alla cartella principale della workspace
WORKDIR /catkin_ws

# Aggiorna apt e installa le dipendenze
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Compila la workspace catkin
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Imposta l'ambiente ROS alla partenza
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copia eventuale entrypoint e launch di test
COPY entrypoint.sh /entrypoint.sh

COPY ./Add/grippers/* /catkin_ws/src/universal_robot/ur_gazebo/urdf/grippers/
COPY ./Add/ur_gripper.launch /catkin_ws/src/universal_robot/ur_gazebo/launch/ur_gripper.launch
COPY ./Add/ur3_gripper_endoscope.xacro /catkin_ws/src/universal_robot/ur_gazebo/urdf/ur3_gripper_endoscope.xacro

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

