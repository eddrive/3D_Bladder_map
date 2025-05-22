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
RUN git clone -b noetic https://github.com/ros-industrial/universal_robot.git

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

# Crea il package customizzato my_ur3_setup (STEP 1 e 2)
WORKDIR /catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg my_ur3_setup rospy std_msgs xacro ur_description gazebo_ros"
RUN mkdir -p /catkin_ws/src/my_ur3_setup/urdf/grippers \
    && mkdir -p /catkin_ws/src/my_ur3_setup/urdf \
    && mkdir -p /catkin_ws/src/my_ur3_setup/launch \
    && mkdir -p /catkin_ws/src/my_ur3_setup/config

# Copia i tuoi file custom dal contesto host nella posizione giusta
COPY ./Add/endoscope.xacro /catkin_ws/src/my_ur3_setup/urdf/endoscope.xacro
COPY ./Add/ur.xacro /catkin_ws/src/universal_robot/ur_gazebo/urdf/ur.xacro
COPY ./Add/endo_moveit_config/ /catkin_ws/src/endo_moveit_config/
# (Aggiungi altri file eventuali nelle cartelle giuste.)

# Torna nella root del workspace e ricompila
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Sourcing automatico dell'ambiente ROS catkin
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copia l'entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


