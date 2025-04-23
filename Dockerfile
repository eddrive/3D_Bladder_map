# Usa l'immagine base di ROS Noetic (Ubuntu 20.04)
FROM ros:noetic-ros-base
ENV LIBGL_ALWAYS_SOFTWARE=1

# Installa gli strumenti di sviluppo, i tool di catkin, i pacchetti base di ROS e il pacchetto x11-xserver-utils
RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-ros-comm \
    x11-xserver-utils \
    iputils-ping \
    mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Crea la cartella della workspace e dei sorgenti
RUN mkdir -p /catkin_ws/src

# Imposta la working directory
WORKDIR /catkin_ws/src

# Clona il repository del driver
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

# Torna alla cartella principale della workspace
WORKDIR /catkin_ws

# Installa le dipendenze tramite rosdep
RUN apt-get update && apt-get install -y python3-rosdep && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Compila la workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Imposta i sourcing per bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copia il file entrypoint nello container
COPY entrypoint.sh /entrypoint.sh

# Rende eseguibile lo script entrypoint
RUN chmod +x /entrypoint.sh

# Imposta l'entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Specifica il comando di default (es. bash)
CMD ["bash"]

