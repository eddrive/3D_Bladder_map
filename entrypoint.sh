#!/bin/bash
set -e

# Imposta variabili per X11 e Qt
export DISPLAY=${DISPLAY:-":0"}
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM="xcb"

# Abilita accesso X11 root e docker (xhost +local:docker va dato sull'host!)
if command -v xhost &> /dev/null; then
    xhost +local:root
fi

# Sourcing ROS e workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Avvia roscore in background
echo "Avvio di roscore in background..."
roscore > /dev/null 2>&1 &
sleep 2

# Pulizia vecchie run_id
rosparam delete /run_id || echo "Nessun run_id presente da eliminare"

# Avvia simulazione UR5 in Gazebo
echo "Avvio simulazione Universal Robot UR3 in Gazebo..."
roslaunch ur_gazebo ur_gripper.launch world:=empty.world &
GAZEBO_PID=$!
sleep 5


# Attesa infinita per mantenere il container attivo (o lascia il controllo a bash o altro comando)
wait

